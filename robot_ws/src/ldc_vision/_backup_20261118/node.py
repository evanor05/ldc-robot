"""ROS 3.14 perception coordinator; Torch stays in the isolated worker process."""
from collections import deque
import copy
from pathlib import Path
import json
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from .gateway import PreviewGateway
from .results import checked_detections, TRAFFIC_CLASSES
from .worker_client import WorkerClient, WorkerFrameError, worker_restart_delay
from .tracking import MultiObjectTracker
from . import traffic_color

MAX_WORKER_RESTARTS = 5
# 交通灯观测是安全决策输入（非显示流）：RELIABLE 避免绿灯确认帧被静默丢弃。
# 状态机本身对丢帧鲁棒（连续 N 帧+间隔超时），但不应主动丢。
TRAFFIC_OBSERVATION_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                     history=HistoryPolicy.KEEP_LAST)


class Detector(Node):
    def __init__(self):
        super().__init__('ldc_detector')
        defaults=dict(model_path=str(Path.home()/'.local/share/ldc/models/yolo26n.pt'),
                      worker_python='/home/<user>/esp/yolo-bench-env/bin/python', device='0',
                      imgsz=640, confidence=.25, gateway_port=18320, class_filter='all',
                      tracking_enabled=True, track_confirm_frames=2, track_max_missed=3,
                      track_min_score=.35, track_iou_threshold=.3,
                      color_mode='opencv', color_label_map='')
        for k,v in defaults.items():self.declare_parameter(k,v)
        cfg={k:self.get_parameter(k).value for k in defaults}
        if not 0<cfg['confidence']<=1 or not 32<=cfg['imgsz']<=1280 or cfg['device'] not in ('0','cpu'):
            raise ValueError('推理参数无效')
        if not 0<cfg['track_min_score']<=1 or not 0<cfg['track_iou_threshold']<=1 or cfg['track_confirm_frames']<1 or cfg['track_max_missed']<1:
            raise ValueError('跟踪参数无效')
        self.stop=threading.Event()
        self.condition=threading.Condition()
        self.raw_frame=self.annotated=self.pending=None
        self.session=uuid.uuid4().hex
        self.sequence=self.received=self.completed=self.dropped=0
        self.last_stamp=0
        self.latencies=deque(maxlen=1024)
        self.finished_at=deque(maxlen=512)
        self.worker_error='模型正在加载'
        self.worker_ready=False
        self.worker_restarts=0
        self.worker_failures=0
        self.infer_errors=0
        self.last_frame_error=''
        self.shutdown_requested=False
        self.had_detection=False
        self.names={}
        self.model_sha256=''
        self.camera_detail='等待摄像头数据'
        self.cfg=cfg
        self.filter_mode=cfg['class_filter']
        if self.filter_mode not in ('all','traffic'):raise ValueError('类别过滤无效')
        if cfg['color_mode'] not in traffic_color.COLOR_MODES:raise ValueError('判色模式无效')
        self.color_mode=cfg['color_mode']
        self.color_labels={}
        self.detection_pub=self.create_publisher(Detection2DArray,'/vision/detections',SENSOR_QOS)
        self.traffic_pub=self.create_publisher(String,'/vision/traffic_lights',TRAFFIC_OBSERVATION_QOS)
        self.stream_state=None
        self.stream_lost_at=0.
        self.annotated_pub=self.create_publisher(CompressedImage,'/vision/annotated/compressed',SENSOR_QOS)
        self.raw_annotation_pub=self.create_publisher(Image,'/vision/annotated',SENSOR_QOS)
        self.diag_pub=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        # 相机订阅用更深队列：depth=1 在发布端快于回调时会直接丢帧，
        # 实测把 17.7Hz 的上游削到 12.6Hz。depth=5 允许少量缓冲，减少无谓丢帧。
        CAMERA_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.on_frame,CAMERA_QOS)
        self.create_subscription(DiagnosticArray,'/diagnostics',self.on_diagnostics,10)
        self.create_timer(.5,self.publish_diagnostics)
        self.tracker_lock=threading.Lock()
        self.tracker=MultiObjectTracker(iou_threshold=cfg['track_iou_threshold'],
                                        confirm_frames=cfg['track_confirm_frames'],
                                        max_missed=cfg['track_max_missed'],
                                        min_score=cfg['track_min_score'])
        self.worker=self._new_worker()
        self.gateway=PreviewGateway(self,cfg['gateway_port'])
        self.thread=threading.Thread(target=self.inference_loop,daemon=True)
        self.thread.start()

    def _new_worker(self):
        cfg=self.cfg
        return WorkerClient(cfg['worker_python'],cfg['model_path'],cfg['device'],cfg['imgsz'],cfg['confidence'],
                            color_mode=cfg['color_mode'],color_label_map=cfg['color_label_map'])

    def on_diagnostics(self,msg):
        for status in msg.status:
            if status.name=='ldc_camera':
                with self.condition:self.camera_detail=status.message

    def on_frame(self,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        with self.condition:
            if not 0<len(msg.data)<=4*1024*1024 or msg.format.lower() not in ('jpeg','jpg') or stamp<=0 or not -.25<=age<=1 or stamp<=self.last_stamp:
                self.dropped+=1
                # 重复旧帧：单独观测（与「新画面无灯」区分）。发布器只在真实节点
                # 上存在；单元测试用最小伪对象驱动 on_frame，按需跳过。
                publish=getattr(self,'_publish_traffic',None)
                if publish is not None and stamp>0 and stamp<=self.last_stamp:
                    publish(traffic_color.status_observation(
                        'stale_frame',stamp_ns=stamp,last_stamp_ns=self.last_stamp,at=time.monotonic()))
                return
            self.last_stamp=stamp
            self.sequence+=1;self.received+=1
            if self.pending is not None:self.dropped+=1
            sample=dict(id=f'{self.session}:{self.sequence}',jpeg=bytes(msg.data),
                        header=copy.deepcopy(msg.header),at=time.monotonic()-max(0,age),sequence=self.sequence)
            self.raw_frame=sample
            self.pending=sample
            self.condition.notify_all()

    @staticmethod
    def header_dict(header):
        return dict(sec=header.stamp.sec,nanosec=header.stamp.nanosec,frame_id=header.frame_id)

    def _publish_traffic(self,record):
        """发布交通灯观测（JSON）。非画面事件按 1Hz 节流，画面帧不节流。"""
        kind=record.get('kind')
        if kind!='frame':
            now=time.monotonic()
            if now-record.get('at',now)<1 and kind!='stream_lost' and kind!='stream_recovered':
                return
        try:
            self.traffic_pub.publish(String(data=json.dumps(record,ensure_ascii=False)))
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f'交通灯观测发布失败：{error}')

    def _observe_stream(self,now):
        """摄像头断流/恢复检测：按 camera_ok 变化发事件，lost 期间 1Hz 重发。"""
        with self.condition:
            raw=self.raw_frame
        camera_ok=raw is not None and now-raw['at']<=1
        if camera_ok:
            if self.stream_state is False:
                self.stream_state=True
                self._publish_traffic(traffic_color.status_observation(
                    'stream_recovered',at=now,lost_seconds=round(now-self.stream_lost_at,3)))
        else:
            if self.stream_state is not False:
                self.stream_state=False;self.stream_lost_at=now
                self._publish_traffic(traffic_color.status_observation('stream_lost',at=now))
            elif now-self.stream_lost_at>=1:
                self.stream_lost_at=now
                self._publish_traffic(traffic_color.status_observation(
                    'stream_lost',at=now,repeat=True))

    def _reset_tracker(self):
        with self.tracker_lock:
            self.tracker.reset()

    def _tracked(self, detections, frame_index):
        if not self.cfg['tracking_enabled']:
            return [dict(item) for item in detections]
        with self.tracker_lock:
            return self.tracker.update(detections, frame_index)

    def _process_sample(self, worker, sample):
        header=self.header_dict(sample['header'])
        result=worker.infer(dict(id=sample['id'],header=header),sample['jpeg'])
        if self.stop.is_set():return
        if time.monotonic()-sample['at']>1:
            self.dropped+=1;return
        try:
            frame=cv2.imdecode(np.frombuffer(sample['jpeg'],np.uint8),cv2.IMREAD_COLOR)
            if frame is None:raise ValueError('来源 JPEG 无法解码')
            height,width=frame.shape[:2]
            detections=checked_detections(result,sample['id'],header,width,height)
            with self.condition:
                mode=self.filter_mode
            selected=[d for d in detections if mode=='all' or d['name'] in TRAFFIC_CLASSES]
            tracked=self._tracked(selected,sample['sequence'])
            array=Detection2DArray(header=sample['header'])
            for item in tracked:
                x1,y1,x2,y2=item['xyxy']
                detection=Detection2D(header=sample['header'])
                hypothesis=ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id=item['name'];hypothesis.hypothesis.score=item['score']
                # This is 2D only: no tracking ID or made-up 3D location.
                detection.results=[hypothesis]
                detection.bbox.center.position.x=(x1+x2)/2;detection.bbox.center.position.y=(y1+y2)/2
                detection.bbox.size_x=x2-x1;detection.bbox.size_y=y2-y1
                array.detections.append(detection)
                cv2.rectangle(frame,(int(x1),int(y1)),(int(x2),int(y2)),(0,230,80),2)
                label=f"{item['name']} {item['score']:.2f}"
                cv2.putText(frame,label,(int(x1),max(15,int(y1)-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,230,80),1)
            ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
            if not ok:raise ValueError('叠框图编码失败')
        except Exception as error:
            # A bad JPEG / malformed model output is a frame-level failure.
            # Keep the worker session alive and process the next frame.
            self.infer_errors+=1;self.last_frame_error=str(error);self.dropped+=1
            self.get_logger().warning(f'YOLO 单帧失败，已跳过：{error}')
            return
        with self.condition:
            # A filter change cannot republish an annotation with the old selection.
            if mode!=self.filter_mode or time.monotonic()-sample['at']>1:return
            self.detection_pub.publish(array)
            lights=[traffic_color.light_record(item) for item in tracked
                    if traffic_color.is_traffic_light_name(item['name'],self.color_labels)]
            published_at=time.monotonic()
            self._publish_traffic(traffic_color.frame_observation(
                session=self.session,frame_id=sample['id'],sequence=sample['sequence'],
                header=header,width=width,height=height,color_mode=self.color_mode,
                lights=lights,captured_at=sample['at'],published_at=published_at,
                wall_at=time.time()))
            self.had_detection=True
            self.annotated_pub.publish(CompressedImage(header=sample['header'],format='jpeg',data=jpeg.tobytes()))
            if self.raw_annotation_pub.get_subscription_count():
                rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                self.raw_annotation_pub.publish(Image(header=sample['header'],height=height,width=width,
                                                     encoding='rgb8',step=width*3,data=rgb.tobytes()))
            self.annotated={**sample,'jpeg':jpeg.tobytes(),'detections':tracked,'width':width,'height':height}
            self.completed+=1
            self.worker_failures=0
            self.last_frame_error=''
            self.latencies.append((time.monotonic()-sample['at'])*1000)
            self.finished_at.append(time.monotonic())

    def inference_loop(self):
        try:
            while not self.stop.is_set():
                worker=self.worker
                try:
                    ready=worker.start()
                    with self.condition:
                        self.names=ready['names'];self.model_sha256=ready['model_sha256']
                        self.color_labels=traffic_color.has_color_labels(self.names)
                        self.worker_ready=True;self.worker_error=''
                    self._reset_tracker()
                    while not self.stop.is_set():
                        with self.condition:
                            self.condition.wait_for(lambda:self.pending is not None or self.stop.is_set(),timeout=.2)
                            sample,self.pending=self.pending,None
                        if not sample:continue
                        if time.monotonic()-sample['at']>1:self.dropped+=1;continue
                        try:
                            self._process_sample(worker,sample)
                        except WorkerFrameError as error:
                            self.infer_errors+=1;self.last_frame_error=str(error);self.dropped+=1
                            self.get_logger().warning(f'YOLO 单帧失败，继续处理后续帧：{error}')
                except Exception as error:
                    if self.stop.is_set():break
                    self.worker_failures+=1
                    with self.condition:
                        self.worker_ready=False;self.annotated=None
                        if self.worker_failures>MAX_WORKER_RESTARTS:
                            self.worker_error=(f'YOLO worker 连续失败 {self.worker_failures} 次，'
                                               f'已达重启上限：{error}')
                            break
                        delay=worker_restart_delay(self.worker_failures)
                        self.worker_error=(f'YOLO worker 异常：{error}；{delay:.1f}s 后'
                                           f'第 {self.worker_failures}/{MAX_WORKER_RESTARTS} 次重启')
                    self.get_logger().error(f'YOLO worker 异常，准备重启：{error}')
                    with self.condition:
                        if self.stop.is_set():break
                        old=self.worker;self.worker=self._new_worker();self.worker_restarts+=1
                    old.close()
                    self.stop.wait(delay)
        except Exception as error:
            if not self.stop.is_set():
                with self.condition:
                    self.worker_ready=False;self.worker_error=str(error);self.annotated=None
                self.get_logger().error(f'YOLO 已停止；原图仍可查看：{error}')
        finally:
            with self.condition:
                worker=self.worker
            worker.close()

    def set_filter(self,mode):
        if mode not in ('all','traffic'):raise ValueError('类别过滤仅支持 all/traffic')
        with self.condition:
            changed=mode!=self.filter_mode
            if changed:self.annotated=None
            self.filter_mode=mode
        if changed:self._reset_tracker()

    def preview(self,kind):
        with self.condition:
            frame=self.raw_frame if kind=='raw' else self.annotated
            if frame is None or time.monotonic()-frame['at']>1:return None
            return frame['sequence'],frame['jpeg']

    def status(self):
        now=time.monotonic()
        with self.condition:
            raw,annotated=self.raw_frame,self.annotated
            camera_ok=raw is not None and now-raw['at']<=1
            detector_ok=camera_ok and self.worker_ready and annotated is not None and now-annotated['at']<=1
            times=[t for t in self.finished_at if now-t<=10]
            fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0
            latencies=sorted(self.latencies)
            with self.tracker_lock:tracking_stats=self.tracker.stats()
            tracking=dict(enabled=bool(self.cfg['tracking_enabled']),**tracking_stats)
            return dict(service='ldc_vision',model_sha256=self.model_sha256,running=True,filter=self.filter_mode,device=self.cfg['device'],timestamp_source='host_receive',
                        camera=dict(ok=camera_ok,detail=self.camera_detail,ageSeconds=round(now-raw['at'],3) if raw else None),
                        detector=dict(ok=bool(detector_ok),detail=self.worker_error or ('正在识别' if detector_ok else '等待有效新帧'),
                                      workerReady=self.worker_ready,restarts=self.worker_restarts,
                                      restartFailures=self.worker_failures,inferErrors=self.infer_errors,
                                      lastFrameError=self.last_frame_error),
                        frame=dict(id=annotated['id'],header=self.header_dict(annotated['header']),
                                   width=annotated['width'],height=annotated['height']) if detector_ok else None,
                        detections=annotated['detections'] if detector_ok else [],
                        tracking=tracking,
                        received=self.received,completed=self.completed,dropped=self.dropped,pending=int(self.pending is not None),
                        fps=round(fps,2),p95Ms=round(latencies[min(len(latencies)-1,int(.95*len(latencies)))],2) if latencies else None)

    def request_shutdown(self):
        self.shutdown_requested=True

    def publish_diagnostics(self):
        if self.shutdown_requested:
            self.stop.set()
            rclpy.shutdown()
            return
        self._observe_stream(time.monotonic())
        with self.condition:
            status=self.status()
            if self.had_detection and not status['detector']['ok']:
                self.detection_pub.publish(Detection2DArray(header=self.raw_frame['header']))
                self.had_detection=False
        report=DiagnosticStatus(name='ldc_vision',level=DiagnosticStatus.OK if status['detector']['ok'] else DiagnosticStatus.ERROR,
                                message=status['detector']['detail'],values=[KeyValue(key=k,value=str(status[k]))
                                for k in ('fps','p95Ms','dropped','pending','device','timestamp_source')]+
                                [KeyValue(key=k,value=str(status['detector'][k]))
                                for k in ('restarts','inferErrors')]+
                                [KeyValue(key='track'+k.capitalize(),value=str(status['tracking'][k]))
                                for k in ('confirmed','lost','total')])
        self.diag_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()),status=[report]))

    def destroy_node(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
            worker=self.worker
        worker.close();self.thread.join(timeout=3)
        self.gateway.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=Detector();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
