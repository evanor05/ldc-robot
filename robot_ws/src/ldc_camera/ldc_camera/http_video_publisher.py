"""Single MJPEG reader, latest-frame ROS publishers and acquisition diagnostics."""
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
from sensor_msgs.msg import Image, CompressedImage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header

from .discovery import discover_camera
from .mjpeg import MjpegParser, validate_url
from .calibration import Calibration
from .lease import CameraLease


def publish_due(now, last_publish, fps):
    """Whether a frame may be published under the fps upper bound."""
    return not last_publish or now - last_publish >= 1.0 / fps


class HttpVideoPublisher(Node):
    def __init__(self):
        super().__init__('http_video_publisher')
        defaults = dict(url='', topic='/camera/image_raw/compressed', raw_topic='/camera/image_raw',
                        frame_id='camera_optical_frame', fps=10.0, reconnect_delay=2.0,
                        publish_raw=False, discovery=True, discovery_port=33333,
                        discovery_timeout=2.0, discovery_broadcast='255.255.255.255',
                        calibration_file='', read_timeout=2.0, use_ffmpeg=True, jpeg_quality=70)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if not 0 < self.cfg['fps'] <= 60 or not .1 <= self.cfg['read_timeout'] <= 30 or self.cfg['reconnect_delay'] < .1:
            raise ValueError('fps/timeout/reconnect_delay 参数无效')
        if self.cfg['url']:
            validate_url(self.cfg['url'])
        elif not self.cfg['discovery']:
            raise ValueError('需要摄像头 URL 或启用发现')
        self.acquisition_lease = CameraLease(self.cfg['topic'])
        self.calibration = Calibration(self.cfg['calibration_file'])
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = self.published = self.reconnects = self.dropped = 0
        self.last_publish = 0.0
        self.detail = '等待摄像头连接'
        self.calibration_detail = self.calibration.error
        self.response = None
        self.publisher = self.create_publisher(CompressedImage, self.cfg['topic'], SENSOR_QOS)
        self.raw = self.create_publisher(Image, self.cfg['raw_topic'], SENSOR_QOS)
        from sensor_msgs.msg import CameraInfo
        self.info = self.create_publisher(CameraInfo, '/camera/camera_info', SENSOR_QOS)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # 读线程与发布线程解耦：capture_loop 只负责把 TCP 读干（避免 publish
        # 阻塞导致 TCP 背压、丢帧），publish_loop 独立从 latest 取最新帧发出。
        self.create_timer(.5, self.publish_diagnostics)
        self.publish_event = threading.Event()
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.pub_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.pub_thread.start()

    def capture_loop(self):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.stop.is_set():
            try:
                url = self.cfg['url']
                if not url:
                    device = discover_camera(timeout=self.cfg['discovery_timeout'], port=self.cfg['discovery_port'],
                                             broadcast=self.cfg['discovery_broadcast'])
                    if not device:
                        raise ValueError('自动发现未找到摄像头')
                    url = device['stream_url']
                response = opener.open(validate_url(url), timeout=self.cfg['read_timeout'])
                with self.lock:
                    self.response = response
                with response:
                    parser = MjpegParser(response.headers.get('Content-Type', ''))
                    while not self.stop.is_set():
                        data = response.read1(65536)
                        if not data:
                            parser.finish()
                            raise ConnectionError('摄像头数据流已结束')
                        for jpeg in parser.feed(data):
                            received = time.monotonic()
                            header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.cfg['frame_id'])
                            # 不在采集线程解码整帧（800x600 imdecode 会与发布抢 GIL，
                            # 把吞吐拖到 ~14Hz）。尺寸从 JPEG 头轻量解析即可。
                            with self.lock:
                                self.sequence += 1
                                if self.latest and self.latest[0] > self.published:
                                    self.dropped += 1
                                self.latest = (self.sequence, jpeg, None, header, received)
                                self.detail = '正在采集；时间基准为主机完整帧接收时刻'
                            # 只通知发布线程，绝不在读线程里调用 publish。
                            self.publish_event.set()
            except Exception as error:
                if not self.stop.is_set():
                    with self.lock:
                        self.detail = str(error)
                        self.latest = None
                        self.reconnects += 1
                    self.stop.wait(self.cfg['reconnect_delay'])
            finally:
                with self.lock:
                    self.response = None

    def publish_loop(self):
        """独立发布线程：有新帧就发最新帧，发布阻塞也绝不拖慢读取。"""
        while not self.stop.is_set():
            if not self.publish_event.wait(0.5):
                continue
            self.publish_event.clear()
            with self.lock:
                sample = self.latest
            if sample is not None:
                try:
                    self._publish_sample(sample)
                except Exception as error:
                    with self.lock:
                        self.detail = f'发布失败：{error}'

    @staticmethod
    def _jpeg_size(jpeg):
        # 轻量解析 JPEG SOF 段拿宽高，避免整帧 cv2.imdecode。
        n = len(jpeg)
        i = 2
        while i + 9 < n:
            if jpeg[i] != 0xFF:
                i += 1
                continue
            marker = jpeg[i+1]
            if marker in (0xC0, 0xC1, 0xC2, 0xC3):
                h = (jpeg[i+5] << 8) | jpeg[i+6]
                w = (jpeg[i+7] << 8) | jpeg[i+8]
                return w, h
            seg_len = (jpeg[i+2] << 8) | jpeg[i+3]
            i += 2 + seg_len
        return None, None

    def _publish_sample(self, sample):
        if not sample or sample[0] == self.published:
            return
        now = time.monotonic()
        if now - sample[4] > 1.0:
            return
        # fps=0 不合法；fps 为上限而不是定时器：低频相机仍按到达即发，
        # 高频相机按 1/fps 最小间隔丢旧帧，latest 始终保留下一帧。
        if not publish_due(now, self.last_publish, self.cfg['fps']):
            return
        seq, jpeg, frame, header, _ = sample
        self.publisher.publish(CompressedImage(header=header, format='jpeg', data=jpeg))
        if frame is not None:
            height, width = frame.shape[:2]
        else:
            width, height = self._jpeg_size(jpeg)
        if width and height:
            info, self.calibration_detail = self.calibration.message(header, width, height)
            self.info.publish(info)
            if self.cfg['publish_raw'] and self.raw.get_subscription_count():
                if frame is None:
                    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    self.raw.publish(Image(header=header, height=height, width=width, encoding='rgb8',
                                           step=width*3, data=rgb.tobytes()))
        self.published = seq
        self.last_publish = now

    def publish_diagnostics(self):
        with self.lock:
            sample, detail = self.latest, self.detail
        ok = sample is not None and time.monotonic()-sample[4] <= 1
        values = dict(timestamp_source='host_receive', calibration=self.calibration_detail,
                      reconnects=self.reconnects, dropped=self.dropped, sequence=self.sequence)
        status = DiagnosticStatus(name='ldc_camera', level=(DiagnosticStatus.OK if self.calibration_detail.startswith('已加载') else DiagnosticStatus.WARN) if ok else DiagnosticStatus.ERROR, message=detail,
                                  values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        self.diagnostics.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))

    def destroy_node(self):
        self.stop.set()
        with self.lock:
            response = self.response
        if response is not None:
            response.close()
        self.thread.join(timeout=self.cfg['read_timeout']+self.cfg['discovery_timeout']+1)
        self.acquisition_lease.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HttpVideoPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
