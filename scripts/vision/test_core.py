#!/usr/bin/env python3
import copy
import json
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
ROOT=Path(__file__).resolve().parents[2]
for package in ('ldc_camera','ldc_vision'):sys.path.insert(0,str(ROOT/'robot_ws/src'/package))
from ldc_camera.mjpeg import MjpegParser,MAX_HEADERS,validate_url
from ldc_camera.calibration import Calibration
from ldc_camera.http_video_publisher import publish_due
from ldc_vision.ipc import read_packet,write_packet,MAX_META
from ldc_vision.results import checked_detections
from ldc_vision.gateway import PreviewGateway
from ldc_vision.worker_client import WorkerClient,WorkerFrameError,worker_restart_delay
from ldc_vision.tracking import MultiObjectTracker,iou
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.serialization import serialize_message

JPEG=b'\xff\xd8fake\xff\xd9'
def part(jpeg=JPEG):return b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n'
class ParserTests(unittest.TestCase):
    def test_every_fragment_size_and_multiple_frames(self):
        for size in (1,2,7,31,128,4096):
            parser=MjpegParser('multipart/x-mixed-replace; boundary="f"');output=[];data=part()*3
            for offset in range(0,len(data),size):output.extend(parser.feed(data[offset:offset+size]))
            self.assertEqual(output,[JPEG]*3);parser.finish()
    def test_truncation_and_oversize(self):
        parser=MjpegParser('boundary=f');parser.feed(part()[:-5])
        with self.assertRaises(ValueError):parser.finish()
        with self.assertRaises(ValueError):MjpegParser('boundary=f',limit=4).feed(part())
        parser=MjpegParser('boundary=f');parser.feed(b'--f\r\nContent-')
        with self.assertRaises(ValueError):parser.finish()
    def test_complete_large_header_and_bad_jpeg(self):
        with self.assertRaises(ValueError):MjpegParser('boundary=f').feed(b'--f\r\nX: '+b'a'*(MAX_HEADERS+1)+b'\r\n\r\n')
        with self.assertRaises(ValueError):MjpegParser('boundary=f').feed(part(b'bad!'))
    def test_url_credentials_and_protocol_rejected(self):
        for url in ('ftp://host/a','http://user:secret@host/a','/relative'):
            with self.assertRaises(ValueError):validate_url(url)

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.header=dict(sec=1,nanosec=2,frame_id='camera_optical_frame')
        self.result=dict(id='frame',header=self.header,width=810,height=1080,coordinates='original_pixels',detections=[dict(name='person',score=.8,xyxy=[-1,20,820,500])])
    def test_original_pixels_clipped_and_identity(self):
        self.assertEqual(checked_detections(self.result,'frame',self.header,810,1080)[0]['xyxy'],[0.,20,810,500])
        for key,value in [('id','other'),('width',640),('coordinates','letterbox')]:
            wrong=copy.deepcopy(self.result);wrong[key]=value
            with self.assertRaises(ValueError):checked_detections(wrong,'frame',self.header,810,1080)
    def test_invalid_numeric_and_degenerate_boxes(self):
        for box in ([0,0,float('nan'),2],[3,2,1,4],[0,0,1,float('inf')]):
            wrong=copy.deepcopy(self.result);wrong['detections'][0]['xyxy']=box
            with self.assertRaises(ValueError):checked_detections(wrong,'frame',self.header,810,1080)
    def test_uncalibrated_and_size_mismatch(self):
        msg,detail=Calibration().message(Header(),800,600);self.assertEqual(msg.k[0],0)
        with tempfile.NamedTemporaryFile('w') as f:
            f.write('image_width: 1\nimage_height: 1\n');f.flush()
            msg,detail=Calibration(f.name).message(Header(),800,600)
            self.assertEqual(msg.k[0],0);self.assertIn('尺寸',detail)
    def test_diagnostics_serializes_installed_byte_contract(self):
        self.assertGreater(len(serialize_message(DiagnosticStatus(level=DiagnosticStatus.ERROR,message='断流'))),0)

class IPCTests(unittest.TestCase):
    def test_roundtrip(self):
        a,b=os.pipe()
        try:
            write_packet(b,dict(id='f'),JPEG);meta,jpeg=read_packet(a,1)
            self.assertEqual((meta,jpeg),(dict(id='f'),JPEG))
        finally:os.close(a);os.close(b)
    def test_timeout_truncation_and_limits(self):
        a,b=os.pipe()
        try:
            with self.assertRaises(TimeoutError):read_packet(a,.01)
            os.write(b,struct.pack('<I',MAX_META+1))
            with self.assertRaises(ValueError):read_packet(a,1)
            os.close(b);b=None
            with self.assertRaises(EOFError):read_packet(a,1)
        finally:os.close(a);b is not None and os.close(b)
    def test_stopped_worker_cannot_restart(self):
        worker=WorkerClient('/nonexistent','missing','0',640,.25);worker.close()
        with self.assertRaises(RuntimeError):worker.start()

class GatewayTests(unittest.TestCase):
    def setUp(self):
        class Owner:
            stop=threading.Event()
            def status(self):return dict(service='ldc_vision')
            def preview(self,kind):return (1,JPEG)
            def set_filter(self,mode):
                if mode not in ('all','traffic'):raise ValueError('filter')
            def request_shutdown(self):self.stop.set()
        self.owner=Owner();self.gateway=PreviewGateway(self.owner,0);self.base='http://127.0.0.1:'+str(self.gateway.server.server_port)
    def tearDown(self):self.owner.stop.set();self.gateway.close()
    def test_status_filter_and_shutdown(self):
        with urllib.request.urlopen(self.base+'/status',timeout=2) as r:self.assertEqual(json.load(r)['service'],'ldc_vision')
        request=urllib.request.Request(self.base+'/filter',data=b'{"mode":"invalid"}',headers={'Content-Type':'application/json'})
        with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(request,timeout=2)
        caught.exception.close()
        with urllib.request.urlopen(urllib.request.Request(self.base+'/shutdown',data=b'{}'),timeout=2):pass
        self.assertTrue(self.owner.stop.wait(.5))
    def test_preview_is_multipart_with_frame_id(self):
        with urllib.request.urlopen(self.base+'/raw.mjpg',timeout=2) as response:
            headers=b''
            while True:
                line=response.readline();headers+=line
                if line==b'\r\n':break
            self.assertIn(b'X-Frame-Id: 1',headers)
            self.assertEqual(response.read(len(JPEG)),JPEG)


class DiscoveryTests(unittest.TestCase):
    def test_loopback_discovery_uses_firmware_fixed_reply_port(self):
        from ldc_camera.discovery import discover_camera,pack_mavlink_v2
        # Fake firmware listens on `firmware_port` and, like udp_discovery.c,
        # replies to the fixed client port `client_port` (not the source port).
        firmware=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);firmware.bind(('127.0.0.1',0));firmware_port=firmware.getsockname()[1];firmware.settimeout(2)
        probe=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);probe.bind(('127.0.0.1',0));client_port=probe.getsockname()[1];probe.close()
        def reply():
            try:
                _,address=firmware.recvfrom(2048)
                texts=['DEVICE_NAME:mock_camera','DEVICE_IP:127.0.0.1','STREAM_PORT:18324']
                for seq,text in enumerate(texts):
                    payload=bytes([6])+text.encode().ljust(50,b'\0')
                    firmware.sendto(pack_mavlink_v2(seq,1,1,253,payload,83),('127.0.0.1',client_port))
            finally:firmware.close()
        server=threading.Thread(target=reply);server.start()
        result=discover_camera(timeout=1,port=firmware_port,bind_port=client_port,broadcast='127.0.0.1',bind='127.0.0.1');server.join(timeout=3)
        self.assertEqual(result['stream_url'],'http://127.0.0.1:18324/')

    def test_discovery_reports_occupied_fixed_port(self):
        from ldc_camera.discovery import discover_camera,DiscoveryPortError
        blocker=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);blocker.bind(('127.0.0.1',0));port=blocker.getsockname()[1]
        try:
            with self.assertRaises(DiscoveryPortError):
                discover_camera(timeout=.2,port=port,bind_port=port,broadcast='127.0.0.1',bind='127.0.0.1')
        finally:blocker.close()

class FrameQueueTests(unittest.TestCase):
    def test_empty_stale_duplicate_and_latest_only_frames(self):
        from types import SimpleNamespace
        from ldc_vision.node import Detector
        from sensor_msgs.msg import CompressedImage
        now=2_000_000_000
        fake=SimpleNamespace(condition=threading.Condition(),dropped=0,last_stamp=0,sequence=0,received=0,pending=None,session='test',
                             get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now)))
        def message(sec,nano=0,jpeg=JPEG):
            msg=CompressedImage(header=Header(frame_id='camera_optical_frame'),format='jpeg',data=jpeg)
            msg.header.stamp.sec=sec;msg.header.stamp.nanosec=nano;return msg
        Detector.on_frame(fake,message(0));Detector.on_frame(fake,message(2,jpeg=b''))
        self.assertEqual(fake.received,0)
        Detector.on_frame(fake,message(2));Detector.on_frame(fake,message(2))
        self.assertEqual(fake.received,1)
        Detector.on_frame(fake,message(2,1));self.assertEqual(fake.received,2)
        self.assertEqual(fake.pending['sequence'],2);self.assertGreaterEqual(fake.dropped,4)


class ThrottleTests(unittest.TestCase):
    def test_fps_is_an_upper_bound(self):
        self.assertTrue(publish_due(1.0, 0.0, 10))
        self.assertFalse(publish_due(1.05, 1.0, 10))
        self.assertTrue(publish_due(1.1, 1.0, 10))
        self.assertFalse(publish_due(1.0, 0.99, 60))


class WorkerRecoveryTests(unittest.TestCase):
    def test_frame_error_keeps_worker_alive_and_sigkill_can_restart(self):
        header={'sec': 1, 'nanosec': 2, 'frame_id': 'camera_optical_frame'}
        with tempfile.TemporaryDirectory() as temp:
            script=Path(temp)/'fake_worker.py'
            script.write_text(
                'import sys\n'
                f'sys.path.insert(0, {str(ROOT/"robot_ws/src/ldc_vision/ldc_vision")!r})\n'
                'from ipc import read_packet,write_packet\n'
                'write_packet(1, {"ready": True, "names": {}, "model_sha256": "fake"})\n'
                'while True:\n'
                '    metadata,image=read_packet(0,30)\n'
                '    if image==b"crash":sys.exit(3)\n'
                '    if image==b"bad":\n'
                '        write_packet(1, {"id":metadata.get("id"),"header":metadata.get("header"),"error":"bad frame"})\n'
                '        continue\n'
                '    write_packet(1, {"id":metadata.get("id"),"header":metadata.get("header"),"width":1,"height":1,\n'
                '                     "coordinates":"original_pixels","detections":[]})\n')
            client=WorkerClient(sys.executable,'unused','cpu',640,.25,worker_script=script)
            try:
                client.start()
                first=client.infer(dict(id='1',header=header),JPEG)
                self.assertEqual(first['detections'],[])
                with self.assertRaises(WorkerFrameError):
                    client.infer(dict(id='2',header=header),b'bad')
                again=client.infer(dict(id='3',header=header),JPEG)
                self.assertEqual(again['detections'],[])
                client.process.kill();client.process.wait()
                with self.assertRaises(Exception):
                    client.infer(dict(id='4',header=header),JPEG)
            finally:client.close()
            replacement=WorkerClient(sys.executable,'unused','cpu',640,.25,worker_script=script)
            try:
                replacement.start()
                recovered=replacement.infer(dict(id='5',header=header),JPEG)
                self.assertEqual(recovered['detections'],[])
            finally:replacement.close()

    def test_restart_backoff_is_exponential_and_capped(self):
        self.assertEqual(worker_restart_delay(1),1)
        self.assertEqual(worker_restart_delay(2),2)
        self.assertEqual(worker_restart_delay(3),4)
        self.assertEqual(worker_restart_delay(10),30)


class NodeSupervisorTests(unittest.TestCase):
    def test_worker_start_failure_restarts_and_recovers(self):
        from types import SimpleNamespace
        from ldc_vision.node import Detector
        class FakeWorker:
            def __init__(self,ready):self.ready=ready;self.closed=False
            def start(self):
                if self.ready:return dict(names={},model_sha256='fake')
                raise RuntimeError('worker start failed')
            def close(self):self.closed=True
        first=FakeWorker(False);second=FakeWorker(True)
        fake=SimpleNamespace(condition=threading.Condition(),stop=threading.Event(),worker=first,
                             pending=dict(at=time.monotonic(),jpeg=b'',header=None),dropped=0,
                             worker_ready=False,worker_error='模型正在加载',worker_failures=0,
                             worker_restarts=0,infer_errors=0,last_frame_error='',annotated=None,
                             names={},model_sha256='',tracker_lock=threading.Lock(),
                             tracker=SimpleNamespace(reset=lambda:None),
                             _reset_tracker=lambda:None,
                             get_logger=lambda:SimpleNamespace(warning=lambda *a:None,error=lambda *a:None))
        def new_worker():
            fake.pending=dict(at=time.monotonic()+10,jpeg=b'',header=None)
            return second
        fake._new_worker=new_worker
        fake._process_sample=lambda worker,sample: fake.stop.set()
        Detector.inference_loop(fake)
        self.assertEqual(fake.worker_restarts,1)
        self.assertTrue(fake.worker_ready)
        self.assertTrue(first.closed and second.closed)


class TrackerTests(unittest.TestCase):
    @staticmethod
    def det(x1=0,y1=0,x2=10,y2=10,name='laptop',score=.8,class_id=0):
        return dict(xyxy=[x1,y1,x2,y2],score=score,name=name,class_id=class_id)

    def test_single_frame_hidden_then_two_frames_visible(self):
        tracker=MultiObjectTracker()
        self.assertEqual(tracker.update([self.det()],1),[])
        self.assertEqual(tracker.stats()['confirmed'],0)
        visible=tracker.update([self.det()],2)
        self.assertEqual(len(visible),1)
        self.assertEqual(visible[0]['name'],'laptop')
        self.assertEqual(visible[0]['track_id'],1)
        self.assertEqual(tracker.stats()['confirmed'],1)
        self.assertEqual(tracker.stats()['total'],1)

    def test_lost_two_frames_survive_then_third_removes(self):
        tracker=MultiObjectTracker()
        tracker.update([self.det()],1);tracker.update([self.det()],2)
        first=tracker.update([],3);second=tracker.update([],4)
        self.assertEqual(len(first),1)
        self.assertEqual(len(second),1)
        self.assertEqual(tracker.stats()['lost'],1)
        third=tracker.update([],5)
        self.assertEqual(third,[])
        self.assertEqual(tracker.stats()['total'],0)

    def test_low_score_tracks_are_not_published(self):
        tracker=MultiObjectTracker(min_score=.35)
        tracker.update([self.det(score=.25)],1)
        visible=tracker.update([self.det(score=.25)],2)
        self.assertEqual(visible,[])
        self.assertEqual(tracker.stats()['confirmed'],1)
        visible=tracker.update([self.det(score=.5)],3)
        self.assertEqual(len(visible),1)
        self.assertGreaterEqual(visible[0]['score'],.35)

    def test_class_is_frozen_until_strong_repeated_class(self):
        tracker=MultiObjectTracker()
        tracker.update([self.det(name='laptop',score=.8,class_id=0)],1)
        frozen=tracker.update([self.det(name='tv',score=.8,class_id=1)],2)
        self.assertEqual(frozen[0]['name'],'laptop')
        switched=tracker.update([self.det(name='tv',score=.8,class_id=1)],3)
        self.assertEqual(switched[0]['name'],'tv')

    def test_moving_box_and_score_are_ema_smoothed(self):
        tracker=MultiObjectTracker()
        tracker.update([self.det(0,0,10,10,score=.8)],1)
        visible=tracker.update([self.det(2,0,12,10,score=.6)],2)
        box=visible[0]['xyxy']
        self.assertAlmostEqual(box[0],.9)
        self.assertAlmostEqual(box[2],10.9)
        self.assertAlmostEqual(visible[0]['score'],.71)

    def test_iou_threshold_one_to_one_and_reset(self):
        tracker=MultiObjectTracker()
        self.assertAlmostEqual(iou([0,0,10,10],[0,0,10,10]),1.0)
        tracker.update([self.det()],1);tracker.update([self.det()],2)
        tracker.reset()
        self.assertEqual(tracker.stats(),{'confirmed':0,'lost':0,'total':0})
        self.assertEqual(tracker.update([self.det()],3),[])


if __name__=='__main__':unittest.main()
