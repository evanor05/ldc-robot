#!/usr/bin/env python3
"""Isolated real ROS + GPU integration and fault injection. Domain 196, loopback only."""
import argparse
from collections import OrderedDict
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[2]

def get(port,path='/status',body=None):
    request=urllib.request.Request(f'http://127.0.0.1:{port}{path}',data=json.dumps(body).encode() if body is not None else None,
                                   headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=3) as response:return json.load(response)

def wait_for(check,timeout=15):
    end=time.monotonic()+timeout;last=None
    while time.monotonic()<end:
        try:
            last=check()
            if last:return last
        except (OSError,ValueError):pass
        time.sleep(.1)
    raise AssertionError(f'condition timed out ({last})')

def descendants(pid):
    found=[pid]
    try:
        for task in Path(f'/proc/{pid}/task').iterdir():
            try:
                for child in (task/'children').read_text().split():found+=descendants(int(child))
            except OSError:pass
    except OSError:pass
    return found

def main():
    p=argparse.ArgumentParser();p.add_argument('--image',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    import socket
    for port in (18323,18324):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','FASTDDS_BUILTIN_TRANSPORTS')})
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage,Image,CameraInfo
    from vision_msgs.msg import Detection2DArray
    rclpy.init();probe=Node('vision_acceptance_probe');messages={k:OrderedDict() for k in ('compressed','raw','info','annotation','detections')}
    lock=threading.Lock();repeat=[];counts={};empty=[]
    def record(kind):
        def callback(msg):
            key=(msg.header.stamp.sec,msg.header.stamp.nanosec,msg.header.frame_id)
            with lock:
                if key in messages[kind]:repeat.append(kind)
                messages[kind][key]=msg;counts[kind]=counts.get(kind,0)+1
                while len(messages[kind])>32:messages[kind].popitem(last=False)
                if kind=='detections' and not msg.detections:empty.append(time.monotonic())
        return callback
    subscriptions=[probe.create_subscription(typ,topic,record(kind),qos_profile_sensor_data) for kind,typ,topic in [
        ('compressed',CompressedImage,'/camera/image_raw/compressed'),('raw',Image,'/camera/image_raw'),
        ('info',CameraInfo,'/camera/camera_info'),('annotation',CompressedImage,'/vision/annotated/compressed'),
        ('detections',Detection2DArray,'/vision/detections')]]
    spin=threading.Thread(target=lambda:rclpy.spin(probe),daemon=True);spin.start()
    source=launch=None;log=(out/'ros.log').open('w');events=[];readers=[]
    def event(name,**details):events.append(dict(test=name,**details));print(name,flush=True)
    try:
        source=subprocess.Popen([sys.executable,str(ROOT/'scripts/vision/mock_camera.py'),'--image',args.image,'--port','18324'],env=env)
        wait_for(lambda:get(18324))
        launch=subprocess.Popen(['ros2','launch','ldc_vision','vision.launch.py','url:=http://127.0.0.1:18324/stream','gateway_port:=18323'],env=env,stdout=log,stderr=subprocess.STDOUT)
        wait_for(lambda:get(18323)['detector']['ok'],60)
        wait_for(lambda:all(counts.get(k,0)>=3 for k in messages),10)
        time.sleep(1)
        with lock:
            common=set.intersection(*(set(m) for m in messages.values()))
            assert common,'图像、CameraInfo、检测未找到同一时间戳'
            for key in common:
                raw,info=messages['raw'][key],messages['info'][key]
                assert (raw.width,raw.height)==(info.width,info.height)
                assert info.k[0]==0 and key[2]=='camera_optical_frame'
            assert not repeat,'重复发布同一帧'
        publishers=[]
        for topic,types in probe.get_topic_names_and_types():
            if any(t in ('geometry_msgs/msg/Twist','geometry_msgs/msg/TwistStamped','geometry_msgs/msg/PoseStamped') for t in types):
                publishers+=probe.get_publishers_info_by_topic(topic)
        assert not publishers,'视觉栈出现运动或目标发布者'
        event('ROS same-frame Header, CameraInfo, raw dimensions, no repeated frames, no motion publishers',common_frames=len(common))
        # Four fanout clients, one deliberately never reading; acquisition remains single.
        import http.client
        for kind in ('raw','raw','annotated','raw'):
            conn=http.client.HTTPConnection('127.0.0.1',18323,timeout=3);conn.request('GET','/'+kind+'.mjpg');response=conn.getresponse();assert response.status==200
            readers.append((conn,response))
        time.sleep(2);state=get(18324);assert state['max_active']==1
        event('four previews including slow clients keep one source connection',source=state)
        for conn,response in readers:response.close();conn.close()
        readers=[]
        get(18323,'/filter',{'mode':'traffic'});wait_for(lambda:get(18323)['detector']['ok'])
        assert get(18323)['detections']==[],'bus sample should yield valid empty traffic result'
        event('traffic filter returns healthy empty detections')
        get(18323,'/filter',{'mode':'all'});wait_for(lambda:bool(get(18323)['detections']))
        for mode in ('freeze','disconnect','truncate','oversize','fragment'):
            get(18324,'/control/'+mode)
            if mode!='fragment':
                wait_for(lambda:not get(18323)['camera']['ok'],7)
                failed=get(18323);assert not failed['detections'] and failed['frame'] is None
            get(18324,'/control/normal');wait_for(lambda:get(18323)['detector']['ok'],12)
            event('source fault and recovery: '+mode)
        # Check no republishing cached frames while camera is frozen.
        get(18324,'/control/freeze');time.sleep(.5)
        before=counts.get('compressed',0);time.sleep(.5);assert counts.get('compressed',0)==before
        wait_for(lambda:not get(18323)['camera']['ok']);wait_for(lambda:bool(empty),3)
        get(18324,'/control/normal');wait_for(lambda:get(18323)['detector']['ok'])
        event('frozen input publishes no cached frames and clears ROS detections')
        owned=descendants(launch.pid)
        workers=[pid for pid in owned if b'ldc_vision/worker.py' in Path(f'/proc/{pid}/cmdline').read_bytes()]
        assert len(workers)==1;worker=workers[0]
        os.kill(worker,signal.SIGSTOP)
        wait_for(lambda:not get(18323)['detector']['workerReady'],9)
        status=get(18323);assert status['camera']['ok'] and not status['detections']
        assert '超时' in status['detector']['detail'] or status['detector'].get('restarts',0)>0
        event('worker timeout clears boxes while raw camera stays healthy',detail=status['detector']['detail'])
        wait_for(lambda:not Path(f'/proc/{worker}').exists(),5)
        get(18323,'/shutdown',{});launch.wait(timeout=12)
        wait_for(lambda:get(18324)['active']==0)
        event('gateway shutdown exits launch, camera and worker',exit_code=launch.returncode)
        # Distinguish abrupt worker exit from an inference timeout.
        command=['ros2','launch','ldc_vision','vision.launch.py','url:=http://127.0.0.1:18324/stream','gateway_port:=18323']
        launch=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        wait_for(lambda:get(18323)['detector']['ok'],60)
        worker=next(pid for pid in descendants(launch.pid) if b'ldc_vision/worker.py' in Path(f'/proc/{pid}/cmdline').read_bytes())
        os.kill(worker,signal.SIGKILL)
        wait_for(lambda:not get(18323)['detector']['workerReady'],8)
        failed=get(18323);assert failed['camera']['ok'] and not failed['detections']
        event('abrupt worker exit clears detections and preserves raw preview',detail=failed['detector']['detail'])
        wait_for(lambda:get(18323)['detector']['workerReady'],30)
        wait_for(lambda:get(18323)['detector']['ok'],30)
        event('worker auto-restarts after SIGKILL',restarts=get(18323)['detector']['restarts'])
        # A camera ROS node crash must stop the whole launch and reap the worker.
        owned=descendants(launch.pid)
        camera=next(pid for pid in owned if b'ldc_camera/lib/ldc_camera/http_video_publisher' in Path(f'/proc/{pid}/cmdline').read_bytes())
        os.kill(camera,signal.SIGKILL);launch.wait(timeout=12)
        wait_for(lambda:get(18324)['active']==0)
        wait_for(lambda:all(not Path(f'/proc/{pid}').exists() or Path(f'/proc/{pid}/stat').read_text().split(') ')[1].startswith('Z') for pid in owned),5)
        event('camera node exit cleans up entire vision launch')
        # GPU unavailability is explicit at the ROS coordinator, never a CPU fallback.
        failure_env={**env,'CUDA_VISIBLE_DEVICES':''}
        launch=subprocess.Popen(command,env=failure_env,stdout=log,stderr=subprocess.STDOUT)
        wait_for(lambda:'GPU 不可用' in get(18323)['detector']['detail'],60)
        failed=get(18323);assert failed['camera']['ok'] and not failed['detections'] and failed['device']=='0'
        event('GPU unavailable is a distinct failure and keeps raw preview',detail=failed['detector']['detail'])
        get(18323,'/shutdown',{});launch.wait(timeout=12)
        (out/'report.json').write_text(json.dumps(dict(passed=True,events=events),indent=2))
    finally:
        for conn,response in readers:response.close();conn.close()
        if launch:
            # Use repository process-tree cleanup, including launch_ros new sessions.
            subprocess.run(['node','--input-type=module','-e',"import {terminateProcess} from './web_console/managedProcess.js';await terminateProcess({pid:Number(process.argv[1])});",str(launch.pid)],cwd=ROOT,check=True)
            launch.wait(timeout=5)
        if source:source.terminate();source.wait(timeout=5)
        probe.destroy_node();rclpy.shutdown();spin.join(timeout=3);log.close()
if __name__=='__main__':main()
