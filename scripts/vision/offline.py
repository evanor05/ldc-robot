#!/usr/bin/env python3
"""Start only the isolated vision stack and console; no chassis/navigation processes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request
import json
ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--image',required=True)
    parser.add_argument('--api-port',type=int,default=18330);parser.add_argument('--web-port',type=int,default=5175)
    args=parser.parse_args();env=os.environ.copy()
    maps=tempfile.TemporaryDirectory(prefix='ldc-vision-maps-')
    env['ROBOT_MAPS_DIR']=maps.name
    env.update(ROBOT_SIMULATION='1',ROBOT_API_PORT=str(args.api_port),VITE_ROBOT_API_PORT=str(args.api_port),
               ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    children=[]
    api=None
    def spawn(command,cwd=ROOT):
        proc=subprocess.Popen(command,cwd=cwd,env=env,start_new_session=True);children.append(proc);return proc
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    try:
        # Never take over another session's source or gateway.
        import socket
        for port in (18321,18322,args.api_port,args.web_port):
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                sock.bind(('127.0.0.1',port))
        spawn([sys.executable,str(ROOT/'scripts/vision/mock_camera.py'),'--image',str(Path(args.image).resolve())])
        api=spawn(['node','server.js'],ROOT/'web_console')
        for _ in range(80):
            try:
                request=urllib.request.Request(f'http://127.0.0.1:{args.api_port}/api/vision/start',data=json.dumps(dict(url='http://127.0.0.1:18322/stream')).encode(),headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(request,timeout=15) as response:
                    assert json.load(response)['running']
                break
            except urllib.error.URLError:
                if any(p.poll() is not None for p in children):raise RuntimeError('离线子进程退出')
                time.sleep(.2)
        else:raise RuntimeError('Node 启动超时')
        spawn(['npm','run','dev','--','--host','127.0.0.1','--port',str(args.web_port),'--strictPort'],ROOT/'web_console')
        print(f'仿真视觉控制台 http://localhost:{args.web_port}/ （Domain 197，网关18321）；Ctrl+C 清理',flush=True)
        while all(p.poll() is None for p in children):time.sleep(.5)
        raise RuntimeError('离线子进程意外退出')
    except KeyboardInterrupt:pass
    finally:
        try:
            if api is not None and api.poll() is None:
                request=urllib.request.Request(f'http://127.0.0.1:{args.api_port}/api/vision/stop',data=b'{}',headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(request,timeout=10):pass
        except (OSError,urllib.error.URLError):pass
        for proc in reversed(children):
            subprocess.run(['node','--input-type=module','-e',
                "import {terminateProcess} from './web_console/managedProcess.js';await terminateProcess({pid:Number(process.argv[1])});",
                str(proc.pid)],cwd=ROOT,check=True)
            proc.wait(timeout=5)
        maps.cleanup()
if __name__=='__main__':main()
