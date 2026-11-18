#!/usr/bin/env python3
"""Own a standalone offline source/launch for exactly 30 minutes of measurements."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('--image',required=True);p.add_argument('--output',required=True);args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    for port in (18321,18322):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    children=[];log=(out/'ros.log').open('w')
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    try:
        children.append(subprocess.Popen([sys.executable,str(ROOT/'scripts/vision/mock_camera.py'),'--image',args.image],stdout=log,stderr=subprocess.STDOUT))
        children.append(subprocess.Popen(['bash',str(ROOT/'scripts/start_vision.sh'),'--sim','url:=http://127.0.0.1:18322/stream'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT))
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            if any(c.poll() is not None for c in children):raise RuntimeError('测试进程提前退出')
            try:
                with urllib.request.urlopen('http://127.0.0.1:18321/status',timeout=2) as response:s=json.load(response)
                if s['detector']['ok'] and s['fps']>=8:break
            except OSError:pass
            time.sleep(.5)
        else:raise RuntimeError('60 秒内 GPU 未就绪')
        subprocess.run([sys.executable,str(ROOT/'scripts/vision/soak.py'),'--output',str(out)],check=True)
    finally:
        for child in reversed(children):
            subprocess.run(['node','--input-type=module','-e',"import {terminateProcess} from './web_console/managedProcess.js';await terminateProcess({pid:Number(process.argv[1])});",str(child.pid)],cwd=ROOT,check=True)
            child.wait(timeout=5)
        log.close()
if __name__=='__main__':main()
