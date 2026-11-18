#!/usr/bin/env python3
"""Loopback MJPEG fixture source, with controllable failure modes for integration tests."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--image',required=True);parser.add_argument('--port',type=int,default=18322)
    parser.add_argument('--fps',type=float,default=10)
    args=parser.parse_args();jpeg=Path(args.image).read_bytes()
    lock=threading.Lock();state=dict(mode='normal',active=0,connections=0,frames=0,max_active=0)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            if self.path.startswith('/control/'):
                mode=self.path.rsplit('/',1)[1]
                if mode not in ('normal','freeze','disconnect','oversize','fragment','truncate'):
                    self.send_error(400);return
                with lock:state['mode']=mode
            if self.path=='/status' or self.path.startswith('/control/'):
                with lock:data=json.dumps(state).encode()
                self.send_response(200);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
            if self.path!='/stream':self.send_error(404);return
            self.connection.settimeout(2)
            with lock:
                state['active']+=1;state['connections']+=1;state['max_active']=max(state['max_active'],state['active'])
            try:
                self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers()
                deadline=time.monotonic()
                while True:
                    with lock:mode=state['mode']
                    if mode=='freeze':time.sleep(.05);continue
                    if mode=='disconnect':break
                    size=99999999 if mode=='oversize' else len(jpeg)
                    part=b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(size).encode()+b'\r\n\r\n'+jpeg+b'\r\n'
                    if mode=='truncate':self.wfile.write(part[:len(part)//2]);self.wfile.flush();break
                    if mode=='fragment':
                        for start in range(0,len(part),127):self.wfile.write(part[start:start+127])
                    else:self.wfile.write(part)
                    self.wfile.flush()
                    with lock:state['frames']+=1
                    deadline+=1/args.fps;time.sleep(max(0,deadline-time.monotonic()))
            except OSError:pass
            finally:
                with lock:state['active']-=1
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.daemon_threads=True
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()
if __name__=='__main__':main()
