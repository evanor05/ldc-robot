#!/usr/bin/env python3
"""Read real MJPEG through Node fanout; one stalled viewer must not stall fast ones."""
import argparse
import http.client
import json
from pathlib import Path
import threading
import time
import urllib.request


def main():
    p=argparse.ArgumentParser();p.add_argument('--api-port',type=int,default=18330);p.add_argument('--source-port',type=int,default=18322);p.add_argument('--output',required=True)
    args=p.parse_args();clients=[];frames=[0,0,0];errors=[];threads=[]
    def read(index,response):
        try:
            while not stop.is_set():
                if not response.readline():return
                length=None
                while True:
                    line=response.readline()
                    if not line:return
                    if line.lower().startswith(b'content-length:'):length=int(line.split(b':',1)[1])
                    if line==b'\r\n':break
                assert length and length<4*1024*1024
                image=response.read(length);assert image[:2]==b'\xff\xd8' and image[-2:]==b'\xff\xd9'
                response.read(2);frames[index]+=1
        except Exception as e:
            if not stop.is_set():errors.append(str(e))
    stop=threading.Event()
    try:
        for i in range(4):
            conn=http.client.HTTPConnection('127.0.0.1',args.api_port,timeout=3);conn.request('GET','/api/vision/'+('annotated' if i==2 else 'raw')+'.mjpg');response=conn.getresponse();assert response.status==200
            clients.append((conn,response))
            if i<3:
                thread=threading.Thread(target=read,args=(i,response),daemon=True);thread.start();threads.append(thread)
        conn=http.client.HTTPConnection('127.0.0.1',args.api_port,timeout=3);conn.request('GET','/api/vision/raw.mjpg');response=conn.getresponse();assert response.status==429;response.close();conn.close()
        time.sleep(12)
        with urllib.request.urlopen(f'http://127.0.0.1:{args.source_port}/status',timeout=2) as r:source=json.load(r)
        with urllib.request.urlopen(f'http://127.0.0.1:{args.api_port}/api/vision/status',timeout=2) as r:state=json.load(r)
        report=dict(fast_client_frames=frames,errors=errors,source=source,vision={k:state[k] for k in ('fps','pending','p95Ms')})
        report['passed']=not errors and min(frames)>=80 and source['max_active']==1 and state['pending']<=1 and state['detector']['ok']
        Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report));assert report['passed']
    finally:
        stop.set()
        for conn,response in clients:response.close();conn.close()
        for thread in threads:thread.join(timeout=4)
if __name__=='__main__':main()
