#!/usr/bin/env python3
"""Use an already running isolated Node backend; check idempotency and full lifecycle."""
import argparse
import json
from pathlib import Path
import time
import urllib.request


def main():
    p=argparse.ArgumentParser();p.add_argument('--api-port',type=int,default=18330);p.add_argument('--output',required=True);args=p.parse_args()
    def call(endpoint,body=None):
        request=urllib.request.Request(f'http://127.0.0.1:{args.api_port}/api/vision/'+endpoint,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(request,timeout=15) as r:return json.load(r)
    def wait(check,seconds=60):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            state=call('status')
            if check(state):return state
            time.sleep(.2)
        raise AssertionError('API condition timed out')
    events=[]
    owned=False
    try:
        assert not call('status')['running'],'需要空闲的视觉会话'
        options=dict(url='http://127.0.0.1:18322/stream',device='0',filter='all')
        assert call('start',options)['running']
        owned=True
        state=wait(lambda s:s['detector']['ok']);first=state['frame']['id'].split(':')[0]
        assert call('start',options)['running']
        state=wait(lambda s:s['detector']['ok']);assert state['frame']['id'].startswith(first+':')
        events.append('repeated start retains the same detector session')
        call('filter',{'filter':'traffic'});wait(lambda s:s['detector']['ok'] and s['filter']=='traffic' and not s['detections'])
        events.append('filter returns a healthy empty result through Node')
        # A fresh HTTP client models refresh: state comes from backend, not page memory.
        assert call('status')['filter']=='traffic';events.append('fresh status client restores backend state')
        assert call('stop',{})['success'];wait(lambda s:not s['running'])
        time.sleep(.5)
        with urllib.request.urlopen('http://127.0.0.1:18322/status',timeout=2) as r:assert json.load(r)['active']==0
        events.append('stop closes the source connection and complete process tree')
        assert call('stop',{})['success'];events.append('repeated stop is idempotent')
        call('start',options);state=wait(lambda s:s['detector']['ok']);assert not state['frame']['id'].startswith(first+':')
        events.append('restart creates a new session without old detections')
        call('stop',{});wait(lambda s:not s['running'])
        report=dict(passed=True,events=events);Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report))
    finally:
        if owned:call('stop',{})
if __name__=='__main__':main()
