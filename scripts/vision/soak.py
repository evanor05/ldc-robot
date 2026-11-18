#!/usr/bin/env python3
"""Record real wall-clock stability evidence; never shorten duration silently."""
import argparse
import json
from pathlib import Path
import time
import urllib.request


def fetch(port,path='/status'):
    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}',timeout=2) as r:return json.load(r)

def rss_snapshot():
    rows=[]
    for p in Path('/proc').glob('[0-9]*'):
        try:
            arguments=(p/'cmdline').read_bytes().decode().split('\0')
            if any(arg.endswith(s) for arg in arguments for s in ('ldc_vision/lib/ldc_vision/detector','ldc_vision/worker.py','ldc_camera/lib/ldc_camera/http_video_publisher')):
                rss=next(int(l.split()[1]) for l in (p/'status').read_text().splitlines() if l.startswith('VmRSS:'))
                rows.append(dict(pid=int(p.name),rss_kib=rss))
        except (OSError,StopIteration):pass
    return rows

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=1800);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    start=time.monotonic();samples=[];errors=[]
    with (out/'samples.jsonl').open('w') as log:
        while time.monotonic()-start<args.seconds:
            try:
                state=fetch(18321);source=fetch(18322)
                sample=dict(elapsed=round(time.monotonic()-start,2),state=state,source=source,processes=rss_snapshot())
                samples.append(sample);log.write(json.dumps(sample)+'\n');log.flush()
                if not state['camera']['ok'] or not state['detector']['ok']:errors.append(sample['elapsed'])
                print(json.dumps({k:state[k] for k in ('fps','p95Ms','pending','completed')}),flush=True)
            except Exception as e:errors.append(str(e));print(str(e),flush=True)
            time.sleep(min(10,max(0,args.seconds-(time.monotonic()-start))))
    report=dict(requested_seconds=args.seconds,elapsed_seconds=round(time.monotonic()-start,2),samples=len(samples),errors=errors,
                minimum_fps=min((s['state']['fps'] for s in samples),default=0),
                maximum_p95_ms=max((s['state']['p95Ms'] or 0 for s in samples),default=0),
                maximum_pending=max((s['state']['pending'] for s in samples),default=0),
                maximum_source_connections=max((s['source']['max_active'] for s in samples),default=0))
    report['passed']=bool(samples) and not errors and report['minimum_fps']>=8 and report['maximum_p95_ms']<=300 and report['maximum_pending']<=1 and report['maximum_source_connections']==1
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    raise SystemExit(0 if report['passed'] else 1)
if __name__=='__main__':main()
