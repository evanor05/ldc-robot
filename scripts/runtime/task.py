#!/usr/bin/env python3
"""Start map/navigation nodes via the existing API, release the temporary lease afterwards."""
import argparse
import json
import os
import threading
import time
import urllib.error
import urllib.request
import uuid


def main():
    p=argparse.ArgumentParser();p.add_argument('task',choices=['mapping','navigation']);p.add_argument('--map',help='已保存地图名，不含 .yaml')
    args=p.parse_args();base=f'http://127.0.0.1:{os.environ.get("ROBOT_API_PORT","3000")}/api';session='startup-'+uuid.uuid4().hex
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def call(endpoint,data=None):
        body=None if data is None else json.dumps(dict(sessionId=session,requestId=uuid.uuid4().hex,**data)).encode()
        request=urllib.request.Request(base+'/'+endpoint,data=body,headers={'Content-Type':'application/json'})
        try:
            with opener.open(request,timeout=15) as response:
                result=json.load(response)
                if result.get('success') is False:raise ValueError(result.get('error','操作失败'))
                return result
        except urllib.error.HTTPError as error:
            try:detail=json.load(error).get('error',str(error))
            finally:error.close()
            raise RuntimeError(detail) from error
    claimed=False
    stop=threading.Event();heartbeats=None
    def heartbeat():
        while not stop.wait(.4):
            try:call('navigation/heartbeat',{})
            except (OSError,ValueError,RuntimeError):return
    try:
        map_path=None
        if args.task=='navigation':
            if not args.map:p.error('导航必须 --map 地图名；不发送初始位置或目标')
            maps=call('list_maps')['maps'];match=next((item for item in maps if item['name']==args.map),None)
            if not match:raise ValueError('该地图不在后端已保存地图列表中')
            map_path=match['path']
        # A freshly created bridge needs its first scan/odom/TF callbacks.
        deadline=time.monotonic()+10
        while True:
            state=call('navigation_status')
            checks=state.get('checks',{})
            if state.get('isRunning') or all(checks.get(key,{}).get('ok') for key in ('lidar','odometry','tf')):break
            if time.monotonic()>deadline:raise RuntimeError('传感器未就绪：'+'、'.join(state.get('missing',[])))
            time.sleep(.2)
        call('control/claim',{});claimed=True
        heartbeats=threading.Thread(target=heartbeat,daemon=True);heartbeats.start()
        result=call('run_task/mapping',{}) if args.task=='mapping' else call('start_navigation',dict(mapPath=map_path))
        print(json.dumps(result,ensure_ascii=False))
        print('节点已启动；临时控制权随即释放。遥控、初始位置和目标请在网页操作。未启用运动。')
    finally:
        stop.set()
        if heartbeats:heartbeats.join(timeout=2)
        if claimed:
            try:call('control/release',{})
            except RuntimeError as error:print('临时控制权已失效或被释放，未接管其他会话：',error)
if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError) as error:raise SystemExit('任务未启动：'+str(error))
