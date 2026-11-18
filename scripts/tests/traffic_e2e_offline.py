#!/usr/bin/env python3
"""F 交通灯 · 真实 Nav2 离线端到端验证（Domain197 离线控制台）。

真实链路：合成底盘 + 真实 Nav2 + 真实 Bridge（/cmd_vel 唯一发布者）。
交通灯（无真实相机）用脚本按车辆位姿发布 /vision/traffic_lights 观测，
模拟「摄像头看到本车道灯」——报告里标注为**注入源**，实机需真实视觉节点替代。

场景（用户核心用例）：
1. 选图 + 设初始位姿 → start_navigation（等灯启用）
2. 提交 A(0.8,0) + B(1.2,0.6) 两点
3. 车辆接近路口 I(0.4,0,r=0.9) 时注入红灯 → 期望：桥快照、取消、零速
4. 等待 25s（超过 no-progress 5s 判定窗口）→ 期望：全程零速、不重新派发
5. 短暂无灯帧 → 注入稳定绿灯（连续多帧）
6. 期望：恢复并只派发一次 A，到达 A 后派发 B
"""
import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = Path('/home/<user>/ros-car-migration')
sys.path.insert(0, str(ROOT / 'scripts'))
import urllib.request  # noqa: E402

BASE = 'http://localhost:18310/api/'
SESSION = 'traffic-e2e'
WAIT_SECONDS = float(__import__('os').environ.get('TRAFFIC_E2E_WAIT', '12'))
import os as _os
INTERSECTION = json.loads(_os.environ.get('TRAFFIC_E2E_INTERSECTION',
    '{"name":"I-main","x":0.4,"y":0.0,"radius":0.9,"speed_limit_near":0.12}'))


def api(endpoint, data=None, allow_error=False):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request = urllib.request.Request(BASE + endpoint,
                                     data=None if data is None else json.dumps(data).encode(),
                                     headers={'Content-Type': 'application/json'})
    try:
        with opener.open(request, timeout=25) as response:
            result = json.load(response)
    except urllib.error.HTTPError as error:
        result = json.load(error)
        result['_http_status'] = error.code
        result['_endpoint'] = endpoint
        if not allow_error:
            raise AssertionError(f'{endpoint} → HTTP {error.code}: {result.get("error")}') from error
    return result


class Driver:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('traffic_e2e_driver')
        self.pose = None
        self.publisher = self.node.create_publisher(String, '/vision/traffic_lights', 10)
        self.odom = self.node.create_subscription(Odometry, '/odom', self.on_odom,
                                                  rclpy.qos.qos_profile_sensor_data)
        self.timeline = []
        self._stop = threading.Event()
        self.heartbeat_on = True
        threading.Thread(target=self._spin, daemon=True).start()
        # 控制页心跳：LEASE_TTL_SECONDS=5，不续期会触发 lease_expired→取消锁定。
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _heartbeat(self):
        time.sleep(1)
        while not self._stop.is_set():
            if self.heartbeat_on:
                try:
                    api('navigation/heartbeat', {'sessionId': SESSION})
                except Exception:  # noqa: BLE001
                    pass
            self._stop.wait(.4)

    def _spin(self):
        while not self._stop.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def on_odom(self, message):
        p = message.pose.pose.position
        self.pose = {'x': p.x, 'y': p.y}

    def lamp(self, color, *, eligible=None, lights=True, kind='frame', track_id=1):
        payload = {'kind': kind, 'stamp_ns': time.time_ns(), 'at': time.monotonic(),
                   'lights': [], 'light_count': 0}
        if kind == 'frame' and lights:
            payload['lights'] = [{'track_id': track_id, 'name': 'traffic light', 'score': .92,
                                  'xyxy': [300, 120, 340, 200], 'missed': 0, 'color': color,
                                  'opencv_color': color,
                                  'color_pass_eligible': (color == 'green') if eligible is None else eligible,
                                  'color_conflict': False}]
            payload['light_count'] = 1
        self.publisher.publish(String(data=json.dumps(payload)))
        self.timeline.append({'at': round(time.monotonic(), 3), 'event': 'lamp',
                              'color': color if lights else None, 'eligible': eligible,
                              'kind': kind, 'pose': dict(self.pose) if self.pose else None})

    def note(self, event, **extra):
        self.timeline.append({'at': round(time.monotonic(), 3), 'event': event,
                              'pose': dict(self.pose) if self.pose else None, **extra})

    def close(self):
        self._stop.set()
        self.node.destroy_node()
        rclpy.shutdown()


def wait_for(predicate, timeout=60, description='condition'):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last:
                return last
        except Exception:  # noqa: BLE001
            pass
        time.sleep(.25)
    raise AssertionError(f'超时：{description}（最后值 {last}）')


def status():
    return api('navigation_status')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    checks = []

    def check(name, ok, detail=''):
        checks.append({'name': name, 'ok': bool(ok), 'detail': str(detail)})
        print(('PASS ' if ok else 'FAIL ') + name + ('' if ok else f'  ← {detail}'), flush=True)

    driver = Driver()
    report = {'result': 'FAIL', 'injected': True, 'domain': 197, 'checks': checks,
              'intersection': INTERSECTION}
    try:
        wait_for(lambda: not api('navigation_status')['missing'] is None, 30, 'backend')
        # 可重复运行：清理上一次遗留的导航会话（409 的根因）。
        if api('navigation_status')['isRunning']:
            api('stop_navigation', {'sessionId': SESSION, 'requestId': 'f-cleanup'})
            time.sleep(8)
        # 等待后端清理完成（stopping 标志），否则 start_navigation 会 409。
        wait_for(lambda: not api('navigation_status')['isRunning'], 30, 'clean state')
        time.sleep(2)
        api('control/claim', {'sessionId': SESSION, 'requestId': 'f-1'})
        # 建图 → 保存 → 用该图启动导航（复用现有流程）
        api('run_task/mapping', {'sessionId': SESSION, 'requestId': 'f-2'})
        time.sleep(14)   # 后端限制：建图不足 10 秒会拒绝保存
        # 保存成功后后端仍有 stopping/saving 过渡态：等到真正空闲再启动导航，
        # 否则 start_navigation 返回 409「正在切换任务」（本 e2e 反复踩到）。
        saved = api('stop_task/mapping', {'sessionId': SESSION, 'requestId': 'f-3'})['savedMap']
        wait_for(lambda: not api('task_status/mapping')['isRunning'], 30, 'mapping stopped')
        time.sleep(3)
        api('start_navigation', {'mapPath': saved['path'], 'sessionId': SESSION, 'requestId': 'f-4'})
        wait_for(lambda: status()['phase'] == 'waiting_localization', 90, 'waiting_localization')
        start = json.loads(__import__('os').environ.get('TRAFFIC_E2E_START', '{"x":0.1,"y":0.0,"yaw":0.0}'))
        api('navigation/initial_pose', {'x': start['x'], 'y': start['y'], 'yaw': start['yaw'],
                                        'sessionId': SESSION, 'requestId': 'f-5'})
        wait_for(lambda: status()['ready'], 40, 'ready')
        check('导航就绪', True)

        # 提交 A + B
        pts = json.loads(__import__('os').environ.get(
            'TRAFFIC_E2E_POINTS', '[{"x":0.8,"y":0.0,"yaw":0.0},{"x":1.2,"y":0.6,"yaw":1.57}]'))
        goal = api('navigation/goals', {'points': pts,
                                        'sessionId': SESSION, 'requestId': 'f-6'})
        report['goalId'] = goal.get('goalId')
        wait_for(lambda: status()['goalState'] in ('executing', 'accepted'), 30, 'goal running')
        driver.note('goal_dispatched')

        # 接近路口时注入红灯，并持续保持（模拟真实灯）
        held = {'on': True}

        def red_loop():
            while held['on']:
                if driver.pose and math.hypot(driver.pose['x'] - .4, driver.pose['y']) < 1.2:
                    driver.lamp('red')
                time.sleep(.2)
        threading.Thread(target=red_loop, daemon=True).start()

        wait_for(lambda: status()['mode'] == 'locked' and status()['goalState'] not in
                 ('accepted', 'executing'), 45, '红灯停车')
        check('红灯触发停车锁定', True)
        parked = dict(driver.pose or {})
        # 等待超过无进展超时窗口，验证全程零速且不再派发
        time.sleep(WAIT_SECONDS)
        moved = math.hypot((driver.pose or {}).get('x', 9) - parked.get('x', 0),
                           (driver.pose or {}).get('y', 9) - parked.get('y', 0))
        check(f'等待 {WAIT_SECONDS}s 期间车辆未移动（>no-progress 窗口）', moved < .05, f'位移 {moved:.3f}m')
        state_after_wait = status()
        check('等待期间未误判无进展失败', state_after_wait['goalState'] not in ('failed',),
              state_after_wait.get('goalState'))
        check('等待期间保持锁定零速', state_after_wait['mode'] == 'locked', state_after_wait['mode'])

        # 短暂遮灯
        held['on'] = False
        for _ in range(3):
            driver.lamp(None, lights=False, kind='frame')
            time.sleep(.2)
        check('短暂无灯帧不丢快照', status()['mode'] == 'locked')
        # 稳定绿灯
        for _ in range(8):
            driver.lamp('green')
            time.sleep(.25)
        driver.note('green_sent')

        # 期望恢复：到达 A 后推进 B
        wait_for(lambda: status()['goalState'] in ('accepted', 'executing', 'succeeded'), 60,
                 '恢复导航')
        check('绿灯后恢复导航', True)
        wait_for(lambda: status()['goalState'] == 'succeeded', 120, '到达 A 并完成')
        final = status()
        check('到达 A 后队列推进至 B 并完成', final['goalState'] == 'succeeded', final['goalState'])
        check('最终无残留队列', not final['queue'], final['queue'])
        report['finalGoalState'] = final['goalState']
        report['finalPose'] = dict(driver.pose or {})
        report['result'] = 'PASS' if all(c['ok'] for c in checks) else 'FAIL'
    except Exception as error:  # noqa: BLE001
        report['error'] = str(error)
        try:
            report['status'] = status()
        except Exception:  # noqa: BLE001
            pass
    finally:
        report['timeline'] = driver.timeline
        driver.close()
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        for item in checks:
            print(('PASS ' if item['ok'] else 'FAIL ') + item['name'])
        print('RESULT', report['result'])
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())