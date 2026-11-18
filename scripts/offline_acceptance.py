#!/usr/bin/env python3
"""HTTP acceptance against real GMapping/AMCL/Nav2 and a moving synthetic robot.

第二阶段新增：控制会话归属（claim/心跳门控/显式接管/旧租约失效）、
请求标识幂等重放、任务事件记录与结构化就绪检查。
"""
import argparse
import json
import os
import itertools
import math
import signal
import rclpy
from nav_msgs.msg import Odometry
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
BASE = 'http://localhost:18310/api/'
SESSION = 'acceptance-control-page'
OBSERVER = 'acceptance-observer-page'
_request_ids = itertools.count(1)

def rid():
    return f'acceptance-req-{next(_request_ids)}'

def api(endpoint, data=None, allow_error=False):
    request = urllib.request.Request(BASE + endpoint,
        data=None if data is None else json.dumps(data).encode(), headers={'Content-Type': 'application/json'})
    try:
        with opener.open(request, timeout=25) as response: result = json.load(response)
    except urllib.error.HTTPError as error:
        result = json.load(error)
        if not allow_error: raise AssertionError(f'{endpoint}: {result}') from error
    if data is not None and not allow_error: assert result.get('success'), (endpoint, result)
    return result

def wait_for(predicate, timeout=30, description='condition'):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last: return last
        except (OSError, ValueError): pass
        time.sleep(.25)
    raise AssertionError(f'Timeout: {description}; last={last}')

def fetch_events():
    """按游标拉全事件环（单次最多返回 200 条）。"""
    events, cursor = [], 0
    while True:
        with opener.open(f'{BASE}navigation/events?since={cursor}', timeout=15) as response:
            payload = json.load(response)
        batch = payload.get('events', [])
        events.extend(batch)
        if not batch or payload.get('latest', 0) <= (batch[-1]['seq'] if batch else cursor):
            return events
        cursor = batch[-1]['seq']

def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '197'
    checks = []
    rclpy.init(); observer = rclpy.create_node('offline_acceptance_observer')
    odometry = []
    observer.create_subscription(Odometry, '/odom', lambda msg: odometry.append(msg), rclpy.qos.qos_profile_sensor_data)
    spin_stop = threading.Event()
    def spin():
        while not spin_stop.is_set(): rclpy.spin_once(observer, timeout_sec=.05)
    spin_thread = threading.Thread(target=spin); spin_thread.start()
    finish = threading.Event(); heartbeat_enabled = threading.Event(); heartbeat_enabled.set()
    def heartbeat():
        while not finish.wait(.4):
            if heartbeat_enabled.is_set():
                try: api('navigation/heartbeat', {'sessionId': SESSION})
                except Exception: pass
    thread = threading.Thread(target=heartbeat, daemon=True); thread.start()
    report = {'result': 'FAIL', 'synthetic': True, 'domain': 197, 'checks': checks,
              'controlSession': SESSION, 'observerSession': OBSERVER}
    def check(name): checks.append(name); print('PASS ' + name, flush=True)
    def status(): return api('navigation_status')
    def idle(): return status()['goalState'] not in ('accepted', 'executing', 'canceling')
    try:
        wait_for(lambda: api('list_maps'), description='backend')
        wait_for(lambda: not status()['missing'], timeout=15, description='sensors')
        # ---- 控制权归属（P1-3）----
        api('control/claim', {'sessionId': SESSION, 'requestId': rid()})
        assert status()['controlOwner'] == SESSION
        denied_claim = api('control/claim', {'sessionId': OBSERVER, 'requestId': rid()}, allow_error=True)
        assert not denied_claim['success'] and '接管' in denied_claim['error']
        check('second page cannot claim while control page holds the lease')
        api('run_task/mapping', {'sessionId': SESSION, 'requestId': rid()})
        time.sleep(6)
        manual = api('control/manual_enable', {'sessionId': SESSION, 'requestId': rid()})
        foreign = api('navigation/velocity', {'sessionId': OBSERVER, 'controlId': manual['controlId'],
                                              'sequence': 999, 'linearX': .3, 'angularZ': 0}, allow_error=True)
        assert not foreign['success'], '旧租约/他会话不得发速度'
        check('velocity from foreign session rejected even with valid controlId')
        for _ in range(20):
            api('navigation/velocity', {'sessionId': SESSION, 'controlId': manual['controlId'],
                                        'sequence': _, 'linearX': .1, 'angularZ': 0})
            time.sleep(.05)
        # 急停属于安全操作：观察页也可执行。
        api('navigation/estop', {'sessionId': OBSERVER, 'requestId': rid()})
        # goals 不再隐式清急停；用显式复位恢复。
        api('control/reset_estop', {'sessionId': SESSION, 'requestId': rid()})
        time.sleep(5)
        maps_dir = Path(os.environ['ROBOT_MAPS_DIR'])
        permissions = maps_dir.stat().st_mode & 0o777
        assert os.geteuid() != 0, 'Run fixture as normal user to verify save permission failure'
        try:
            maps_dir.chmod(0o555)
            failed_save = api('stop_task/mapping', {'sessionId': SESSION, 'requestId': rid()}, allow_error=True)
            assert not failed_save['success'] and api('task_status/mapping')['isRunning']
        finally:
            maps_dir.chmod(permissions)
        check('save failure preserves GMapping and permits retry')
        saved = api('stop_task/mapping', {'sessionId': SESSION, 'requestId': rid()})['savedMap']; assert saved
        report['map'] = saved
        assert Path(saved['yaml']).stat().st_size > 0 and Path(saved['image']).stat().st_size > 0
        assert not api('task_status/mapping')['isRunning']
        assert api('list_maps')['maps'][0]['path'] == saved['path']
        assert api('stop_task/mapping', {'sessionId': SESSION, 'requestId': rid()})['savedMap'] is None
        check('moving GMapping → save-before-stop → new map first → repeat stop')
        api('start_navigation', {'mapPath': saved['path'], 'sessionId': SESSION, 'requestId': rid()})
        # 启动阶段先等 Nav2 真正进入“等待人工定位”，不要用固定 sleep 猜测。
        wait_for(lambda: status()['phase'] == 'waiting_localization', timeout=75,
                 description='waiting_localization phase')
        time.sleep(32)
        assert not status()['ready']
        waiting = status()
        assert waiting['phase'] == 'waiting_localization'
        assert waiting['checks']['localization']['waitingUser'] is True
        assert waiting['phase'] != 'failed' and not waiting['error']
        check('waiting for human localization does not time out Nav2 activation')
        rejected = api('navigation/goals', {'points': [{'x': .7, 'y': 0}], 'sessionId': SESSION,
                                            'requestId': rid()}, allow_error=True)
        assert not rejected['success']
        check('navigation rejects goal before AMCL initial pose')
        # Robot moved about 0.1 m during mapping. GMapping map frame starts at odom origin.
        api('navigation/initial_pose', {'x': .1, 'y': 0., 'yaw': 0., 'sessionId': SESSION, 'requestId': rid()})
        wait_for(lambda: status()['ready'], timeout=20, description='AMCL and map TF')
        ready = status()
        # 结构化就绪检查：六类子系统 + 最近更新 + 下一步（P2-1）。
        for key in ('lidar', 'odometry', 'tf', 'localization', 'nav2', 'control'):
            assert key in ready['checks'], ready['checks']
            assert ready['checks'][key]['ok'] is True, (key, ready['checks'])
            assert 'detail' in ready['checks'][key]
        assert isinstance(ready['checks']['lidar']['ageSeconds'], (int, float))
        assert ready['checks']['control']['owner'] == SESSION
        assert ready['nextAction']
        check('readiness checks expose six subsystems with recency and next action')
        goal_request_id = rid()
        goal = api('navigation/goals', {'points': [{'x': .8, 'y': .0, 'yaw': 0.}],
                                        'sessionId': SESSION, 'requestId': goal_request_id})
        report['firstGoalId'] = goal['goalId']
        # 重复 requestId 幂等重放：不重复提交目标（P1-2 乱序/重复请求）。
        replay = api('navigation/goals', {'points': [{'x': .8, 'y': .0, 'yaw': 0.}],
                                          'sessionId': SESSION, 'requestId': goal_request_id})
        assert replay['goalId'] == goal['goalId'], replay
        busy = status()
        assert busy['goalId'] == goal['goalId'] and not busy['queue'], busy
        check('duplicate requestId replays result without resubmitting goal')
        wait_for(idle, timeout=75, description='single goal terminal')
        assert status()['goalState'] == 'succeeded', status()
        assert odometry and math.hypot(odometry[-1].pose.pose.position.x - .8, odometry[-1].pose.pose.position.y) < .4
        report['singleGoalTruePose'] = [odometry[-1].pose.pose.position.x, odometry[-1].pose.pose.position.y]
        check('real NavigateToPose single goal succeeds with physical pose check')
        api('navigation/goals', {'points': [{'x': 1.2, 'y': 0., 'yaw': 0.}, {'x': 1.2, 'y': .6, 'yaw': 1.57}],
                                 'sessionId': SESSION, 'requestId': rid()})
        wait_for(idle, timeout=100, description='multi goal terminal')
        assert status()['goalState'] == 'succeeded', status()
        assert math.hypot(odometry[-1].pose.pose.position.x - 1.2, odometry[-1].pose.pose.position.y - .6) < .4
        check('multiple goals advance only after Action success with physical pose check')
        api('navigation/goals', {'points': [{'x': -1., 'y': 0.}], 'sessionId': SESSION, 'requestId': rid()})
        api('navigation/cancel', {'sessionId': SESSION, 'requestId': rid()})
        assert status()['goalState'] == 'canceled' and status()['mode'] == 'locked'
        check('cancel is confirmed and motion locked')
        api('navigation/goals', {'points': [{'x': -1., 'y': 0.}], 'sessionId': SESSION, 'requestId': rid()})
        takeover = api('control/manual_enable', {'sessionId': SESSION, 'requestId': rid()})
        assert takeover['controlId'] and status()['mode'] == 'manual' and idle()
        check('manual takeover waits for navigation cancellation')
        api('navigation/estop', {'sessionId': SESSION, 'requestId': rid()})
        assert not api('navigation/velocity', {'sessionId': SESSION, 'controlId': takeover['controlId'],
                                               'linearX': .1, 'angularZ': 0}, True)['success']
        check('estop invalidates previous manual session')
        # goals 不再隐式清急停；下一条目标前必须先显式复位。
        api('control/reset_estop', {'sessionId': SESSION, 'requestId': rid()})
        # ---- 显式接管：先取消并确认停止，再生成新租约（P1-3）----
        api('navigation/goals', {'points': [{'x': .6, 'y': .2, 'yaw': 0.}], 'sessionId': SESSION, 'requestId': rid()})
        wait_for(lambda: status()['goalState'] in ('accepted', 'executing'), timeout=20, description='goal running for takeover')
        takeover_result = api('control/takeover', {'sessionId': OBSERVER, 'requestId': rid()})
        assert takeover_result['owner'] == OBSERVER
        after_takeover = status()
        assert after_takeover['mode'] == 'locked' and after_takeover['goalState'] not in ('accepted', 'executing')
        assert after_takeover['controlOwner'] == OBSERVER
        denied_heartbeat = api('navigation/heartbeat', {'sessionId': SESSION}, allow_error=True)
        assert not denied_heartbeat['success'], '旧租约不能续期'
        denied_goal = api('navigation/goals', {'points': [{'x': .6, 'y': .2}], 'sessionId': SESSION,
                                               'requestId': rid()}, allow_error=True)
        assert not denied_goal['success'], '旧租约不能发目标'
        # 任意页面仍可急停；接管后把控制权交还给主测试会话。
        api('navigation/estop', {'sessionId': SESSION, 'requestId': rid()})
        api('control/takeover', {'sessionId': SESSION, 'requestId': rid()})
        api('control/reset_estop', {'sessionId': SESSION, 'requestId': rid()})
        assert status()['controlOwner'] == SESSION
        check('explicit takeover cancels motion before granting new lease')
        # ---- 心跳失联：观察页心跳不能续期，过期即释放归属（P1-3/P1-4）----
        api('navigation/goals', {'points': [{'x': -1., 'y': 0.}], 'sessionId': SESSION, 'requestId': rid()})
        heartbeat_enabled.clear()
        observer_renewals = 0
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            result = api('navigation/heartbeat', {'sessionId': OBSERVER}, allow_error=True)
            if not result.get('success'): observer_renewals += 1
            time.sleep(.4)
        assert observer_renewals >= 3, '观察页心跳必须全部被拒绝'
        wait_for(lambda: status()['mode'] == 'locked' and idle(), timeout=10, description='lease expiry')
        assert not status()['queue']
        assert status()['controlOwner'] is None, '失联后归属必须释放'
        heartbeat_enabled.set()
        time.sleep(1)
        still_locked = status()
        assert still_locked['mode'] == 'locked'
        # 恢复心跳不等于恢复控制：本会话心跳也被拒绝（归属已释放），需重新 claim。
        assert not api('navigation/heartbeat', {'sessionId': SESSION}, allow_error=True)['success']
        api('control/claim', {'sessionId': SESSION, 'requestId': rid()})
        check('browser heartbeat loss cancels; observer renewal never keeps motion; reconnect never resumes')
        api('navigation/goals', {'points': [{'x': 2.8, 'y': -1.5, 'yaw': 0.}], 'sessionId': SESSION, 'requestId': rid()})
        os.kill(int(os.environ['ROBOT_SIM_PID']), signal.SIGUSR1)
        wait_for(lambda: status()['mode'] == 'locked' and idle(), timeout=10, description='sensor dropout')
        assert status()['missing']
        os.kill(int(os.environ['ROBOT_SIM_PID']), signal.SIGUSR1)
        wait_for(lambda: status()['ready'], timeout=10, description='sensor recovery')
        assert status()['mode'] == 'locked'
        time.sleep(.5)
        assert abs(odometry[-1].twist.twist.linear.x) < .001
        check('sensor dropout cancels and stops; recovery stays locked')
        # Continue publishing packets and fresh TF while replaying old samples.
        # Receipt freshness must not hide a frozen acquisition task.
        api('navigation/goals', {'points': [{'x': -1., 'y': .5}], 'sessionId': SESSION, 'requestId': rid()})
        os.kill(int(os.environ['ROBOT_SIM_PID']), signal.SIGUSR2)
        try:
            wait_for(lambda: status()['mode'] == 'locked' and idle() and not status()['ready'],
                     timeout=10, description='replayed sample rejection')
            frozen = status()
            for name in ('lidar', 'odometry'):
                assert not frozen['checks'][name]['ok']
                assert frozen['checks'][name]['receiptAgeSeconds'] < .5
                assert frozen['checks'][name]['sampleAgeSeconds'] > 1.5
        finally:
            os.kill(int(os.environ['ROBOT_SIM_PID']), signal.SIGUSR2)
        wait_for(lambda: status()['ready'], timeout=10, description='new sample recovery')
        assert status()['mode'] == 'locked'
        check('replayed stale samples cancel despite fresh packets and TF; recovery stays locked')
        # 桥接在 Action 之前拒绝越界/未知目标，避免 NavFn 超时后才发现无路。
        for point, expected in [({'x': 50., 'y': 50.}, '超出地图范围'),
                                ({'x': 5., 'y': 5.}, '未知区域')]:
            rejected_goal = api('navigation/goals', {'points': [point], 'sessionId': SESSION,
                                                      'requestId': rid()}, allow_error=True)
            assert not rejected_goal['success'], rejected_goal
            assert expected in rejected_goal['error'], rejected_goal
        assert status()['goalState'] not in ('accepted', 'executing', 'canceling')
        assert status()['mode'] == 'locked'
        check('out-of-map and unknown goals rejected before Action')
        # Kill exactly our isolated controller child, identified through the backend's descendants.
        api('navigation/goals', {'points': [{'x': -1., 'y': 0.}], 'sessionId': SESSION, 'requestId': rid()})
        def descendants(pid):
            children_file = Path(f'/proc/{pid}/task/{pid}/children')
            if not children_file.exists(): return []
            children = [int(x) for x in children_file.read_text().split()]
            return children + [child for parent in children for child in descendants(parent)]
        controller = [pid for pid in descendants(int(os.environ['ROBOT_BACKEND_PID']))
                      if Path(f'/proc/{pid}/cmdline').exists() and b'/controller_server' in Path(f'/proc/{pid}/cmdline').read_bytes()]
        assert len(controller) == 1, controller
        os.kill(controller[0], signal.SIGTERM)
        wait_for(lambda: status()['mode'] == 'locked' and idle() and not status()['ready'], timeout=15, description='controller exit')
        assert not status()['ready']
        check('controller process exit invalidates readiness and cancels motion')
        # ---- 任务事件（P2-2）：运行 ID、目标 ID、原因，增量游标 ----
        events = fetch_events()
        seen = {event['type'] for event in events}
        for expected in ('startup_begin', 'localization_requested', 'localization_confirmed',
                         'goal_submitted', 'goal_accepted', 'goal_succeeded', 'goal_canceled',
                         'goal_failed', 'cancel_confirmed', 'estop', 'manual_enabled',
                         'lease_expired', 'sensor_dropout', 'sensor_recovered',
                         'control_claimed', 'control_takeover', 'heartbeat_denied'):
            assert expected in seen, f'缺少事件 {expected}；实际 {sorted(seen)}'
        assert all('seq' in e and 'at' in e and 'runId' in e and 'type' in e for e in events)
        goal_events = [e for e in events if e['type'] == 'goal_succeeded']
        assert goal_events and all(e.get('goalId') for e in goal_events)
        submitted = [e for e in events if e['type'] == 'goal_submitted' and e.get('requestId')]
        assert submitted, '目标事件必须关联请求标识'
        failed_events = [e for e in events if e['type'] == 'goal_failed']
        assert failed_events and failed_events[0]['reason']
        # 错误不能被普通轮询擦掉：failed 之后连续 status 轮询仍保留错误，直至新操作。
        for _ in range(4): status(); time.sleep(.1)
        assert status()['error'], '目标失败后的错误必须保留，不被轮询清除'
        check('task events record run, goal, reason and errors survive polling')
        api('stop_navigation', {'sessionId': SESSION, 'requestId': rid()})
        assert not status()['isRunning']
        check('stop navigation confirms shutdown')
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
        try: report['status'] = status()
        except Exception: pass
        try: report['events'] = fetch_events()[-40:]
        except Exception: pass
        raise
    finally:
        finish.set(); thread.join(timeout=2)
        spin_stop.set(); spin_thread.join(timeout=2); observer.destroy_node(); rclpy.shutdown()
        (args.output / 'acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))

if __name__ == '__main__': main()
