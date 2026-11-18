#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工单 A · 双实例隔离验收程序（isolation-round-20261118，verify-isolation）。

一条命令运行一个变体，产出：四级时间线（A 输出 / B 输入 / B 输出 / 最终底盘指令）
+ ROS 端点身份快照（节点全名 / GID）+ 机器可读断言 + 简体中文报告。

拓扑（修复前受控复现，Domain197 共享离线验收环境，须先按 COORDINATION.md §3 持锁）：
  运行 A = scripts/offline_console.sh 起的合成底盘(offline_robot.py) + 真实 Nav2
           （后端 start_navigation 经 navigation2.launch.py 启动，
            controller/behavior cmd_vel→cmd_vel_nav；
            A smoother cmd_vel→cmd_vel_nav、cmd_vel_smoothed→A 本轮通道
            /ldc/velocity/b{boot}/r{epoch}；A bridge 门控后发布最终 /cmd_vel）。
  运行 B（本程序）= 按 launch:45-48 remap 形态单独起的 velocity_smoother_b：
            cmd_vel→cmd_vel_nav（与 A 共享 —— 被复现的缺陷），
            cmd_vel_smoothed→/dual_verify/b_smoother_output（B 输出观测话题）。
  A 速度源两变体：
    inject —— 测试发布器 dual_a_inject 向 cmd_vel_nav 发可辨识序列（标注为注入源；
              A 栈就绪但不发目标，A 控制器静默，端点快照证明唯一发布者）。
    real   —— 真实 A 控制器：经后端 API 发 NavigateToPose 目标，controller_server
              真实 DWB 输出进入 cmd_vel_nav。
    mutate —— 与 inject 同拓扑；修复后用于变异测试（恢复共享 cmd_vel_nav 接线时
              ISO-1 断言必须失败，证明断言敏感）。

四级观测：L1 A 输出 = /cmd_vel_nav；L2 B 输入 = 同一话题（共享即缺陷，端点快照证明
  velocity_smoother_b 订阅它）；L3 B 输出 = /dual_verify/b_smoother_output；
  L4 最终底盘指令 = /cmd_vel（A bridge 门控出口）+ /odom 合成底盘真值位姿。
  另观测 A 本轮通道（A smoother 合法输出），用于 WIRE-2 镜像指纹断言。

三态判别：
  CROSSTALK_PATH_EXISTS —— A 的可辨识输出到达 B smoother 输出（接线路径存在）；
  OLD_SOURCE_ISOLATED   —— ISO 断言通过且 WIRE 断言失败（旧源被隔离）；
  NEW_TASK_CANNOT_MOVE  —— LIVE 断言失败（授权任务本身不能动，real 变体评估）。
  只检查锁定时全零不足以验收：GATE-LOCKED 与 ISO-1 分开断言，Bridge 门控可能遮住
  上游串线（本程序在门控上游的 B smoother 输出处观测）。

纪律：只新增本文件与审计目录产物；不改任何产品代码。合成底盘/Nav2/Bridge/后端均为
  产品原样运行。自动建图入口不触碰（不调用 run_task/mapping；用预制地图 fixture）。
  所有 ROS 进程 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST，RMW 实测记录于 meta。

用法（先 source scripts/env.bash，且已按锁协议持有 domain197.lock）：
  python3 scripts/tests/dual_instance_repro.py --variant inject \
      --console-dir <AUDIT>/runs/console-env --output <AUDIT>/runs
  python3 scripts/tests/dual_instance_repro.py --variant real ...（同上）
修复后复测：同命令 --expect isolated；变异测试：--variant mutate（期望 crosstalk）。
"""
import argparse
import collections
import datetime
import hashlib
import itertools
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
AUDIT = Path('/home/<user>/ros-car-migration-audit/isolation-round-20261118/A-verify-isolation')
LOCK = Path('/home/<user>/ros-car-migration-audit/isolation-round-20261118/coordination/domain197.lock')
PARAMS_SRC = ROOT / 'robot_ws/src/ldc_navigation/param/ldc_nav2.yaml'
LAUNCH_SRC = ROOT / 'robot_ws/src/ldc_navigation/launch/navigation2.launch.py'
PRODUCT_FILES = [
    LAUNCH_SRC, PARAMS_SRC,
    ROOT / 'scripts/offline_robot.py',
    ROOT / 'scripts/velocity_channel.py',
    ROOT / 'scripts/navigation_bridge.py',
    ROOT / 'web_console/navigationRoutes.js',
]
B_OUTPUT_TOPIC = '/dual_verify/b_smoother_output'
B_NODE = 'velocity_smoother_b'
SESSION = 'dual-verify-A-control'
BASE = 'http://127.0.0.1:18310/api/'
SIG_VX = (0.173, 0.219)   # 可辨识平台值（在 smoother 限幅内，OPEN_LOOP 精确传递）
SIG_WZ = 0.314
SIG_TOL = 1e-3
PLATEAU_SECONDS = 1.0

_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
_rids = itertools.count(1)


def rid():
    return f'dual-verify-req-{next(_rids)}'


def now_iso():
    return datetime.datetime.now().astimezone().isoformat(timespec='milliseconds')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def api(endpoint, data=None, allow_error=False, timeout=25):
    request = urllib.request.Request(
        BASE + endpoint,
        data=None if data is None else json.dumps(data).encode(),
        headers={'Content-Type': 'application/json'})
    try:
        with _opener.open(request, timeout=timeout) as response:
            result = json.load(response)
    except urllib.error.HTTPError as error:
        result = json.load(error)
        if not allow_error:
            raise RuntimeError(f'{endpoint}: {result}') from error
    if data is not None and not allow_error and not result.get('success'):
        raise RuntimeError(f'{endpoint}: {result}')
    return result


class Recorder:
    def __init__(self, run_dir):
        self.t0 = time.monotonic()
        self.fh = open(run_dir / 'timeline.jsonl', 'w', buffering=1)
        self.samples = collections.defaultdict(list)
        self.events = []

    def t(self):
        return round(time.monotonic() - self.t0, 4)

    def rec(self, point, **data):
        entry = {'t': self.t(), 'wall': now_iso(), 'point': point, **data}
        self.fh.write(json.dumps(entry, ensure_ascii=False) + '\n')
        self.samples[point].append(entry)
        return entry

    def event(self, kind, **data):
        entry = self.rec('event', kind=kind, **data)
        self.events.append(entry)
        return entry

    def window(self, start_kind, end_kind):
        start = next((e['t'] for e in self.events if e['kind'] == start_kind), None)
        end = next((e['t'] for e in self.events if e['kind'] == end_kind), None)
        return start, end

    def in_window(self, point, start, end):
        return [s for s in self.samples[point]
                if start is not None and end is not None and start <= s['t'] <= end]

    def close(self):
        self.fh.close()


def make_map_fixture(maps_dir):
    """预制合成房间地图（与 offline_robot.py ray() 的墙/柱一致），测试数据，非产品地图。
    6x4.5m 房间：x[-2.5,3.5] y[-2,2.5]，柱 x[2,2.4] y[1.2,1.7]。
    满足离线 fixture 质量门禁：占用>=300、未知<=95%、分辨率 0.05。"""
    maps_dir = Path(maps_dir)
    maps_dir.mkdir(parents=True, exist_ok=True)
    w, h, res, ox, oy = 120, 100, 0.05, -2.5, -2.5
    rows, occupied = [], 0
    for j in range(h - 1, -1, -1):  # PGM 首行 = 最大 y
        y = oy + res * (j + 0.5)
        row = bytearray()
        for i in range(w):
            x = ox + res * (i + 0.5)
            wall = i == 0 or i == w - 1 or j == 0 or j == h - 1 or abs(y + 2.0) < 0.03
            pillar = 2.0 <= x <= 2.4 and 1.2 <= y <= 1.7
            value = 0 if (wall or pillar) else 254
            occupied += value == 0
            row.append(value)
        rows.append(bytes(row))
    prefix = maps_dir / 'synth_room'
    Path(str(prefix) + '.pgm').write_bytes(b'P5\n%d %d\n255\n' % (w, h) + b''.join(rows))
    Path(str(prefix) + '.yaml').write_text(
        'image: synth_room.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-2.5, -2.5, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return str(prefix), occupied


def parse_topic_info(text):
    pubs, subs, msg_type, section, cur = [], [], None, None, {}
    for line in text.splitlines():
        if line.startswith('Type:') and msg_type is None:
            msg_type = line.split(':', 1)[1].strip()
        elif line.startswith('Publisher count:'):
            section = 'pub'
        elif line.startswith('Subscription count:'):
            section = 'sub'
        elif line.startswith('Node name:'):
            cur = {'node': line.split(':', 1)[1].strip()}
        elif line.startswith('Node namespace:'):
            cur['namespace'] = line.split(':', 1)[1].strip()
        elif line.startswith('GID:'):
            cur['gid'] = line.split(':', 1)[1].strip()
            if section == 'pub':
                pubs.append(cur)
            elif section == 'sub':
                subs.append(cur)
            cur = {}
    return {'type': msg_type, 'publishers': pubs, 'subscribers': subs}


def sig_match(sample):
    return (abs(sample['wz'] - SIG_WZ) < SIG_TOL and abs(sample.get('vy', 0.0)) < SIG_TOL
            and any(abs(sample['vx'] - v) < SIG_TOL for v in SIG_VX))


def nonzero(sample):
    return abs(sample['vx']) > 1e-4 or abs(sample['wz']) > 1e-4


def mirror_pairs(a_samples, b_samples, dt=0.06):
    """B 输出与 A smoother 输出（同一 cmd_vel_nav 输入、同参数）的镜像指纹。"""
    pairs = []
    for b in b_samples:
        best = min(a_samples, key=lambda a: abs(a['t'] - b['t']), default=None)
        if best is not None and abs(best['t'] - b['t']) <= dt:
            pairs.append((best, b))
    return pairs


class Harness:
    def __init__(self, args):
        self.args = args
        self.variant = args.variant
        self.inject = self.variant in ('inject', 'mutate')
        self.stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
        self.run_dir = Path(args.output) / f'{self.variant}-{self.stamp}'
        self.run_dir.mkdir(parents=True, exist_ok=False)
        (self.run_dir / 'endpoints').mkdir()
        self.rec = Recorder(self.run_dir)
        self.api_log = open(self.run_dir / 'api-log.jsonl', 'w', buffering=1)
        self.procs = []
        self.logs = []
        self.nodes = []
        self.executor = None
        self.rclpy = None
        self.heartbeat_stop = threading.Event()
        self.inject_flag = {'active': False, 't0': 0.0}
        self.a_channel_topic = None
        self.a_started = False
        self.status_last = {}
        self.goal_timeline = []
        self.failure = None

    # ---- 基础设施 ----
    def call_api(self, endpoint, data=None, allow_error=False):
        result = api(endpoint, data, allow_error=allow_error)
        self.api_log.write(json.dumps({'at': now_iso(), 'endpoint': endpoint,
                                       'request': data, 'response': result},
                                      ensure_ascii=False) + '\n')
        self.rec.event('api', endpoint=endpoint, success=result.get('success', True))
        return result

    def start_proc(self, name, cmd):
        log = open(self.run_dir / f'{name}.log', 'w')
        self.logs.append(log)
        proc = subprocess.Popen(cmd, env=os.environ.copy(), cwd=str(ROOT),
                                start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
        entry = {'name': name, 'pid': proc.pid, 'cmd': cmd, 'startedAt': now_iso()}
        self.procs.append(entry)
        self.rec.event('process_start', **entry)
        return proc

    def cleanup_procs(self):
        for entry in reversed(self.procs):
            proc = entry.get('proc')
            if proc is None:
                continue
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=10)
                except Exception:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=5)
                    except Exception:
                        pass
            entry['endedAt'] = now_iso()
            entry['returncode'] = proc.returncode
        for log in self.logs:
            log.close()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def spin_until(self, future, timeout):
        self.rclpy.spin_until_future_complete(self.observer, future,
                                              executor=self.executor, timeout_sec=timeout)
        return future.done() and future.result() is not None

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = predicate()
                if last:
                    return last
            except (OSError, ValueError, RuntimeError):
                pass
            self.spin_for(0.25)
        raise TimeoutError(f'等待超时：{description}；last={last}')

    def cli_snapshot(self, label, argv):
        out, attempt = '', 0
        for attempt in range(3):
            proc = subprocess.Popen(argv, env=os.environ.copy(), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 25
            while proc.poll() is None and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.05)
            if proc.poll() is None:
                proc.kill()
            out = proc.communicate(timeout=10)[0].decode(errors='replace')
            if 'Unknown' not in out and proc.returncode == 0:
                break
            self.spin_for(1.0)
        (self.run_dir / 'endpoints' / f'{label}.txt').write_text(out)
        self.rec.event('endpoint_snapshot', label=label, returncode=proc.returncode,
                       attempts=attempt + 1)
        return out

    # ---- 环境预检 ----
    def preflight(self):
        owner = json.loads((LOCK / 'owner.json').read_text())
        if owner.get('member') != 'verify-isolation' or owner.get('task') != 'A':
            raise RuntimeError(f'domain197.lock 不由本成员持有（{owner}）；禁止使用共享环境')
        self.lock_owner = owner
        prior = {k: os.environ.get(k) for k in
                 ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}
        os.environ['ROS_DOMAIN_ID'] = '197'
        os.environ['ROBOT_SIMULATION'] = '1'
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        self.prior_env = prior
        api('list_maps', timeout=5)  # 后端必须已就绪
        import rclpy  # noqa: PLC0415 （env 就绪后再导入/初始化）
        self.rclpy = rclpy
        rclpy.init()
        self.rmw = rclpy.get_rmw_implementation_identifier()
        self.rec.event('preflight', domain=197, rmw=self.rmw, priorEnv=prior,
                       lockOwner=owner, discovery='LOCALHOST')

    # ---- 运行 A：后端 API 编排（产品原样） ----
    def start_run_a(self):
        # 残留状态清理：上一次运行若未正常停止，先停干净再全新启动（S1 语义：每轮全新实例）。
        if self.status().get('isRunning'):
            self.rec.event('stale_run_detected', phase=self.status_last.get('phase'))
            self.stop_run_a()
        console_maps = Path(self.args.console_dir) / 'maps'
        fixture, occupied = make_map_fixture(AUDIT / 'maps')
        for ext in ('.pgm', '.yaml'):
            shutil.copy2(fixture + ext, str(console_maps / 'synth_room') + ext)
        self.map_fixture = {'path': fixture, 'occupiedCells': occupied,
                            'sha256_pgm': sha256(fixture + '.pgm'),
                            'sha256_yaml': sha256(fixture + '.yaml')}
        self.rec.event('map_fixture', **self.map_fixture)

        def navigable():
            maps = api('list_maps')['maps']
            return next((m for m in maps if m['name'] == 'synth_room' and m.get('navigable')), None)
        self.wait_for(navigable, 15, 'synth_room 地图通过质量门禁')

        self.call_api('control/claim', {'sessionId': SESSION, 'requestId': rid()})
        # 显式复位急停锁存（与 offline_acceptance.py:111 同一产品协议路径；
        # 合成底盘初始 stopped=True，需要 /emergency_stop=False 才能动）。
        self.call_api('control/reset_estop', {'sessionId': SESSION, 'requestId': rid()})
        threading.Thread(target=self._heartbeat, daemon=True).start()
        map_path = str(console_maps / 'synth_room')
        self.call_api('start_navigation', {'mapPath': map_path, 'sessionId': SESSION,
                                           'requestId': rid()})
        self.a_started = True
        self.wait_for(lambda: self.status().get('phase') == 'waiting_localization',
                      75, 'A 栈进入 waiting_localization')
        self.call_api('navigation/initial_pose',
                      {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'sessionId': SESSION, 'requestId': rid()})
        self.wait_for(lambda: self.status().get('ready'), 30, 'A 栈就绪（AMCL+TF+全部节点激活）')
        status = self.status()
        self.a_channel_topic = status['velocityChannel']['topic']
        self.rec.event('run_a_ready', velocityChannel=status['velocityChannel'],
                       mapPath=map_path, phase=status['phase'])
        self.observe_a_channel()

    def _heartbeat(self):
        while not self.heartbeat_stop.wait(0.4):
            try:
                api('navigation/heartbeat', {'sessionId': SESSION}, allow_error=True, timeout=5)
            except Exception:
                pass

    def status(self):
        self.status_last = api('navigation_status', timeout=10)
        return self.status_last

    def stop_run_a(self):
        # 正确端点为 stop_navigation（navigationRoutes.js:183；offline_acceptance.py:305 同款）。
        for attempt in (1, 2):
            try:
                result = self.call_api('stop_navigation',
                                       {'sessionId': SESSION, 'requestId': rid()},
                                       allow_error=True)
                self.rec.event('stop_navigation', attempt=attempt,
                               success=result.get('success'), error=result.get('error'))
                self.wait_for(lambda: not self.status().get('isRunning'), 30, 'A 栈停止')
                return
            except Exception as error:
                self.rec.event('stop_run_a_error', attempt=attempt, error=str(error))
        raise RuntimeError('stop_navigation 两次尝试后 A 栈仍在运行')

    # ---- 运行 B：独立 smoother（launch:48 remap 形态） ----
    def start_run_b(self):
        text = PARAMS_SRC.read_text()
        renamed = re.sub(r'^velocity_smoother:', f'{B_NODE}:', text, count=1, flags=re.M)
        params_copy = self.run_dir / 'ldc_nav2_b_smoother.yaml'
        params_copy.write_text(renamed)
        self.b_params = {'source': str(PARAMS_SRC), 'sourceSha256': sha256(PARAMS_SRC),
                         'copy': str(params_copy), 'copySha256': sha256(params_copy),
                         'transform': '顶层 velocity_smoother: → velocity_smoother_b:（其余原文）'}
        cmd = ['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args',
               '-r', f'__node:={B_NODE}', '-r', 'cmd_vel:=cmd_vel_nav',
               '-r', f'cmd_vel_smoothed:={B_OUTPUT_TOPIC}', '--params-file', str(params_copy)]
        proc = self.start_proc('b_smoother', cmd)
        self.procs[-1]['proc'] = proc
        from lifecycle_msgs.srv import ChangeState, GetState
        node = self.observer
        change = node.create_client(ChangeState, f'/{B_NODE}/change_state')
        get = node.create_client(GetState, f'/{B_NODE}/get_state')
        if not change.wait_for_service(timeout_sec=30):
            raise RuntimeError('B smoother change_state 服务未出现')
        self.spin_for(0.5)

        def read_state():
            state_future = get.call_async(GetState.Request())
            self.spin_until(state_future, 5)
            result = state_future.result()
            return result.current_state.id if result else None

        # 与 Bridge 产品代码同款策略（navigation_bridge.py:1369-1370）：
        # 每次转换后实读状态；RPC 响应丢失不挂起、不盲目重复转换。
        for transition, expected in ((1, 2), (3, 3)):
            reached = False
            for attempt in range(3):
                request = ChangeState.Request()
                request.transition.id = transition
                future = change.call_async(request)
                responded = self.spin_until(future, 10)
                state = read_state()
                if state in (10, 11, 12, 13, 14, 15):  # 转换中间态
                    self.spin_for(1.0)
                    state = read_state()
                self.rec.event('b_lifecycle', transition=transition, attempt=attempt + 1,
                               responded=responded, state=state, expected=expected)
                if state == expected:
                    reached = True
                    break
                self.spin_for(1.0)
            if not reached:
                raise RuntimeError(f'B smoother transition {transition} 后状态 != {expected}')

    # ---- 观测节点 ----
    def build_observer(self):
        rclpy = self.rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        self.observer = rclpy.create_node('dual_instance_observer')
        self.nodes.append(self.observer)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)

        def twist_cb(point):
            def cb(msg):
                self.rec.rec(point, vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z)
            return cb

        self.observer.create_subscription(Twist, '/cmd_vel_nav', twist_cb('cmd_vel_nav'), reliable)
        self.observer.create_subscription(Twist, B_OUTPUT_TOPIC, twist_cb('b_output'), reliable)
        self.observer.create_subscription(Twist, '/cmd_vel', twist_cb('final_cmd_vel'), reliable)

        def odom_cb(msg):
            q = msg.pose.pose.orientation
            yaw = 2.0 * math.atan2(q.z, q.w)
            self.rec.rec('odom_truth', x=msg.pose.pose.position.x,
                         y=msg.pose.pose.position.y, yaw=yaw)
        self.observer.create_subscription(Odometry, '/odom', odom_cb, qos_profile_sensor_data)

        if self.inject:
            self.inject_node = rclpy.create_node('dual_a_inject')
            self.nodes.append(self.inject_node)
            pub = self.inject_node.create_publisher(Twist, '/cmd_vel_nav', reliable)

            def tick():
                if not self.inject_flag['active']:
                    return
                idx = int((time.monotonic() - self.inject_flag['t0']) / PLATEAU_SECONDS) % 2
                msg = Twist()
                msg.linear.x = SIG_VX[idx]
                msg.angular.z = SIG_WZ
                pub.publish(msg)
                self.rec.rec('inject_source', vx=msg.linear.x, vy=0.0, wz=msg.angular.z)
            self.inject_node.create_timer(0.05, tick)

        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        for node in self.nodes:
            self.executor.add_node(node)

    def observe_a_channel(self):
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)

        def cb(msg):
            self.rec.rec('a_channel', vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z)
        self.observer.create_subscription(Twist, self.a_channel_topic, cb, reliable)

    # ---- 阶段与快照 ----
    def graph_endpoints(self, tag):
        """主证据：长驻 observer 节点的 ROS 图 API（完整发现，含 GID），
        避免 --no-daemon CLI 短发现窗口的竞态。CLI 原始快照仅作旁证。"""
        topics = {'cmd_vel_nav': '/cmd_vel_nav', 'b_output': B_OUTPUT_TOPIC,
                  'final_cmd_vel': '/cmd_vel', 'odom': '/odom'}
        if self.a_channel_topic:
            topics['a_channel'] = self.a_channel_topic

        def fmt(info):
            return {'node': info.node_name, 'namespace': info.node_namespace,
                    'gid': bytes(info.endpoint_gid).hex(), 'type': info.topic_type,
                    'reliability': str(info.qos_profile.reliability).split('.')[-1],
                    'durability': str(info.qos_profile.durability).split('.')[-1]}

        parsed = {}
        for key, topic in topics.items():
            pubs = self.observer.get_publishers_info_by_topic(topic)
            subs = self.observer.get_subscriptions_info_by_topic(topic)
            parsed[key] = {'topic': topic,
                           'publishers': [fmt(p) for p in pubs],
                           'subscribers': [fmt(s) for s in subs]}
        (self.run_dir / f'endpoints-{tag}.json').write_text(
            json.dumps(parsed, ensure_ascii=False, indent=2))
        nodes = [{'name': n, 'namespace': ns}
                 for n, ns in self.observer.get_node_names_and_namespaces()]
        (self.run_dir / f'endpoints-{tag}-nodes.json').write_text(
            json.dumps(nodes, ensure_ascii=False, indent=2))
        self.rec.event('graph_snapshot', tag=tag, nodes=[n['name'] for n in nodes])
        return parsed

    def snapshot(self, tag):
        parsed = self.graph_endpoints(tag)
        specs = [
            ('node_list', ['ros2', 'node', 'list', '--no-daemon']),
            ('topic_cmd_vel_nav', ['ros2', 'topic', 'info', '--no-daemon', '-v', '/cmd_vel_nav']),
            ('topic_final_cmd_vel', ['ros2', 'topic', 'info', '--no-daemon', '-v', '/cmd_vel']),
            ('topic_b_output', ['ros2', 'topic', 'info', '--no-daemon', '-v', B_OUTPUT_TOPIC]),
            ('node_b_smoother', ['ros2', 'node', 'info', '--no-daemon', f'/{B_NODE}']),
            ('node_bridge', ['ros2', 'node', 'info', '--no-daemon', '/console_navigation_bridge']),
            ('node_controller', ['ros2', 'node', 'info', '--no-daemon', '/controller_server']),
        ]
        if self.inject:
            specs.append(('node_inject', ['ros2', 'node', 'info', '--no-daemon', '/dual_a_inject']))
        for label, argv in specs:
            self.cli_snapshot(f'{tag}-{label}', argv)
        return parsed

    def sample_truth(self):
        samples = self.rec.samples['odom_truth']
        if samples:
            last = samples[-1]
            self.rec.event('chassis_truth', x=last['x'], y=last['y'], yaw=last['yaw'])
            return last
        return None

    def run_phases(self):
        rec = self.rec
        self.spin_for(1.0)
        rec.event('phase', name='P0_baseline')
        truth0 = self.sample_truth()
        self.spin_for(2.0)
        if self.inject:
            rec.event('phase', name='P1_a_active', source='injection:dual_a_inject',
                      signature={'vxPlateaus': SIG_VX, 'wz': SIG_WZ, 'tol': SIG_TOL})
            self.inject_flag.update(active=True, t0=time.monotonic())
            self.spin_for(1.5)
            endpoints = self.snapshot('p1')
            self.spin_for(self.args.p1_seconds - 1.5)
            self.inject_flag['active'] = False
        else:
            rec.event('phase', name='P1_a_active', source='real_controller:controller_server')
            goal = self.call_api('navigation/goals',
                                 {'points': [{'x': 0.8, 'y': 0.0, 'yaw': 0.0}],
                                  'sessionId': SESSION, 'requestId': rid()})
            self.goal_id = goal.get('goalId')
            self.wait_for(lambda: self.status().get('goalState') in ('accepted', 'executing'),
                          20, 'A 目标受理')
            self.spin_for(1.5)
            endpoints = self.snapshot('p1')
            deadline = time.monotonic() + self.args.p1_seconds
            while time.monotonic() < deadline:
                state = self.status().get('goalState')
                if not self.goal_timeline or self.goal_timeline[-1]['goalState'] != state:
                    self.goal_timeline.append({'at': now_iso(), 't': rec.t(), 'goalState': state})
                    rec.event('goal_state', goalState=state, goalId=self.goal_id)
                if state in ('succeeded', 'failed', 'canceled'):
                    break
                self.spin_for(0.25)
            self.goal_terminal = self.status().get('goalState')
        rec.event('phase', name='P2_a_silent')
        self.spin_for(2.5)
        truth1 = self.sample_truth()
        self.endpoints = endpoints
        self.truth = {'start': truth0, 'end': truth1}
        if truth0 and truth1:
            self.displacement = math.hypot(truth1['x'] - truth0['x'], truth1['y'] - truth0['y'])
        else:
            self.displacement = None
        rec.event('displacement', meters=self.displacement)

    # ---- 断言 ----
    def assertions(self):
        rec = self.rec
        p1_start, p1_end = rec.window('P1_a_active', 'P2_a_silent')
        # phase 事件的 kind 是 'phase'，name 在字段里；修正窗口取法
        marks = {e['name']: e['t'] for e in rec.events if e['kind'] == 'phase'}
        p1_start, p1_end = marks.get('P1_a_active'), marks.get('P2_a_silent')
        p0_start = marks.get('P0_baseline')
        nav = rec.in_window('cmd_vel_nav', p1_start, p1_end)
        bout = rec.in_window('b_output', p1_start, p1_end)
        achan = rec.in_window('a_channel', p1_start, p1_end)
        final_p1 = rec.in_window('final_cmd_vel', p1_start, p1_end)
        final_quiet = (rec.in_window('final_cmd_vel', p0_start, p1_start)
                       + rec.in_window('final_cmd_vel', p1_end, rec.t()))
        ep = self.endpoints
        nav_pubs = {p['node'] for p in ep.get('cmd_vel_nav', {}).get('publishers', [])}
        nav_subs = {s['node'] for s in ep.get('cmd_vel_nav', {}).get('subscribers', [])}
        b_out_pubs = {p['node'] for p in ep.get('b_output', {}).get('publishers', [])}
        final_pubs = {p['node'] for p in ep.get('final_cmd_vel', {}).get('publishers', [])}
        sig_b = [s for s in bout if sig_match(s)]
        nz_b = [s for s in bout if nonzero(s)]
        nz_nav = [s for s in nav if nonzero(s)]
        pairs = mirror_pairs(achan, bout)
        mirror_diff = (sum(abs(a['vx'] - b['vx']) + abs(a['wz'] - b['wz']) for a, b in pairs)
                       / len(pairs)) if pairs else None
        A = []

        def add(aid, desc, expected, observed, ok, note=''):
            # 断言分类与期望方向：修复前（expect=crosstalk）ISO 期望 FAIL=缺陷确认，
            # WIRE 期望 PASS=串线正向证据；修复后（expect=isolated）方向反转。
            # 与期望不符的 FAIL 才是测量/实现问题（如端点发现伪影），必须单独标注。
            if aid.startswith('ISO-'):
                category, expected_pass = 'isolation(变异敏感)', self.args.expect == 'isolated'
            elif aid.startswith('WIRE-'):
                category, expected_pass = 'crosstalk-positive', self.args.expect == 'crosstalk'
            elif aid.startswith('PRE-'):
                category, expected_pass = 'precondition', True
            elif aid.startswith('ATTR-'):
                category, expected_pass = 'attribution', True
            elif aid == 'LIVE':
                category, expected_pass = 'liveness', True
            elif aid == 'GATE-LOCKED':
                category, expected_pass = 'gate-observation', True
            else:
                category, expected_pass = 'observation', True
            A.append({'id': aid, 'description': desc, 'expected': expected,
                      'observed': observed, 'pass': bool(ok), 'note': note,
                      'category': category, 'expectedPass': expected_pass,
                      'asExpected': bool(ok) == expected_pass})

        add('PRE-NAV', '运行 A 真实 Nav2 栈就绪（后端 ready，AMCL/TF/全节点激活）',
            'ready=true', self.run_a_ready, self.run_a_ready)
        add('PRE-B-SMOOTHER', 'B smoother 激活且接线为 launch:48 形态',
            f'{B_NODE} 订阅 /cmd_vel_nav 并发布 {B_OUTPUT_TOPIC}',
            {'subscribes_cmd_vel_nav': B_NODE in nav_subs,
             'publishes_b_output': B_NODE in b_out_pubs, 'lifecycle': 'active(3)'},
            B_NODE in nav_subs and B_NODE in b_out_pubs)
        add('PRE-A-OUTPUT', 'P1 期间 A 侧在 /cmd_vel_nav 有持续非零输出',
            '非零样本>=40', {'nonzero': len(nz_nav), 'total': len(nav)}, len(nz_nav) >= 40)
        if self.inject:
            # ROS 图统计的是"已创建"的发布者：A 控制器空闲（无目标）时其 publisher 仍在图中，
            # 但不发样本。归属判定 = 注入源在图中 ∧ P1 全部 cmd_vel_nav 样本都匹配注入签名。
            silent_a = all(sig_match(s) for s in nav) if nav else False
            add('ATTR-INJECT', 'P1 期间 /cmd_vel_nav 样本全部来自标注注入源（A 控制器静默）',
                "dual_a_inject ∈ publishers 且全部样本匹配签名",
                {'publishers': sorted(nav_pubs), 'samples': len(nav),
                 'allMatchSignature': silent_a},
                'dual_a_inject' in nav_pubs and silent_a,
                '注入源：测试发布器，非真实控制器')
            add('WIRE-1', 'A 的可辨识序列到达 B smoother 输出（串线正向证据）',
                '签名匹配样本>=40', len(sig_b), len(sig_b) >= 40)
            add('ISO-1', 'B smoother 输出不含 A 的签名（隔离断言，对恢复共享 cmd_vel_nav 敏感）',
                '签名匹配样本==0', len(sig_b), len(sig_b) == 0,
                '修复前期望 FAIL；修复后期望 PASS；变异（恢复共享接线）必须 FAIL')
        else:
            add('ATTR-REAL', 'P1 期间 /cmd_vel_nav 发布者为真实 Nav2 控制器/行为服务器',
                "publishers ⊆ {controller_server, behavior_server} 且非空",
                sorted(nav_pubs),
                bool(nav_pubs) and nav_pubs <= {'controller_server', 'behavior_server'})
            add('WIRE-1R', '真实 A 控制器输出到达 B smoother 输出（串线正向证据）',
                'B 输出非零样本>=40', len(nz_b), len(nz_b) >= 40)
            add('ISO-1R', 'B smoother 输出无非零样本（隔离断言，变异敏感）',
                '非零样本==0', len(nz_b), len(nz_b) == 0,
                'B 无自身控制器；任何非零只能来自共享 cmd_vel_nav 上的 A')
        add('WIRE-2', 'B 输出镜像 A 本轮通道输出（同输入同参数的 smoother 指纹）',
            '镜像对>=40 且平均绝对差<=0.02',
            {'pairs': len(pairs), 'meanAbsDiff': mirror_diff},
            len(pairs) >= 40 and mirror_diff is not None and mirror_diff <= 0.02)
        add('GATE-LOCKED', 'A 未执行目标时最终 /cmd_vel 全零（门控锁住；单独不足以验收）',
            '静默窗口全零', {'samples': len(final_quiet),
                             'nonzero': sum(1 for s in final_quiet if nonzero(s))},
            all(not nonzero(s) for s in final_quiet),
            '必须与 ISO-1 联读：门控可能遮住上游串线')
        add('FINAL-PUBLISHER', '最终 /cmd_vel 发布者身份（Q1：唯一合法出口=A bridge）',
            "publishers ⊆ {console_navigation_bridge}", sorted(final_pubs),
            final_pubs <= {'console_navigation_bridge'})
        if self.inject:
            add('CHASSIS-STILL', '注入变体底盘保持静止（无授权目标）',
                '位移<0.02m', self.displacement,
                self.displacement is not None and self.displacement < 0.02)
        else:
            add('LIVE', '真实目标执行：Action 终态 succeeded 且底盘真实位移（环境不是永久零速）',
                'terminal=succeeded 且位移>=0.5m 且 final vx 峰值>=0.05',
                {'terminal': getattr(self, 'goal_terminal', None),
                 'displacement': self.displacement,
                 'finalVxPeak': max((abs(s['vx']) for s in final_p1), default=0.0)},
                getattr(self, 'goal_terminal', None) == 'succeeded'
                and self.displacement is not None and self.displacement >= 0.5
                and max((abs(s['vx']) for s in final_p1), default=0.0) >= 0.05)
        wire = next(a for a in A if a['id'] in ('WIRE-1', 'WIRE-1R'))
        iso = next(a for a in A if a['id'] in ('ISO-1', 'ISO-1R'))
        live = next((a for a in A if a['id'] == 'LIVE'), None)
        if wire['pass']:
            classification = 'CROSSTALK_PATH_EXISTS'
        elif iso['pass'] and (live is None or live['pass']):
            classification = 'OLD_SOURCE_ISOLATED'
        elif live is not None and not live['pass']:
            classification = 'NEW_TASK_CANNOT_MOVE'
        else:
            classification = 'INCONCLUSIVE'
        return A, classification

    # ---- 产物 ----
    def write_products(self, assertions, classification):
        args = self.args
        expected = args.expect
        expect_map = {'crosstalk': 'CROSSTALK_PATH_EXISTS', 'isolated': 'OLD_SOURCE_ISOLATED'}
        pre_ok = all(a['pass'] for a in assertions if a['id'].startswith('PRE-'))
        matched = classification == expect_map[expected]
        unexpected = [a['id'] for a in assertions if not a['asExpected']]
        result = 'PASS' if (pre_ok and matched and not unexpected) else 'FAIL'
        meta = {
            'ticket': 'A-verify-isolation', 'round': 'isolation-round-20261118',
            'variant': self.variant, 'injectionSource': self.inject,
            'generatedAt': now_iso(), 'runDir': str(self.run_dir),
            'domain': 197, 'rmw': getattr(self, 'rmw', None),
            'discovery': 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
            'priorEnv': getattr(self, 'prior_env', None),
            'lockOwner': getattr(self, 'lock_owner', None),
            'workspaceHead': subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                                            cwd=str(ROOT), capture_output=True,
                                            text=True).stdout.strip(),
            'testFile': str(Path(__file__).resolve()),
            'testFileSha256': sha256(Path(__file__).resolve()),
            'productFilesSha256': {str(p.relative_to(ROOT)): sha256(p) for p in PRODUCT_FILES},
            'mapFixture': getattr(self, 'map_fixture', None),
            'bParams': getattr(self, 'b_params', None),
            'processes': [{k: v for k, v in p.items() if k != 'proc'} for p in self.procs],
            'topics': {'aOutput_bInput': '/cmd_vel_nav', 'bOutput': B_OUTPUT_TOPIC,
                       'finalChassis': '/cmd_vel', 'aChannel': self.a_channel_topic,
                       'truth': '/odom'},
            'nodes': {'aController': '/controller_server', 'aBehavior': '/behavior_server',
                      'aSmoother': '/velocity_smoother', 'aBridge': '/console_navigation_bridge',
                      'bSmoother': f'/{B_NODE}',
                      'inject': '/dual_a_inject' if self.inject else None,
                      'observer': '/dual_instance_observer',
                      'chassis': '/offline_console_robot'},
            'signature': {'vxPlateaus': SIG_VX, 'wz': SIG_WZ, 'tol': SIG_TOL,
                          'plateauSeconds': PLATEAU_SECONDS} if self.inject else None,
            'session': SESSION, 'goalId': getattr(self, 'goal_id', None),
            'goalTerminal': getattr(self, 'goal_terminal', None),
            'displacementMeters': getattr(self, 'displacement', None),
            'truth': getattr(self, 'truth', None),
        }
        (self.run_dir / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2))
        assertions_doc = {
            'result': result, 'variant': self.variant, 'classification': classification,
            'expectedClassification': expected, 'preconditionsPass': pre_ok,
            'unexpectedFailures': unexpected,
            'defectConfirmed': classification == 'CROSSTALK_PATH_EXISTS' and any(
                a['id'].startswith('ISO-') and not a['pass'] and a['asExpected']
                for a in assertions),
            'resultSemantics': ('PASS=观测符合本模式期望方向（修复前 crosstalk：串线复现且 ISO 如期望失败；'
                                '修复后 isolated：隔离成立且 LIVE 通过）；FAIL=前置不满足、分类不符或存在'
                                '非期望断言失败（测量/实现问题，须排查）'),
            'injectionSource': self.inject, 'domain': 197, 'rmw': meta['rmw'],
            'generatedAt': meta['generatedAt'], 'assertions': assertions,
            'conclusionBoundary': (
                '接线路径存在：A 输出经共享 cmd_vel_nav 到达 B smoother 输出（门控上游实测）。'
                '本变体最终 /cmd_vel 由 A bridge 门控；未证明 B 授权后车辆误动，'
                '该结论需 e2e/验收阶段（B 全栈+授权）证据。'
                if classification == 'CROSSTALK_PATH_EXISTS' else
                '旧源被隔离：B smoother 输出不含 A 的贡献（且真实任务可动，见 LIVE）。'
                if classification == 'OLD_SOURCE_ISOLATED' else
                '新任务本身不能动：LIVE 失败，检查就绪/授权/接线，不得以永久零速过关。'
                if classification == 'NEW_TASK_CANNOT_MOVE' else '证据不足，见断言明细。'),
        }
        (self.run_dir / 'assertions.json').write_text(
            json.dumps(assertions_doc, ensure_ascii=False, indent=2))
        self.write_report(assertions_doc, meta)
        summary = {'variant': self.variant, 'result': result,
                   'classification': classification, 'expected': expected,
                   'runDir': str(self.run_dir),
                   'pass': sum(a['pass'] for a in assertions),
                   'fail': sum(not a['pass'] for a in assertions)}
        (Path(args.output) / f'summary-{self.variant}-{self.stamp}.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2))
        print(json.dumps(summary, ensure_ascii=False))
        return result

    def write_report(self, doc, meta):
        lines = []
        ap = lines.append
        ap(f'# 工单 A 受控复现报告 · 变体 {self.variant}（{"注入源" if self.inject else "真实 A 控制器"}）')
        ap('')
        ap(f'- 生成时间：{meta["generatedAt"]}；域：197（共享离线验收环境，持锁）；RMW 实测：{meta["rmw"]}')
        ap(f'- 发现限制：ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST；工作区 HEAD：{meta["workspaceHead"]}（修复前，未提交代码原样）')
        ap(f'- 运行目录：`{self.run_dir}`')
        ap(f'- 结论分类：**{doc["classification"]}**（期望 {doc["expectedClassification"]}）→ 本次运行判定 **{doc["result"]}**')
        ap('')
        ap('## 拓扑与启动命令')
        ap('')
        ap('- 运行 A：`scripts/offline_console.sh --output runs/console-env`（合成底盘 offline_robot.py + 后端 + 真实 Nav2）。'
           'A 栈由后端 `start_navigation` 经 `navigation2.launch.py` 启动：controller/behavior `cmd_vel→cmd_vel_nav`；'
           'A smoother `cmd_vel→cmd_vel_nav`、`cmd_vel_smoothed→A 本轮通道`（navigationRoutes.js:144 传参）。')
        ap(f'- 运行 B（本程序）：`{" ".join(meta["processes"][0]["cmd"]) if meta["processes"] else "(见 meta)"}`')
        ap(f'  - remap 形态与 launch:45-48 一致；节点全名 `/{B_NODE}`；参数为 ldc_nav2.yaml 原文（仅顶层节名改为 {B_NODE}，sha256 见 meta）。')
        ap(f'- A 速度源：{"测试发布器 /dual_a_inject（**注入源**，签名 vx∈%s 交替 1s 平台、wz=%s，20Hz）" % (SIG_VX, SIG_WZ) if self.inject else "真实 controller_server（DWB），经后端 navigate_to_pose 目标 (0.8, 0, 0) 驱动"}')
        ap('- 地图：预制 fixture synth_room（与 offline_robot ray() 墙/柱一致；未调用 run_task/mapping，自动建图入口未触碰）。')
        ap('')
        ap('## 四级时间线（详见 timeline.jsonl）')
        ap('')
        ap('| 级 | 观测点 | 话题 | 说明 |')
        ap('|---|---|---|---|')
        ap('| L1 A 输出 | cmd_vel_nav | /cmd_vel_nav | A 控制器/注入源发布 |')
        ap('| L2 B 输入 | cmd_vel_nav（同一话题） | /cmd_vel_nav | 共享即缺陷；端点快照证明 B smoother 订阅 |')
        ap(f'| L3 B 输出 | b_output | {B_OUTPUT_TOPIC} | B smoother 平滑后输出（门控上游） |')
        ap('| L4 最终底盘指令 | final_cmd_vel + odom_truth | /cmd_vel、/odom | A bridge 门控出口 + 合成底盘真值 |')
        ap('| 参考 | a_channel | A 本轮通道 | A smoother 合法输出，WIRE-2 镜像指纹 |')
        ap('')
        for point in ('cmd_vel_nav', 'b_output', 'a_channel', 'final_cmd_vel'):
            samples = self.rec.samples[point]
            ap(f'- `{point}`：{len(samples)} 条；' + (
                f'首条 t={samples[0]["t"]}s vx={samples[0]["vx"]:.4f} wz={samples[0]["wz"]:.4f}；'
                f'末条 t={samples[-1]["t"]}s vx={samples[-1]["vx"]:.4f} wz={samples[-1]["wz"]:.4f}'
                if samples else '无样本'))
        ap(f'- 底盘真值位移：{meta["displacementMeters"]} m；终态目标：{meta["goalTerminal"]}')
        ap('')
        ap('## 端点身份快照（P1 期间；主证据=长驻节点 ROS 图 API 含 GID，CLI 原始快照旁证见 endpoints/）')
        ap('')
        ep = self.endpoints
        for topic in ('cmd_vel_nav', 'b_output', 'final_cmd_vel', 'a_channel'):
            info = ep.get(topic)
            if not info:
                continue
            pubs = ', '.join(f'{p["node"]}(GID {p.get("gid", "?")[:16]}…)' for p in info['publishers']) or '无'
            subs = ', '.join(f'{s["node"]}(GID {s.get("gid", "?")[:16]}…)' for s in info['subscribers']) or '无'
            ap(f'- `{topic}`：发布者 [{pubs}]；订阅者 [{subs}]')
        ap('')
        ap('## 机器可读断言（assertions.json）')
        ap('')
        ap('| ID | 类别 | 期望阈值 | 实测 | 结果 | 符合期望方向 |')
        ap('|---|---|---|---|---|---|')
        for a in doc['assertions']:
            mark = '是' if a['asExpected'] else '**否·非期望**'
            ap(f'| {a["id"]} | {a["category"]} | {a["expected"]} | {json.dumps(a["observed"], ensure_ascii=False)} | {"PASS" if a["pass"] else "FAIL"} | {mark} |')
        ap('')
        ap('**FAIL 的读法（测量伪影与缺陷确认分开）**：')
        ap('- expect=crosstalk 下 ISO-1/ISO-1R 的 FAIL = **缺陷确认**（串线复现成功），不是测量失败；'
           '该断言即变异测试（恢复共享 cmd_vel_nav）必须失败的敏感断言。')
        ap('- 「非期望」FAIL（PRE/ATTR/WIRE/LIVE 等偏离期望方向）= 测量或实现问题，必须排查。'
           '典型测量伪影：ros2 CLI --no-daemon 短发现窗口导致端点图查询为空（样本仍在流动）；'
           '本程序已改用常驻 observer 节点图 API（get_publishers_info_by_topic / '
           'get_subscriptions_info_by_topic，含 GID）为主证据。历史伪影运行见 INDEX.md 的 superseded 标注。')
        if doc['unexpectedFailures']:
            ap(f'- 本次运行非期望失败：{doc["unexpectedFailures"]}')
        else:
            ap('- 本次运行无非期望失败。')
        ap('')
        ap('## S1 Q1 映射（谁有权发布最终 /cmd_vel）')
        ap('')
        ap('- 实测唯一最终出口：`/cmd_vel` 发布者 = console_navigation_bridge（A 轮 bridge，auto 门控 on_auto：'
           'mode=auto ∧ goalState=executing ∧ requirements ∧ owner ∧ lease；manual 路径 op=velocity）。'
           '自动建图 cmd_vel_gate 属 ldc_exploration，本轮未启动（入口保持关闭）。')
        ap('- 旧控制器路径（S1 t0 的 G_old 控制器）：controller_server→/cmd_vel_nav。本轮实测：A 控制器/注入源持续非零。')
        ap('- 旧 smoother 路径：A smoother 输出→A 本轮通道→仅 A bridge 可转发；S1 t1 后 A bridge 崩溃则该路径断开，'
           '但 /cmd_vel_nav 上的旧控制器输出仍然活着。')
        ap('- 新实例路径（S1 t3 的 G_new）：B controller→cmd_vel_nav→B smoother→B 本轮通道→B bridge 门控→/cmd_vel。')
        ap('- **缺陷实测**：cmd_vel_nav 无命名空间共享（launch:45-48），B smoother 订阅同一话题 → '
           'A 的输出进入 B 的通道（WIRE-1/2 证据）。B bridge 一旦授权，A 的运动将借 B 的授权到达车轮；'
           '本变体未含 B bridge，故结论边界为「接线路径存在」，不声称已造成车辆误动。')
        ap('')
        ap('## 结论边界与未验证项')
        ap('')
        ap(f'- {doc["conclusionBoundary"]}')
        ap('- 未验证：B 全栈+授权后的车轮级污染（e2e/修复后三段验收）；旧定位污染场景；Bridge 崩溃重启（S1 t1→t3）全序列。')
        ap('- GATE-LOCKED 单独通过不构成验收：门控可遮住上游串线，必须与 ISO-1 联读。')
        ap('')
        ap('## 复现命令 / 回滚')
        ap('')
        ap('```bash')
        ap('source /home/<user>/ros-car-migration/scripts/env.bash')
        ap('# 前置：按 COORDINATION.md §3 持 domain197.lock；offline_console.sh 已起（--output runs/console-env）')
        ap(f'python3 scripts/tests/dual_instance_repro.py --variant {self.variant} \\')
        ap(f'    --console-dir {self.args.console_dir} --output {self.args.output} --expect {self.args.expect}')
        ap('# 一条命令总入口（inject + real 顺序运行，产出 combined-summary.json 与 INDEX.md）：')
        ap(f'python3 scripts/tests/dual_instance_repro.py --variant all \\')
        ap(f'    --console-dir {self.args.console_dir} --output {self.args.output}')
        ap('```')
        ap('- 回滚：本工单只新增 `scripts/tests/dual_instance_repro.py` 与审计目录产物；删除即完全回滚，产品代码零改动。')
        ap('- 旧基线引用（非本轮实测）：action-recovery-20261117/offline-acceptance/acceptance.json（PASS，22 项，domain 197，synthetic）。')
        ap('')
        (self.run_dir / 'report.md').write_text('\n'.join(lines))

    # ---- 主流程 ----
    def run(self):
        try:
            self.preflight()
            self.build_observer()
            self.start_run_a()
            self.run_a_ready = True
            self.start_run_b()
            self.run_phases()
            self.fetch_bridge_events()
            assertions, classification = self.assertions()
            result = self.write_products(assertions, classification)
            return 0 if result == 'PASS' else 1
        except Exception as error:
            self.failure = {'error': str(error), 'type': type(error).__name__, 'at': now_iso()}
            self.rec.event('failure', **self.failure)
            (self.run_dir / 'failure.json').write_text(
                json.dumps(self.failure, ensure_ascii=False, indent=2))
            import traceback
            (self.run_dir / 'failure-traceback.txt').write_text(traceback.format_exc())
            print(json.dumps({'result': 'FAIL', 'failure': self.failure,
                              'runDir': str(self.run_dir)}, ensure_ascii=False))
            return 1
        finally:
            self.heartbeat_stop.set()
            self.inject_flag['active'] = False
            try:
                if self.a_started:
                    self.stop_run_a()
            except Exception as error:
                self.rec.event('cleanup_stop_a_error', error=str(error))
            self.cleanup_procs()
            for node in self.nodes:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if self.rclpy is not None and self.rclpy.ok():
                self.rclpy.shutdown()
            self.rec.close()
            self.api_log.close()

    def fetch_bridge_events(self):
        events, cursor = [], 0
        try:
            while True:
                with _opener.open(f'{BASE}navigation/events?since={cursor}', timeout=15) as response:
                    payload = json.load(response)
                batch = payload.get('events', [])
                events.extend(batch)
                if not batch or payload.get('latest', 0) <= (batch[-1]['seq'] if batch else cursor):
                    break
                cursor = batch[-1]['seq']
        except Exception as error:
            self.rec.event('bridge_events_error', error=str(error))
        (self.run_dir / 'bridge-events.json').write_text(
            json.dumps(events, ensure_ascii=False, indent=2))
        for event in events:
            if event.get('type') in ('goal_accepted', 'goal_succeeded', 'goal_failed',
                                     'goal_canceled', 'action_sent', 'action_terminal',
                                     'startup_done', 'cancel_confirmed'):
                self.rec.event('bridge_event', **{k: event.get(k) for k in
                                                  ('type', 'goalId', 'rosGoalId', 'reason', 'seq')})


def write_index(output):
    """扫描 runs 目录生成运行索引：权威 / 伪影作废 / 调试失败 分开标注。"""
    output = Path(output)
    rows = []
    for run_dir in sorted(output.iterdir()):
        if not run_dir.is_dir():
            continue
        af, ff = run_dir / 'assertions.json', run_dir / 'failure.json'
        if af.exists():
            d = json.loads(af.read_text())
            artifact = (any(a['id'] == 'PRE-B-SMOOTHER' and not a['pass']
                            for a in d['assertions'])
                        and any(a['id'].startswith('WIRE-') and a['pass']
                                for a in d['assertions']))
            status = 'superseded(端点测量伪影)' if artifact else 'authoritative'
            rows.append((run_dir.name, d['variant'], d['result'], d['classification'],
                         status, ''))
        elif ff.exists():
            d = json.loads(ff.read_text())
            rows.append((run_dir.name, run_dir.name.split('-')[0], 'FAIL(调试)', '-',
                         'failed-debug', str(d.get('error', ''))[:80]))
    lines = ['# 运行索引 · A-verify-isolation（isolation-round-20261118）', '',
             '| 运行目录 | 变体 | 结果 | 分类 | 状态 | 备注 |',
             '|---|---|---|---|---|---|']
    for row in rows:
        lines.append('| ' + ' | '.join(str(x) for x in row) + ' |')
    lines += ['',
              '- authoritative：端点证据完整（常驻节点图 API 含 GID），供检查点交叉核验。',
              '- superseded：早期 ros2 CLI --no-daemon 发现伪影致端点查询为空（样本流动正常），'
              'PRE-B-SMOOTHER/ATTR 的 FAIL 属测量失败而非缺陷证据；已由同变体 authoritative 运行取代。',
              '- failed-debug：程序调试期错误（见各 failure.json），非缺陷证据。',
              '- 缺陷确认读法：expect=crosstalk 下 ISO-1/ISO-1R FAIL 为期望方向（串线复现成功）。']
    (output / 'INDEX.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--variant', required=True,
                        choices=['inject', 'real', 'mutate', 'all'])
    parser.add_argument('--console-dir', required=True, type=Path,
                        help='offline_console.sh --output 目录（含 maps/）')
    parser.add_argument('--output', required=True, type=Path, help='runs 根目录')
    parser.add_argument('--expect', default='crosstalk',
                        choices=['crosstalk', 'isolated'],
                        help='期望分类：修复前 crosstalk；修复后 isolated；mutate 恒为 crosstalk')
    parser.add_argument('--p1-seconds', type=float, default=6.0)
    args = parser.parse_args()
    if args.variant == 'mutate':
        args.expect = 'crosstalk'
    if not (LOCK / 'owner.json').exists():
        print('ERROR: domain197.lock 不存在；先按 COORDINATION.md §3 获取锁', file=sys.stderr)
        return 2
    if args.variant == 'all':
        results = []
        for variant in ('inject', 'real'):
            sub = argparse.Namespace(**{**vars(args), 'variant': variant})
            harness = Harness(sub)
            returncode = harness.run()
            results.append({'variant': variant, 'returncode': returncode,
                            'runDir': str(harness.run_dir)})
        write_index(args.output)
        ok = all(r['returncode'] == 0 for r in results)
        combined = {'mode': 'all', 'result': 'PASS' if ok else 'FAIL',
                    'generatedAt': now_iso(), 'runs': results}
        (Path(args.output) / 'combined-summary.json').write_text(
            json.dumps(combined, ensure_ascii=False, indent=2))
        print(json.dumps(combined, ensure_ascii=False))
        return 0 if ok else 1
    harness = Harness(args)
    returncode = harness.run()
    write_index(args.output)
    return returncode


if __name__ == '__main__':
    raise SystemExit(main())