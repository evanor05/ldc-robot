#!/usr/bin/env python3
"""D 工单停车延迟基线编排器（首轮：只新增测量代码，不改任何产品文件）。

同场景同负载矩阵：先建立非零运动，再由外部测试端（本进程经 stdin 管道）发停止，
采集四时刻 + tick 间隔，分场景统计 p50/p95/p99/max/漏测。

场景（canonical，auto 运动：begin→initial_pose→goals→Nav2 替身发速度→stop）：
  normal           参数服务在线，无注入
  params_offline   参数服务离线（替身不创建 GetParameters 服务）
  lock_contention  停止窗口内另一进程持有账本 BEGIN IMMEDIATE（SQLite timeout=0）
  slow_disk        LD_PRELOAD fsync shim 延迟账本落盘（默认 150ms/次）
窗口探针（manual 运动，先建立非零再在指定窗口内发停止）：
  stop_during_params   参数读取窗口内发停止（params_offline 环境）
  stop_during_prepare  目标持久记录 fsync 窗口内发停止（slow_disk 环境）

四时刻（全部 time.monotonic()，同机跨进程可比）：
  t0  外部测试端写停止请求（本编排器）
  t1  Bridge command_reader 入队（latency_probe TimedQueue）
  t2  ROS 执行线程开始 process（latency_probe）
  t3  独立订阅者观察到停止后首条零速（latency_observer 进程）
中间量 t2c = 执行线程 zero() 发布时刻；分段：t1-t0 管道/读线程，t2-t1 排队等 tick，
t2c-t2 执行到发零速，t3-t2c DDS 交付+观察者调度，t3-t0 软件链路总延迟。

用法（先 source /opt/ros/lyrical/setup.bash）：
  python3 scripts/latency_run.py --out <审计目录>/baseline-<时间戳> [--samples 30]
  python3 scripts/latency_run.py --out ... --quick      # 冒烟：每场景 3 样本
  python3 scripts/latency_run.py --out ... --scenarios normal,slow_disk

本模块顶层不导入 rclpy，分析函数可被 scripts/tests/latency_test_analysis.py 单测。
"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / 'scripts'

SEGMENTS = ('pipe_read', 'queue_wait', 'exec_to_zero', 'dds_delivery', 'total')
SEGMENT_LABELS = {
    'pipe_read': 't1-t0 管道+读线程',
    'queue_wait': 't2-t1 排队等执行线程',
    'exec_to_zero': 't2c-t2 执行到发零速',
    'dds_delivery': 't3-t2c DDS交付+观察者',
    'total': 't3-t0 软件链路总延迟',
}

SCENARIOS = {
    'normal': {'params': 'online', 'fsync_ms': 0, 'contention': False,
               'motion': 'auto', 'trigger': None},
    'params_offline': {'params': 'offline', 'fsync_ms': 0, 'contention': False,
                       'motion': 'auto', 'trigger': None},
    'lock_contention': {'params': 'online', 'fsync_ms': 0, 'contention': True,
                        'motion': 'auto', 'trigger': None},
    'slow_disk': {'params': 'online', 'fsync_ms': 150, 'contention': False,
                  'motion': 'auto', 'trigger': None},
    # 重档(150ms)下单次派发阻塞>1.5s 会触发传感器过期保护取消，auto 运动无法建立；
    # 轻档(50ms)派发阻塞~0.4s 可建立运动，用于同负载停车延迟对照。
    'slow_disk_mild': {'params': 'online', 'fsync_ms': 50, 'contention': False,
                       'motion': 'auto', 'trigger': None},
    'stop_during_params': {'params': 'offline', 'fsync_ms': 0, 'contention': False,
                           'motion': 'manual', 'trigger': 'params_start'},
    'stop_during_prepare': {'params': 'online', 'fsync_ms': 150, 'contention': False,
                            'motion': 'manual', 'trigger': 'prepare_start'},
}
PROBE_SCENARIOS = ('stop_during_params', 'stop_during_prepare')


# ---------------- 分析函数（纯函数，可单测） ----------------

def percentile(values, q):
    """nearest-rank 百分位：sorted[ceil(q*n)-1]；空集返回 None。"""
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(q * len(ordered)) - 1)
    return ordered[min(index, len(ordered) - 1)]


def is_zero(msg):
    return all(msg.get(key, 0.0) == 0.0 for key in ('lx', 'ly', 'lz', 'ax', 'ay', 'az'))


def first_zero_after(messages, t0, start=0):
    """t0 之后首条全零 /cmd_vel 的到达时刻；没有则 None。start 为搜索起点下标。"""
    for msg in messages[start:]:
        if msg['mono'] >= t0 and is_zero(msg):
            return msg['mono']
    return None


def motion_evidence(messages, t0, window=1.0, min_nonzero=3, max_gap=0.6):
    """停止前是否已建立非零运动：[t0-window, t0) 内非零条数与最后非零时刻。

    消息按到达单调递增，从尾部反向扫描到窗口外即停，避免全量遍历。
    """
    recent = []
    for msg in reversed(messages):
        if msg['mono'] < t0 - window:
            break
        if msg['mono'] < t0 and not is_zero(msg):
            recent.append(msg)
    recent.reverse()
    last = recent[-1]['mono'] if recent else None
    ok = len(recent) >= min_nonzero and last is not None and (t0 - last) <= max_gap
    return {'nonzeroCount': len(recent), 'lastNonzero': last, 'established': ok}


def tick_stats(ticks):
    """ticks: [(start, end)] → 相邻 tick 间隔与单次耗时统计（秒）。"""
    starts = [t[0] for t in ticks]
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    durations = [end - start for start, end in ticks]
    def describe(values):
        return {'n': len(values),
                'p50': percentile(values, .50), 'p95': percentile(values, .95),
                'p99': percentile(values, .99), 'max': max(values) if values else None}
    return {'interval': describe(gaps), 'duration': describe(durations)}


def segment_stats(samples, key):
    values = [s['segments'][key] for s in samples
              if s.get('segments') and s['segments'].get(key) is not None]
    missed = sum(1 for s in samples if not (s.get('segments') and s['segments'].get(key) is not None))
    return {'n': len(values), 'missed': missed,
            'p50': percentile(values, .50), 'p95': percentile(values, .95),
            'p99': percentile(values, .99), 'max': max(values) if values else None,
            'raw': values if len(values) < 20 else None}


def ms(value):
    return None if value is None else round(value * 1000, 3)


# ---------------- 进程与管道管理 ----------------

class Tail:
    """增量读取 JSONL 文件（观察者/探针时间线）。"""

    def __init__(self, path):
        self.path = Path(path)
        self.offset = 0
        self.records = []

    def poll(self):
        if not self.path.exists():
            return []
        with open(self.path, 'r', encoding='utf-8') as handle:
            handle.seek(self.offset)
            chunk = handle.read()
            self.offset = handle.tell()
        fresh = []
        for line in chunk.splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                continue
            self.records.append(record)
            fresh.append(record)
        return fresh


HOLDER_SCRIPT = r'''
import sqlite3, sys, time
db = sqlite3.connect(sys.argv[1], timeout=5)
db.execute('BEGIN IMMEDIATE')
with open(sys.argv[2], 'w') as marker:
    marker.write(repr(time.monotonic()))
sys.stdin.readline()
db.rollback()
db.close()
'''


class Orchestrator:
    def __init__(self, out_dir, fsync_ms=150, cancel_delay=0.05):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.fsync_ms = fsync_ms
        self.cancel_delay = cancel_delay
        self.journal = self.out / 'journal' / 'actions.sqlite3'
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        self.shim = self.out / 'latency_fsync_shim.so'
        self.responses = {}
        self.response_cv = threading.Condition()
        self.id_counter = 0
        self.procs = []
        self.bridge = None
        self.stub = None
        self.holder = None
        self.sid = 'latency-session'
        self.heartbeat_stop = threading.Event()
        self.observer_tail = Tail(self.out / 'observer.jsonl')
        self.probe_tail = None
        self.stop_heartbeat = threading.Event()

    # ---- 环境 ----

    def base_env(self):
        env = dict(os.environ)
        env.update({
            'ROS_DOMAIN_ID': '194',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
            'ROBOT_SIMULATION': '1',
            'ROBOT_CONSOLE_LOG_DIR': str(self.out / 'bridge-logs'),
            'LDC_ACTION_JOURNAL_PATH': str(self.journal),
            'ROBOT_RUNS_DIR': str(self.out / 'runs'),
            'ROBOT_RUN_ACTIVE_MARKER': str(self.out / 'runs' / '.run-active'),
            'LATENCY_CANCEL_DELAY': str(self.cancel_delay),
        })
        env.pop('PYTEST_CURRENT_TEST', None)
        env.pop('LD_PRELOAD', None)
        return env

    def spawn(self, name, args, env=None, stdin_pipe=False, stdout_pipe=False):
        log = open(self.out / f'{name}.log', 'a', buffering=1, encoding='utf-8')
        proc = subprocess.Popen(
            [sys.executable, *args],
            stdin=subprocess.PIPE if stdin_pipe else subprocess.DEVNULL,
            stdout=subprocess.PIPE if stdout_pipe else log,
            stderr=log, env=env or self.base_env(), cwd=str(REPO), text=True)
        self.procs.append((name, proc, log))
        return proc

    def compile_shim(self):
        source = SCRIPTS / 'latency_fsync_shim.c'
        subprocess.run(['gcc', '-shared', '-fPIC', '-O2', '-o', str(self.shim),
                        str(source), '-ldl'], check=True, capture_output=True, text=True)

    # ---- Bridge 请求/响应 ----

    def next_id(self):
        self.id_counter += 1
        return f'lat-{self.id_counter}'

    def send(self, request, wait=None):
        request = dict(request)
        request.setdefault('id', self.next_id())
        request.setdefault('sessionId', self.sid)
        line = json.dumps(request, ensure_ascii=False) + '\n'
        at = time.monotonic()
        self.bridge.stdin.write(line)
        self.bridge.stdin.flush()
        if wait:
            return self.wait_response(request['id'], wait)
        return request['id'], at

    def wait_response(self, request_id, timeout):
        deadline = time.monotonic() + timeout
        with self.response_cv:
            while request_id not in self.responses:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.response_cv.wait(remaining)
            return self.responses[request_id]

    def read_responses(self):
        evidence = open(self.out / 'bridge-stdout.jsonl', 'a', buffering=1, encoding='utf-8')
        for line in self.bridge.stdout:
            at = time.monotonic()
            evidence.write(json.dumps({'mono': at, 'line': line.rstrip('\n')}) + '\n')
            try:
                payload = json.loads(line)
            except ValueError:
                continue
            if isinstance(payload, dict) and payload.get('id'):
                with self.response_cv:
                    self.responses[payload['id']] = (at, payload)
                    self.response_cv.notify_all()
        evidence.close()

    def status(self, timeout=3.0):
        result = self.send({'op': 'status'}, wait=timeout)
        return result[1] if result else None

    def heartbeat_loop(self):
        while not self.stop_heartbeat.is_set():
            try:
                if self.bridge and self.bridge.poll() is None:
                    self.send({'op': 'heartbeat'})
            except (OSError, ValueError):
                pass
            self.stop_heartbeat.wait(1.0)

    # ---- 启动/停止 ----

    def start_global(self):
        self.compile_shim()
        # offline_robot.main() 硬编码域197守卫；latency_robot 复用其 Robot 类于私有域194。
        self.spawn('latency_robot', [str(SCRIPTS / 'latency_robot.py')])
        self.spawn('latency_observer', [str(SCRIPTS / 'latency_observer.py'),
                                        '--out', str(self.out / 'observer.jsonl')])
        time.sleep(1.0)

    def start_scenario(self, name):
        config = SCENARIOS[name]
        scenario_dir = self.out / name
        scenario_dir.mkdir(parents=True, exist_ok=True)
        self.scenario_dir = scenario_dir
        self.probe_tail = Tail(scenario_dir / 'probe.jsonl')
        for marker in (scenario_dir / 'triggers').glob('*'):
            marker.unlink()
        stub_env = self.base_env()
        stub_env['LATENCY_PARAMS'] = config['params']
        self.stub = self.spawn(f'{name}-stub',
                               [str(SCRIPTS / 'latency_nav2_stub.py'),
                                '--log', str(scenario_dir / 'stub.jsonl')],
                               env=stub_env, stdin_pipe=True)
        bridge_env = self.base_env()
        if config['fsync_ms']:
            bridge_env['LD_PRELOAD'] = str(self.shim)
            bridge_env['LATENCY_FSYNC_DELAY_MS'] = str(config['fsync_ms'])
            bridge_env['LATENCY_FSYNC_LOG'] = str(scenario_dir / 'fsync.jsonl')
        self.bridge = self.spawn(f'{name}-bridge',
                                 [str(SCRIPTS / 'latency_probe.py'),
                                  '--probe-out', str(scenario_dir / 'probe.jsonl'),
                                  '--trigger-dir', str(scenario_dir / 'triggers'),
                                  '--journal-path', str(self.journal)],
                                 env=bridge_env, stdin_pipe=True, stdout_pipe=True)
        threading.Thread(target=self.read_responses, daemon=True).start()
        self.stop_heartbeat.clear()
        threading.Thread(target=self.heartbeat_loop, daemon=True).start()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if self.status(timeout=2.0):
                return True
            time.sleep(0.2)
        return False

    def stop_scenario(self):
        self.stop_heartbeat.set()
        self.release_holder()
        if self.bridge and self.bridge.poll() is None:
            try:
                self.send({'op': 'shutdown'})
                self.bridge.wait(timeout=9)
            except (subprocess.TimeoutExpired, OSError):
                self.bridge.terminate()
                try:
                    self.bridge.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.bridge.kill()
        if self.stub and self.stub.poll() is None:
            self.stub.terminate()
            try:
                self.stub.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.stub.kill()
        self.bridge = self.stub = None

    def cleanup(self):
        self.stop_scenario()
        for name, proc, log in self.procs:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
            log.close()

    # ---- 注入 ----

    def start_holder(self):
        marker = self.scenario_dir / 'holder-active'
        if marker.exists():
            marker.unlink()
        self.holder = subprocess.Popen(
            [sys.executable, '-c', HOLDER_SCRIPT, str(self.journal), str(marker)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            text=True, env=self.base_env())
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if marker.exists():
                return True
            time.sleep(0.002)
        return False

    def release_holder(self):
        if self.holder is None:
            return
        try:
            self.holder.stdin.write('release\n')
            self.holder.stdin.flush()
            self.holder.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            self.holder.terminate()
        self.holder = None

    def stub_control(self, **control):
        if self.stub and self.stub.poll() is None:
            try:
                self.stub.stdin.write(json.dumps(control) + '\n')
                self.stub.stdin.flush()
            except (OSError, ValueError):
                pass

    def wait_trigger(self, name, armed_at, timeout):
        path = self.scenario_dir / 'triggers' / name
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                value = float(path.read_text(encoding='utf-8'))
                if value >= armed_at:
                    return value
            except (OSError, ValueError):
                pass
            time.sleep(0.001)
        return None

    # ---- 样本流程 ----

    def wait_status(self, predicate, timeout, interval=0.1):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.status()
            if last and predicate(last):
                return last
            time.sleep(interval)
        return None

    def establish_auto_motion(self, sample):
        begin = self.send({'op': 'begin', 'mapPath': 'latency-stub://free'}, wait=10)
        if not begin or not begin[1].get('success'):
            sample['missed'] = 'begin_failed'
            return False
        sample['velocityTopic'] = begin[1].get('velocityTopic')
        ready = self.wait_status(lambda s: s.get('checks', {}).get('nav2', {}).get('activeNodes', 0) >= 2, 15)
        if not ready:
            sample['missed'] = 'lifecycle_not_ready'
            return False
        pose = self.send({'op': 'initial_pose', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}, wait=5)
        if not pose or not pose[1].get('success'):
            sample['missed'] = 'initial_pose_failed'
            return False
        ready = self.wait_status(lambda s: s.get('localized') and s.get('phase') == 'ready', 25)
        if not ready:
            sample['missed'] = 'startup_not_ready'
            return False
        sample['startupReadyAt'] = time.monotonic()
        # begin 清空了地图缓存；等替身 1Hz 重发布覆盖后再发目标。
        time.sleep(1.2)
        goals = self.send({'op': 'goals', 'points': [{'x': 1.0, 'y': 0.0}]}, wait=10)
        if not goals or not goals[1].get('success'):
            sample['missed'] = 'goals_failed'
            sample['goalsError'] = goals[1].get('error') if goals else None
            return False
        executing = self.wait_status(lambda s: s.get('goalState') == 'executing', 8)
        if not executing:
            sample['missed'] = 'goal_not_executing'
            return False
        self.stub_control(velocityTopic=sample['velocityTopic'], velocityEnabled=True, linearX=0.2)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            self.observer_tail.poll()
            evidence = motion_evidence(self.observer_tail.records, time.monotonic())
            if evidence['established']:
                sample['motion'] = 'auto'
                return True
            time.sleep(0.02)
        sample['missed'] = 'motion_not_established'
        return False

    def establish_manual_motion(self, sample, defer_pose=False):
        begin = self.send({'op': 'begin', 'mapPath': 'latency-stub://free'}, wait=10)
        if not begin or not begin[1].get('success'):
            sample['missed'] = 'begin_failed'
            return False
        sample['velocityTopic'] = begin[1].get('velocityTopic')
        ready = self.wait_status(lambda s: s.get('checks', {}).get('nav2', {}).get('activeNodes', 0) >= 2, 15)
        if not ready:
            sample['missed'] = 'lifecycle_not_ready'
            return False
        if not defer_pose:
            pose = self.send({'op': 'initial_pose', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}, wait=5)
            if not pose or not pose[1].get('success'):
                sample['missed'] = 'initial_pose_failed'
                return False
        enable = self.send({'op': 'manual_enable'}, wait=5)
        if not enable or not enable[1].get('success'):
            sample['missed'] = 'manual_enable_failed'
            sample['manualError'] = enable[1].get('error') if enable else None
            return False
        control_id = enable[1].get('controlId')
        sample['manualVelocityStop'] = threading.Event()
        sample['manualSequence'] = 0

        def velocity_loop():
            while not sample['manualVelocityStop'].is_set():
                sample['manualSequence'] += 1
                try:
                    self.send({'op': 'velocity', 'controlId': control_id,
                               'sequence': sample['manualSequence'],
                               'linearX': 0.2, 'angularZ': 0.0})
                except (OSError, ValueError):
                    return
                sample['manualVelocityStop'].wait(0.1)

        threading.Thread(target=velocity_loop, daemon=True).start()
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            self.observer_tail.poll()
            evidence = motion_evidence(self.observer_tail.records, time.monotonic())
            if evidence['established']:
                sample['motion'] = 'manual'
                return True
            time.sleep(0.02)
        sample['manualVelocityStop'].set()
        sample['missed'] = 'motion_not_established'
        return False

    def run_sample(self, name, index):
        config = SCENARIOS[name]
        sample = {'scenario': name, 'index': index, 'startedAt': time.monotonic()}
        try:
            return self._run_sample_inner(name, index, config, sample)
        except (OSError, ValueError) as error:
            sample['missed'] = f'harness_error:{type(error).__name__}:{error}'
            return sample
        finally:
            # 任何退出路径都停掉遥控速度 loop、替身速度输出与锁持有者，避免泄漏到下一样本。
            if sample.get('manualVelocityStop'):
                sample['manualVelocityStop'].set()
            self.stub_control(velocityEnabled=False)
            self.release_holder()

    def _run_sample_inner(self, name, index, config, sample):
        # 外层 run_sample 负责异常捕获与资源清理；此处保持单一退出路径返回样本。
        if True:
            if config['motion'] == 'auto':
                if not self.establish_auto_motion(sample):
                    return sample
            else:
                # stop_during_params：延迟 initial_pose 到运动建立之后——params 读取被
                # startup_done（需定位确认）门控，从而保证窗口必然晚于非零运动。
                defer = name == 'stop_during_params'
                if not self.establish_manual_motion(sample, defer_pose=defer):
                    return sample
                if defer:
                    pose = self.send({'op': 'initial_pose', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}, wait=5)
                    if not pose or not pose[1].get('success'):
                        sample['missed'] = 'initial_pose_failed'
                        return sample
            # 窗口探针：在指定窗口内发停止；canonical：运动稳定后发停止。
            armed_at = sample['startedAt']
            if config['trigger']:
                if name == 'stop_during_prepare':
                    # goals 需要 startup_done；等就绪期间遥控速度 loop 保持非零运动。
                    ready = self.wait_status(lambda s: s.get('phase') == 'ready', 25)
                    if not ready:
                        sample['missed'] = 'startup_not_ready'
                        return sample
                    time.sleep(1.2)  # 等替身地图重发布（begin 清空了缓存）
                    goals = self.send({'op': 'goals', 'points': [{'x': 1.0, 'y': 0.0}]})
                    sample['goalsRequestId'] = goals[0]
                trigger_at = self.wait_trigger(config['trigger'], armed_at,
                                               30 if name == 'stop_during_params' else 8)
                if trigger_at is None:
                    sample['missed'] = 'trigger_window_not_hit'
                    return sample
                sample['triggerAt'] = trigger_at
            else:
                time.sleep(0.3)
            if config['contention']:
                if not self.start_holder():
                    sample['missed'] = 'holder_failed'
                    return sample
                sample['holderActiveAt'] = time.monotonic()
                time.sleep(0.2)
            if sample.get('manualVelocityStop'):
                sample['manualVelocityStop'].set()
            self.observer_tail.poll()
            self.probe_tail.poll()
            sample['zeroSearchFrom'] = len(self.observer_tail.records)
            t0 = time.monotonic()
            stop_id, _ = self.send({'op': 'stop', 'requestId': f'{name}-{index}'})
            sample['t0'] = t0
            sample['stopId'] = stop_id
            sample['motionEvidence'] = motion_evidence(self.observer_tail.records, t0)
            if not sample['motionEvidence']['established']:
                sample['missed'] = 'motion_not_established_at_stop'
            # 等待独立观察者看到首条零速。
            deadline = time.monotonic() + 6
            t3 = None
            while time.monotonic() < deadline:
                self.observer_tail.poll()
                t3 = first_zero_after(self.observer_tail.records, t0, sample['zeroSearchFrom'])
                if t3 is not None:
                    break
                time.sleep(0.005)
            sample['t3'] = t3
            if t3 is None and not sample.get('missed'):
                sample['missed'] = 'zero_not_observed'
            response = self.wait_response(stop_id, 8)
            sample['stopResponse'] = response[1] if response else None
            sample['tResp'] = response[0] if response else None
            # 等待取消确认（锁竞争下可能走 5s 超时；释放后再补一次取消以结清）。
            quiet = self.wait_status(lambda s: s.get('cancelConfirmed'), 8)
            sample['cancelConfirmedAt'] = time.monotonic() if quiet else None
            if quiet:
                sample['effectiveParams'] = quiet.get('effectiveParams')
            if not quiet and config['contention']:
                self.release_holder()
                sample['holderReleasedAt'] = time.monotonic()
                extra_id, _ = self.send({'op': 'cancel', 'requestId': f'{name}-{index}-settle'})
                extra = self.wait_response(extra_id, 6)
                sample['settleResponse'] = extra[1] if extra else None
                quiet = self.wait_status(lambda s: s.get('cancelConfirmed'), 5)
                sample['cancelConfirmedAt'] = time.monotonic() if quiet else None
            if config['contention']:
                self.release_holder()
            self.stub_control(velocityEnabled=False)
            time.sleep(0.2)
            self.probe_tail.poll()
            self.collect_marks(sample)
            return sample

    def collect_marks(self, sample):
        t0 = sample.get('t0')
        if t0 is None:
            return
        marks = self.probe_tail.records
        stop_id = sample.get('stopId')
        enqueue = [m for m in marks if m['event'] == 'enqueue' and m.get('id') == stop_id]
        process = [m for m in marks if m['event'] == 'process_begin' and m.get('id') == stop_id]
        zeros = [m['mono'] for m in marks if m['event'] == 'zero_publish' and m['mono'] >= t0]
        respond = [m for m in marks if m['event'] == 'respond' and m.get('id') == stop_id]
        sample['t1'] = enqueue[0]['mono'] if enqueue else None
        sample['t2'] = process[0]['mono'] if process else None
        sample['t2c'] = zeros[0] if zeros else None
        sample['tRespProbe'] = respond[0]['mono'] if respond else None
        segments = {}
        pairs = {'pipe_read': ('t1', 't0'), 'queue_wait': ('t2', 't1'),
                 'exec_to_zero': ('t2c', 't2'), 'dds_delivery': ('t3', 't2c'),
                 'total': ('t3', 't0')}
        for key, (end, start) in pairs.items():
            a, b = sample.get(start), sample.get(end)
            segments[key] = (b - a) if (a is not None and b is not None) else None
        sample['segments'] = segments
        window = [m for m in marks if m['event'] == 'tick' and t0 - 1.0 <= m['mono'] <= (sample.get('t3') or t0 + 2) + 1.0]
        sample['tickWindow'] = tick_stats([(m['mono'], m['end']) for m in window])
        params = [m for m in marks if m['event'] in ('params_read_begin', 'params_read_end')
                  and m['mono'] >= sample['startedAt']]
        begins = [m['mono'] for m in params if m['event'] == 'params_read_begin']
        ends = [m['mono'] for m in params if m['event'] == 'params_read_end']
        sample['paramsReadSeconds'] = (ends[0] - begins[0]) if begins and ends else None
        prep = [m for m in marks if m['event'] in ('journal_prepare_begin', 'journal_prepare_end')
                and m['mono'] >= sample['startedAt']]
        pb = [m['mono'] for m in prep if m['event'] == 'journal_prepare_begin']
        pe = [m['mono'] for m in prep if m['event'] == 'journal_prepare_end']
        sample['journalPrepareSeconds'] = (pe[0] - pb[0]) if pb and pe else None
        sample['confirmSeconds'] = (sample['tResp'] - t0) if sample.get('tResp') else None

    # ---- 场景与报告 ----

    def run_scenario(self, name, samples):
        config = SCENARIOS[name]
        scenario_start_wall = time.time()
        print(f'[{name}] 启动（params={config["params"]}, fsync={config["fsync_ms"]}ms, '
              f'contention={config["contention"]}, motion={config["motion"]}）', flush=True)
        if not self.start_scenario(name):
            return {'scenario': name, 'error': 'bridge_not_ready', 'samples': []}
        records = []
        for index in range(samples):
            record = self.run_sample(name, index)
            records.append(record)
            with open(self.out / name / 'samples.jsonl', 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(record, ensure_ascii=False, default=str) + '\n')
            total = record.get('segments', {}).get('total')
            print(f'[{name}] 样本 {index + 1}/{samples}: '
                  f'{"漏测 " + record["missed"] if record.get("missed") else f"total={ms(total)}ms"}',
                  flush=True)
            if str(record.get('missed', '')).startswith(('begin_failed', 'harness_error')):
                # begin 失败多因上一样本残留未确认目标：重启 bridge 保证下一样本干净。
                # goals_failed（如重档慢磁盘的保护性取消）终态会自然结清，无需重启。
                self.stop_scenario()
                if not self.start_scenario(name):
                    break
        self.stop_scenario()
        return self.summarize(name, records, scenario_start_wall)

    def summarize(self, name, records, scenario_start_wall=None):
        # 重新整读探针文件：场景中途若重启过 bridge，内存 tail 只覆盖最后一个实例。
        full_probe = Tail(self.scenario_dir / 'probe.jsonl')
        full_probe.poll()
        self.probe_tail = full_probe
        ticks = [(m['mono'], m['end']) for m in full_probe.records if m['event'] == 'tick']
        summary = {
            'scenario': name, 'config': SCENARIOS[name],
            'sampleCount': len(records),
            'missedCount': sum(1 for r in records if r.get('missed')),
            'missedReasons': dict(collections.Counter(r['missed'] for r in records if r.get('missed'))),
            'segments': {key: segment_stats(records, key) for key in SEGMENTS},
            'tickAll': tick_stats(ticks),
            'confirmation': segment_stats(
                [{'segments': {'confirm': r.get('confirmSeconds')}} for r in records], 'confirm'),
            'paramsReadSeconds': [r.get('paramsReadSeconds') for r in records
                                  if r.get('paramsReadSeconds') is not None],
            'journalPrepareSeconds': [r.get('journalPrepareSeconds') for r in records
                                      if r.get('journalPrepareSeconds') is not None],
        }
        worst = max((r for r in records if r.get('segments', {}).get('total') is not None),
                    key=lambda r: r['segments']['total'], default=None)
        if worst:
            summary['worstSample'] = worst
            summary['worstTimeline'] = self.timeline(worst)
        # 证据聚合：替身实际被调用的参数服务；账本写入失败事件（本场景时间窗内）。
        stub_calls = collections.Counter()
        stub_path = self.scenario_dir / 'stub.jsonl'
        if stub_path.exists():
            for line in stub_path.read_text(encoding='utf-8').splitlines():
                try:
                    record = json.loads(line)
                except ValueError:
                    continue
                if record.get('event') == 'get_parameters_called':
                    stub_calls[record.get('service', '')] += 1
        summary['paramServiceCalls'] = dict(stub_calls)
        journal_failures = []
        events_path = self.out / 'bridge-logs' / 'bridge-events.jsonl'
        if events_path.exists() and scenario_start_wall:
            for line in events_path.read_text(encoding='utf-8').splitlines():
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if (event.get('type') in ('action_journal_failed', 'run_manifest_failed')
                        and event.get('at', 0) >= scenario_start_wall):
                    journal_failures.append({'type': event['type'], 'reason': event.get('reason'),
                                             'at': event['at']})
        summary['journalFailureEvents'] = journal_failures[:50]
        summary['journalFailureCount'] = len(journal_failures)
        with open(self.out / name / 'summary.json', 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2, default=str)
        return summary

    def timeline(self, sample):
        t0 = sample['t0']
        end = (sample.get('t3') or t0 + 2) + 0.6
        start = t0 - 0.6
        events = []
        for mark in self.probe_tail.records:
            if start <= mark['mono'] <= end and mark['event'] != 'tick':
                events.append({'mono': mark['mono'], 'source': 'probe', **{k: v for k, v in mark.items() if k not in ('mono', 'wall', 'pid')}})
        for msg in self.observer_tail.records:
            if start <= msg['mono'] <= end and msg.get('topic') == '/cmd_vel':
                events.append({'mono': msg['mono'], 'source': 'observer', 'topic': '/cmd_vel',
                               'lx': msg['lx'], 'az': msg['az']})
        events.sort(key=lambda e: e['mono'])
        for event in events:
            event['relMs'] = ms(event['mono'] - t0)
        return events

    def write_report(self, summaries, meta):
        report = build_report(summaries, meta)
        (self.out / 'report.md').write_text(report, encoding='utf-8')
        with open(self.out / 'summary-all.json', 'w', encoding='utf-8') as handle:
            json.dump({'meta': meta, 'summaries': summaries}, handle,
                      ensure_ascii=False, indent=2, default=str)


def build_report(summaries, meta):
    lines = ['# D 工单停车延迟基线报告（修复前实测）', '',
             f'- 生成时刻：{meta["generatedAt"]}',
             f'- 环境：ROS_DISTRO={meta.get("rosDistro")}，RMW={meta.get("rmw")}（运行时实证），'
             f'ROS_DOMAIN_ID=194（私有），ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
             f'- 代码版本：工作区未修复代码（navigation_bridge.py sha256={meta.get("bridgeSha256")}）',
             f'- 注入参数：fsync 延迟 {meta.get("fsyncMs")}ms/次；取消终态延迟 {meta.get("cancelDelay")}s；'
             f'SQLite timeout=0（产品既有）',
             '- 时钟：全部 time.monotonic()（CLOCK_MONOTONIC，同机跨进程可比）',
             '- 百分位方法：nearest-rank（sorted[ceil(q*n)-1]）；单位 ms',
             '- 场景负载：begin→initial_pose→goals→替身 20Hz 非零速度（auto）或 10Hz 遥控（manual 探针），'
             '运动建立后由外部编排器经 stdin 发 stop', '']
    lines += ['## 场景矩阵（软件链路延迟，t3-t0）', '',
              '| 场景 | 样本 | 漏测 | p50 | p95 | p99 | max |',
              '|---|---|---|---|---|---|---|']
    for summary in summaries:
        total = summary['segments']['total']
        lines.append(f'| {summary["scenario"]} | {total["n"]} | {summary["missedCount"]} | '
                     f'{ms(total["p50"])} | {ms(total["p95"])} | {ms(total["p99"])} | {ms(total["max"])} |')
    lines.append('')
    for summary in summaries:
        lines += [f'## {summary["scenario"]}', '',
                  f'配置：`{json.dumps(summary["config"], ensure_ascii=False)}`', '',
                  '| 分段 | n | 漏测 | p50 | p95 | p99 | max |',
                  '|---|---|---|---|---|---|---|']
        for key in SEGMENTS:
            stats = summary['segments'][key]
            lines.append(f'| {SEGMENT_LABELS[key]} | {stats["n"]} | {stats["missed"]} | '
                         f'{ms(stats["p50"])} | {ms(stats["p95"])} | {ms(stats["p99"])} | {ms(stats["max"])} |')
        confirm = summary['confirmation']
        lines.append(f'| 停止响应确认 tResp-t0 | {confirm["n"]} | {confirm["missed"]} | '
                     f'{ms(confirm["p50"])} | {ms(confirm["p95"])} | {ms(confirm["p99"])} | {ms(confirm["max"])} |')
        tick = summary['tickAll']['interval']
        lines += ['', f'相邻 tick 间隔（全场景）：n={tick["n"]}，p50={ms(tick["p50"])}ms，'
                      f'p95={ms(tick["p95"])}ms，p99={ms(tick["p99"])}ms，max={ms(tick["max"])}ms', '']
        if summary['missedReasons']:
            lines += [f'漏测原因：{json.dumps(summary["missedReasons"], ensure_ascii=False)}', '']
        for key in ('paramsReadSeconds', 'journalPrepareSeconds'):
            values = summary.get(key) or []
            if values:
                lines.append(f'{key}: n={len(values)}，p50={ms(percentile(values, .5))}ms，'
                             f'max={ms(max(values))}ms')
        for key in SEGMENTS:
            raw = summary['segments'][key].get('raw')
            if raw is not None:
                lines.append(f'{key} 原始值（n<20，ms）：{[ms(v) for v in raw]}')
        worst = summary.get('worstSample')
        if worst:
            lines += ['', f'最差样本 #{worst["index"]}：total={ms(worst["segments"]["total"])}ms，'
                          f't0={worst["t0"]:.6f}，运动证据={json.dumps(worst.get("motionEvidence"), ensure_ascii=False)}',
                          '', '时间线（相对 t0，ms）：', '', '```json']
            lines.append(json.dumps(summary.get('worstTimeline', []), ensure_ascii=False, indent=1))
            lines += ['```', '']
    lines += ['## 边界与未验证项', '',
              '- 本报告只测软件链路（请求→Bridge→/cmd_vel 零速被独立订阅者观察）；'
              '实车制动距离未测量（无实车），需另行记录 d = v·t_link + v²/(2a)，'
              '其中 v 为停止前速度（本测试 0.2 m/s），a 为实车制动减速度（未核验）。',
              '- 合成底盘 offline_robot.py 有 0.4s 指令看门狗；本测量以独立订阅者观察 /cmd_vel 为准，'
              '不受底盘看门狗影响。',
              '- Nav2 为最小替身（生命周期/Action/参数/定位输入），不是真实 Nav2 控制器；'
              '真实 Nav2 的取消终态延迟可能不同（替身默认 0.05s，可配）。',
              '- 慢磁盘注入作用于 fsync/fdatasync（SQLite synchronous=FULL 路径）；'
              '不代表真实磁盘的全部延迟形态（如调度抖动）。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--probe-samples', type=int, default=15)
    parser.add_argument('--quick', action='store_true', help='冒烟：每场景 3 样本')
    parser.add_argument('--scenarios', default=','.join(SCENARIOS))
    parser.add_argument('--fsync-ms', type=int, default=150)
    parser.add_argument('--cancel-delay', type=float, default=0.05)
    args = parser.parse_args()
    if not os.environ.get('ROS_DISTRO'):
        sys.exit('请先 source /opt/ros/lyrical/setup.bash')
    names = [n.strip() for n in args.scenarios.split(',') if n.strip()]
    for name in names:
        if name not in SCENARIOS:
            sys.exit(f'未知场景 {name}')
    orchestrator = Orchestrator(args.out, fsync_ms=args.fsync_ms, cancel_delay=args.cancel_delay)
    meta = {'generatedAt': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'rosDistro': os.environ.get('ROS_DISTRO'),
            'rmw': os.environ.get('RMW_IMPLEMENTATION') or 'rmw_fastrtps_cpp（发行版默认，运行时实证）',
            'fsyncMs': args.fsync_ms, 'cancelDelay': args.cancel_delay,
            'samples': args.samples, 'probeSamples': args.probe_samples,
            'bridgeSha256': None}
    import hashlib
    meta['bridgeSha256'] = hashlib.sha256((SCRIPTS / 'navigation_bridge.py').read_bytes()).hexdigest()
    # 运行时实证 RMW（不依赖环境变量推断）。
    try:
        probe = subprocess.run(
            [sys.executable, '-c',
             'import rclpy; rclpy.init(); '
             'from rclpy.utilities import get_rmw_implementation_identifier as g; '
             'print(g()); rclpy.shutdown()'],
            capture_output=True, text=True, timeout=30, env=orchestrator.base_env())
        if probe.returncode == 0 and probe.stdout.strip():
            meta['rmw'] = probe.stdout.strip() + '（运行时实证）'
    except (subprocess.TimeoutExpired, OSError):
        pass
    summaries = []
    try:
        orchestrator.start_global()
        for name in names:
            count = 3 if args.quick else (args.probe_samples if name in PROBE_SCENARIOS else args.samples)
            summaries.append(orchestrator.run_scenario(name, count))
    finally:
        orchestrator.cleanup()
        orchestrator.write_report(summaries, meta)
    print(f'报告：{orchestrator.out / "report.md"}', flush=True)


if __name__ == '__main__':
    main()