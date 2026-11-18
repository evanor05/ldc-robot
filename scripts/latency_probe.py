#!/usr/bin/env python3
"""D 工单停车延迟探针（首轮：只新增测量代码，不改任何产品文件）。

以子类方式包装 navigation_bridge.Bridge：所有 super() 调用执行原产品逻辑，
子类只追加 time.monotonic()（CLOCK_MONOTONIC，同机跨进程可比）时间戳采集：

四时刻中的 Bridge 侧三个：
  t1 enqueue        —— command_reader 线程把停止请求放入 commands 队列（包装 Queue.put）
  t2 process_begin  —— ROS 执行线程 tick 取出该请求开始 process()
  t2c zero_publish  —— 执行线程调用 zero() 发布零速（lock 路径内）
t0 由外部编排器记录（写 stdin 前），t3 由独立订阅者进程 latency_observer.py 记录。

另采集：相邻 tick 实际间隔与单次 tick 耗时、request_effective_params 同步块
起止、journal.prepare / _action_event 起止、lock 起止、停止响应时刻。

触发标记（供编排器在指定窗口内注入停止请求，纯测量用途）：
  --trigger-dir 目录下写 params_start / prepare_start 标记文件（内容为 monotonic）。

运行方式与 navigation_bridge.main() 一致：spin_once(timeout_sec=.1)，stdin JSON 行。
"""
import argparse
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy  # noqa: E402
import navigation_bridge as nb  # noqa: E402


class TimedQueue(queue.Queue):
    """put 时记录单调时刻；回调用于把入队事件写入探针时间线。"""

    def __init__(self, on_put=None):
        super().__init__()
        self._on_put = on_put

    def put(self, item, block=True, timeout=None):
        at = time.monotonic()
        super().put(item, block, timeout)
        if self._on_put is not None:
            self._on_put(item, at)


class ProbeBridge(nb.Bridge):
    MARKED_OPS = ('stop', 'cancel', 'estop', 'begin', 'goals', 'initial_pose', 'manual_enable')

    def __init__(self, command_stream=None, journal_path=None, probe_out=None, trigger_dir=None):
        self._probe_lock = threading.Lock()
        self._probe_handle = open(probe_out, 'a', buffering=1, encoding='utf-8') if probe_out else None
        self._trigger_dir = Path(trigger_dir) if trigger_dir else None
        if self._trigger_dir:
            self._trigger_dir.mkdir(parents=True, exist_ok=True)
        super().__init__(command_stream=command_stream, journal_path=journal_path)
        # 包装入队队列：read_commands 线程动态引用 self.commands，替换安全。
        self.commands = TimedQueue(on_put=self._on_enqueue)

    # ---- 时间线输出 ----

    def _mark(self, event, at=None, **fields):
        if self._probe_handle is None:
            return
        record = {'event': event, 'mono': at if at is not None else time.monotonic(),
                  'wall': time.time(), 'pid': os.getpid(), **fields}
        with self._probe_lock:
            try:
                self._probe_handle.write(json.dumps(record, ensure_ascii=False) + '\n')
            except (OSError, ValueError):
                pass

    def _on_enqueue(self, item, at):
        if isinstance(item, dict):
            self._mark('enqueue', at, op=item.get('op'), id=item.get('id'))

    def _trigger(self, name):
        if self._trigger_dir is None:
            return
        try:
            tmp = self._trigger_dir / (name + '.tmp')
            tmp.write_text(repr(time.monotonic()), encoding='utf-8')
            tmp.replace(self._trigger_dir / name)
        except OSError:
            pass

    # ---- 包装点（全部透传 super，不改逻辑）----

    def tick(self):
        start = time.monotonic()
        try:
            super().tick()
        finally:
            end = time.monotonic()
            self._mark('tick', start, end=end)

    def process(self, request):
        op = request.get('op') if isinstance(request, dict) else None
        marked = op in self.MARKED_OPS
        if marked:
            self._mark('process_begin', op=op, id=request.get('id'))
        try:
            return super().process(request)
        finally:
            if marked:
                self._mark('process_end', op=op, id=request.get('id'))

    def zero(self):
        self._mark('zero_publish')
        super().zero()

    def lock(self, reason='', kind='protective'):
        self._mark('lock_begin', reason=reason, kind=kind)
        try:
            super().lock(reason, kind)
        finally:
            self._mark('lock_end', kind=kind)

    def request_effective_params(self):
        self._mark('params_read_begin')
        self._trigger('params_start')
        try:
            super().request_effective_params()
        finally:
            self._mark('params_read_end')

    def _before_action_send(self, identifier, token):
        self._mark('journal_prepare_begin', rosGoalId=identifier, goalId=token)
        self._trigger('prepare_start')
        try:
            super()._before_action_send(identifier, token)
        finally:
            self._mark('journal_prepare_end', rosGoalId=identifier, goalId=token)

    def _action_event(self, kind, reason='', **extra):
        self._mark('action_event_begin', kind=kind, rosGoalId=extra.get('rosGoalId'))
        try:
            super()._action_event(kind, reason, **extra)
        finally:
            self._mark('action_event_end', kind=kind, rosGoalId=extra.get('rosGoalId'),
                       error=self.action_journal.error or None)

    def respond(self, request, **data):
        if isinstance(request, dict) and request.get('op') in ('stop', 'cancel', 'estop'):
            self._mark('respond', id=request.get('id'), success=data.get('success'),
                       error=data.get('error'))
        super().respond(request, **data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe-out', required=True, help='探针时间线 JSONL 输出路径')
    parser.add_argument('--trigger-dir', default=None, help='窗口触发标记目录（可选）')
    parser.add_argument('--journal-path', default=None, help='账本路径（默认走产品环境变量逻辑）')
    args = parser.parse_args()
    rclpy.init()
    node = ProbeBridge(command_stream=sys.stdin, journal_path=args.journal_path,
                       probe_out=args.probe_out, trigger_dir=args.trigger_dir)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.lock()
        node.manifest_finish('aborted', 'latency probe 退出')
        node._append_log('bridge-samples', {'kind': 'session-end', 'at': time.time(),
                                           'mode': node.mode, 'lockKind': node.lock_kind,
                                           'lastStop': node.last_stop})
        node.close_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()