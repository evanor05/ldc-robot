"""D 工单测量工具单测：分析函数纯逻辑 + 入队时间戳包装（不需要运行中的 ROS）。

覆盖基线报告赖以成立的判定：
- 百分位 nearest-rank 口径；
- 「停止后首条零速」只数 t0 之后，之前的周期零速不计入；
- 「先建立非零运动」的证据判定（条数 + 最后非零距 t0 的间隔）；
- tick 间隔/耗时统计与分段漏测计数；
- TimedQueue.put 的单调时刻记录顺序。
"""
import importlib.util
import math
from pathlib import Path
import sys
import time
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


run = load('latency_run')


def cmd(mono, lx=0.0, az=0.0):
    return {'topic': '/cmd_vel', 'mono': mono, 'lx': lx, 'az': az,
            'ly': 0.0, 'lz': 0.0, 'ax': 0.0, 'ay': 0.0}


class PercentileTests(unittest.TestCase):
    def test_nearest_rank(self):
        values = [10, 20, 30, 40, 50]
        self.assertEqual(run.percentile(values, .50), 30)
        self.assertEqual(run.percentile(values, .95), 50)
        self.assertEqual(run.percentile(values, .99), 50)
        self.assertEqual(run.percentile(values, 0.0), 10)

    def test_single_and_empty(self):
        self.assertEqual(run.percentile([7], .99), 7)
        self.assertIsNone(run.percentile([], .5))

    def test_unsorted_input(self):
        self.assertEqual(run.percentile([50, 10, 30, 20, 40], .5), 30)


class ZeroDetectionTests(unittest.TestCase):
    def test_periodic_zeros_before_stop_do_not_count(self):
        messages = [cmd(1.0), cmd(1.05), cmd(1.10, lx=0.2), cmd(1.15, lx=0.2),
                    cmd(1.20), cmd(1.25)]
        # t0=1.12：之前的周期零速（1.0/1.05）不得计入；首条零速是 1.20。
        self.assertEqual(run.first_zero_after(messages, 1.12), 1.20)

    def test_nonzero_after_stop_is_skipped(self):
        messages = [cmd(1.0, lx=0.2), cmd(1.1, lx=0.2), cmd(1.2, lx=0.1), cmd(1.3)]
        self.assertEqual(run.first_zero_after(messages, 1.05), 1.3)

    def test_partial_zero_is_not_zero(self):
        messages = [cmd(1.0, lx=0.0, az=0.3), cmd(1.1)]
        self.assertEqual(run.first_zero_after(messages, 0.9), 1.1)

    def test_no_zero_returns_none(self):
        self.assertIsNone(run.first_zero_after([cmd(1.0, lx=0.2)], 0.5))

    def test_start_index(self):
        messages = [cmd(1.0), cmd(2.0), cmd(3.0)]
        self.assertEqual(run.first_zero_after(messages, 0.5, start=2), 3.0)


class MotionEvidenceTests(unittest.TestCase):
    def test_established(self):
        messages = [cmd(0.90, lx=0.2), cmd(0.95, lx=0.2), cmd(1.00, lx=0.2)]
        evidence = run.motion_evidence(messages, 1.05)
        self.assertTrue(evidence['established'])
        self.assertEqual(evidence['nonzeroCount'], 3)
        self.assertEqual(evidence['lastNonzero'], 1.00)

    def test_too_few_nonzero(self):
        messages = [cmd(0.95, lx=0.2), cmd(1.00, lx=0.2)]
        self.assertFalse(run.motion_evidence(messages, 1.05)['established'])

    def test_stale_motion(self):
        messages = [cmd(0.10, lx=0.2), cmd(0.15, lx=0.2), cmd(0.20, lx=0.2)]
        self.assertFalse(run.motion_evidence(messages, 1.05)['established'])

    def test_zeros_do_not_count_as_motion(self):
        messages = [cmd(0.90), cmd(0.95), cmd(1.00)]
        self.assertFalse(run.motion_evidence(messages, 1.05)['established'])


class TickAndSegmentTests(unittest.TestCase):
    def test_tick_stats(self):
        ticks = [(0.00, 0.002), (0.05, 0.052), (0.10, 0.102), (0.20, 0.202)]
        stats = run.tick_stats(ticks)
        self.assertEqual(stats['interval']['n'], 3)
        self.assertAlmostEqual(stats['interval']['max'], 0.10, places=6)
        self.assertAlmostEqual(stats['interval']['p50'], 0.05, places=6)
        self.assertAlmostEqual(stats['duration']['p50'], 0.002, places=6)

    def test_segment_stats_counts_missed(self):
        samples = [
            {'segments': {'total': 0.05}},
            {'segments': {'total': 0.08}},
            {'segments': {'total': None}},
            {'missed': 'zero_not_observed'},
        ]
        stats = run.segment_stats(samples, 'total')
        self.assertEqual(stats['n'], 2)
        self.assertEqual(stats['missed'], 2)
        self.assertEqual(stats['max'], 0.08)
        self.assertEqual(stats['raw'], [0.05, 0.08])  # n<20 附原始值

    def test_segment_stats_no_raw_when_enough(self):
        samples = [{'segments': {'total': i / 1000}} for i in range(25)]
        stats = run.segment_stats(samples, 'total')
        self.assertEqual(stats['n'], 25)
        self.assertIsNone(stats['raw'])


class TimedQueueTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = load('latency_probe')

    def test_put_records_monotonic_order(self):
        seen = []
        queue = self.probe.TimedQueue(on_put=lambda item, at: seen.append((item, at)))
        before = time.monotonic()
        queue.put({'op': 'stop', 'id': 'x'})
        queue.put({'op': 'status', 'id': 'y'})
        after = time.monotonic()
        self.assertEqual(len(seen), 2)
        self.assertEqual(seen[0][0]['op'], 'stop')
        self.assertTrue(before <= seen[0][1] <= seen[1][1] <= after)
        self.assertEqual(queue.get_nowait()['id'], 'x')

    def test_queue_without_callback(self):
        queue = self.probe.TimedQueue()
        queue.put({'op': 'stop'})
        self.assertEqual(queue.get_nowait()['op'], 'stop')


if __name__ == '__main__':
    unittest.main()