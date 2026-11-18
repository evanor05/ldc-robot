"""C：初始位姿与启动状态机的离线测试。

覆盖 2026-11-11 18:04:29 的三个根因：
1. /initialpose 用 now 当 stamp → AMCL 报 future TF；
2. 定位确认用"收包时刻"判定 → 旧消息也能被当成已确认；
3. 只有一个从启动起算的 60s 超时 → 页面只能看到"导航启动超时"。
"""
import importlib.util
import time
import types
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

spec = importlib.util.spec_from_file_location(
    'scripts.navigation_bridge', Path(__file__).resolve().parents[1] / 'navigation_bridge.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

SESSION = 'unit-test-session'


class FakeClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return rclpy.time.Time(nanoseconds=self.ns, clock_type=rclpy.clock.ClockType.ROS_TIME)


class FakeBuffer:
    """可控的 TF：只回答显式登记过的 parent/child。"""

    def __init__(self, stamps):
        self.stamps = stamps

    def can_transform(self, parent, child, stamp):
        return (parent, child) in self.stamps

    def lookup_transform(self, parent, child, stamp):
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=self.stamps[(parent, child)]))


def msg_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def make_node():
    node = module.Bridge()
    node.action.destroy()
    node.action = FakeAction()
    node.requirements = lambda: []
    node.sensors = lambda: []
    node.running = True
    node.replies = []
    node.respond = lambda request, **data: node.replies.append((request, data))
    node.process({'op': 'claim', 'id': 0, 'sessionId': SESSION})
    node.replies.clear()
    published = []
    node.pose.publish = published.append       # 收集 /initialpose，不真的发 DDS
    clock = FakeClock()
    node.get_clock = lambda: clock
    node.lifecycle['amcl'] = (3, time.monotonic())
    return node, clock, published


def request_pose(node, x=1.0, y=2.0, yaw=0.0):
    node.process({'op': 'initial_pose', 'id': 1, 'sessionId': SESSION,
                  'x': x, 'y': y, 'yaw': yaw})


class FakeAction:
    """status()/structured_checks() 会问 Action 是否就绪，测试里恒为就绪。"""

    def server_is_ready(self):
        return True

    def destroy(self):
        pass


def feed_odometry(node, clock, stamp_ns):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
    msg.pose.pose.orientation.w = 1.0
    clock.ns = stamp_ns
    node.on_odom(msg)


def feed_pose(node, clock, stamp_ns, x=1.0, y=2.0):
    feed_odometry(node, clock, stamp_ns)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.0685
    clock.ns = stamp_ns + 100_000_000
    node.on_pose(msg)


class InitialPoseStampTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def tearDown(self):
        self.node.destroy_node()

    def test_stamp_falls_back_before_now_and_never_in_future(self):
        """没有可用 TF 时必须回退 now-0.2s，绝不能用 now（AMCL future TF 的根因）。"""
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        request_pose(self.node)
        self.assertEqual(len(published), 1)
        stamped = msg_ns(published[-1].header.stamp)
        self.assertEqual(stamped, clock.ns - 200_000_000)
        self.assertLess(stamped, clock.ns)

    def test_stamp_prefers_latest_transform(self):
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        tf_stamp = rclpy.time.Time(nanoseconds=clock.ns - 100_000_000).to_msg()
        self.node.buffer = FakeBuffer({('map', 'base_footprint'): tf_stamp})
        request_pose(self.node)
        self.assertEqual(msg_ns(published[-1].header.stamp), msg_ns(tf_stamp))

    def test_future_transform_is_ignored(self):
        """TF 在"未来"（时钟不同步）时不能照抄，否则又变成 future TF。"""
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        future = rclpy.time.Time(nanoseconds=clock.ns + 2_000_000_000).to_msg()
        self.node.buffer = FakeBuffer({('map', 'base_footprint'): future})
        request_pose(self.node)
        stamped = msg_ns(published[-1].header.stamp)
        self.assertLessEqual(stamped, clock.ns)
        self.assertEqual(stamped, clock.ns - 200_000_000)

    def test_stale_transform_falls_back(self):
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        stale = rclpy.time.Time(nanoseconds=clock.ns - 5_000_000_000).to_msg()
        self.node.buffer = FakeBuffer({('map', 'odom'): stale})
        request_pose(self.node)
        self.assertEqual(msg_ns(published[-1].header.stamp), clock.ns - 200_000_000)


class LocalizationConfirmationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def tearDown(self):
        self.node.destroy_node()

    def test_old_amcl_message_does_not_confirm(self):
        """请求之前产生的 AMCL 位姿（延迟到达）不得置 localization_confirmed。"""
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        request_pose(self.node)
        requested = self.node.pose_requested_stamp_ns
        self.assertGreater(requested, 0)
        feed_pose(self.node, clock, requested - 500_000_000)
        self.assertFalse(self.node.localization_confirmed)
        # 旧消息的 stamp 仍被记录，便于失败时报出 last_pose_stamp
        self.assertEqual(self.node.last_pose_stamp_ns, requested - 500_000_000)

    def test_valid_pose_after_request_confirms_quickly(self):
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        request_pose(self.node)
        requested = self.node.pose_requested_stamp_ns
        feed_pose(self.node, clock, requested + 100_000_000)
        self.assertTrue(self.node.localization_confirmed)
        confirmed = [event for event in self.node.events if event['type'] == 'localization_confirmed']
        self.assertTrue(confirmed)
        self.assertEqual(confirmed[-1].get('requestedStamp'), requested)

    def test_retries_send_five_times_then_stop(self):
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        request_pose(self.node)
        self.assertEqual(len(published), 1)
        for _ in range(8):
            self.node.initial_pose_next_at = 0     # 模拟到达重发时刻
            self.node.service_initial_pose()
        self.assertEqual(len(published), 5, '默认共 5 次（首次 + 4 次重发）')
        self.assertEqual(self.node.initial_pose_retries_left, 0)

    def test_confirmed_pose_stops_retries(self):
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        request_pose(self.node)
        feed_pose(self.node, clock, self.node.pose_requested_stamp_ns + 100_000_000)
        self.node.initial_pose_next_at = 0
        self.node.service_initial_pose()
        self.assertEqual(len(published), 1, '确认后不得继续重发')

    def test_timeout_reports_code_stamps_and_phase(self):
        """超时必须带 code/requestedStamp/lastPoseStamp，而不是只有"启动超时"。"""
        self.node, clock, published = make_node()
        clock.ns = 1_700_000_000_000_000_000
        request_pose(self.node)
        self.node.initial_pose_retries_left = 0
        self.node.initial_pose_sent_at = time.monotonic() - (self.node.initial_pose_timeout + 1)
        self.node.service_initial_pose()
        failure = self.node.initial_pose_failed
        self.assertIsNotNone(failure)
        self.assertEqual(failure['code'], 'initial_pose_timeout')
        self.assertEqual(failure['requestedStamp'], self.node.pose_requested_stamp_ns)
        self.assertIn('lastPoseStamp', failure)
        self.assertIn('初始位置', self.node.error)
        localization = self.node.localization_check()
        self.assertFalse(localization['ok'])
        self.assertEqual(localization['code'], 'initial_pose_timeout')
        status = self.node.status()
        self.assertEqual(status['phase'], 'failed')
        self.assertEqual(status['initialPoseFailure']['code'], 'initial_pose_timeout')


class StartupTimeoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def tearDown(self):
        self.node.destroy_node()

    def test_timeout_counts_from_last_progress_not_from_start(self):
        """启动 300s 前开始，但 5s 前刚有进展 → 不能判超时。"""
        self.node, clock, published = make_node()
        self.node.nav_started = time.monotonic() - 300
        self.node.last_progress_at = time.monotonic() - 5
        self.node.pose_requested = float('inf')
        self.node.lifecycle = {}
        self.node.start_localized_navigation()
        self.assertFalse(self.node.startup_failed)

    def test_timeout_after_progress_goes_stale_reports_details(self):
        self.node, clock, published = make_node()
        self.node.nav_started = time.monotonic() - 300
        self.node.last_progress_at = time.monotonic() - (self.node.startup_timeout + 5)
        self.node.lifecycle = {'map_server': (3, time.monotonic())}   # 只有前半激活
        self.node.start_localized_navigation()
        self.assertTrue(self.node.startup_failed)
        failure = self.node.startup_failure
        self.assertEqual(failure['code'], 'startup_timeout')
        self.assertIn('requestedStamp', failure)
        self.assertIn('localizationCode', failure)
        # 错误信息不能只有"导航启动超时"，必须带定位状态与节点数
        self.assertIn('节点激活', failure['detail'])
        self.assertIn('定位', failure['detail'])
        event = [item for item in self.node.events if item['type'] == 'startup_failed'][-1]
        self.assertIn('localizationCode', event)


if __name__ == '__main__':
    unittest.main()
