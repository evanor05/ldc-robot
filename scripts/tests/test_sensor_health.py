"""Real Bridge callbacks: reception does not establish measurement validity."""
import math
import unittest
from unittest.mock import patch, Mock
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from scripts.navigation_bridge import Bridge


class SensorHealthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()
    def setUp(self):
        with patch.object(Bridge, 'read_commands'):
            self.node = Bridge()
        self.node.fresh_transform = lambda *args: True
    def tearDown(self): self.node.destroy_node()
    def odom(self):
        msg = Odometry(); msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'; msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.
        return msg
    def scan(self):
        msg = LaserScan(); msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'; msg.angle_min = -math.pi
        msg.angle_increment = math.pi / 180; msg.angle_max = msg.angle_min + 359 * msg.angle_increment
        msg.range_min = .08; msg.range_max = 8.; msg.scan_time = .1
        msg.ranges = [float('inf')] * 360
        return msg
    def feed(self):
        odom, scan = self.odom(), self.scan()
        self.node.on_odom(odom); self.node.on_scan(scan)
        return odom, scan
    def test_live_stationary_odom_and_all_inf_scan_are_valid(self):
        self.feed(); self.assertEqual(self.node.sensors(), [])
    def test_old_and_future_packets_do_not_establish_health(self):
        odom, scan = self.odom(), self.scan()
        odom.header.stamp.sec -= 10; scan.header.stamp.sec += 10
        self.node.on_odom(odom); self.node.on_scan(scan)
        self.assertTrue(self.node.sensors())
        self.assertFalse(self.node.structured_checks()['odometry']['ok'])
        self.assertFalse(self.node.structured_checks()['lidar']['ok'])
    def test_invalid_odom_and_all_nan_scan_invalidate_previous_health(self):
        self.feed(); odom, scan = self.odom(), self.scan()
        odom.pose.pose.orientation.w = 0.; scan.ranges = [float('nan')]*360
        self.node.on_odom(odom); self.node.on_scan(scan)
        self.assertTrue(self.node.sensors())
    def test_bad_frame_and_scan_geometry_are_rejected(self):
        odom, scan = self.odom(), self.scan()
        odom.child_frame_id = 'map'; scan.angle_max = 0.
        self.node.on_odom(odom); self.node.on_scan(scan)
        self.assertTrue(self.node.sensors())
    def test_duplicate_stamp_does_not_refresh_valid_measurement_time(self):
        odom, scan = self.feed()
        self.node.odom_health.accepted_at -= 2
        self.node.scan_health.accepted_at -= 2
        self.node.on_odom(odom); self.node.on_scan(scan)
        self.assertTrue(self.node.sensors())
    def test_sensor_failure_stops_motion_and_recovery_does_not_rearm(self):
        self.feed(); self.node.mode = 'manual'; self.node.owner = 'test'
        import time
        self.node.lease_at = self.node.manual_at = time.monotonic()
        self.node.velocity = Mock(); self.node.estop = Mock()
        bad = self.odom(); bad.twist.twist.linear.x = float('nan')
        self.node.on_odom(bad); self.node.tick()
        self.assertEqual(self.node.mode, 'locked')
        self.feed(); self.node.tick()
        self.assertEqual(self.node.mode, 'locked')
    def test_invalid_automatic_velocity_locks_and_never_forwards(self):
        self.node.mode = 'auto'; self.node.goal_state = 'executing'
        self.node.requirements = lambda: []; self.node.owner = 'test'
        import time
        self.node.lease_at = time.monotonic()
        self.node.velocity = Mock(); self.node.estop = Mock()
        for field, value in [('x', float('nan')), ('x', .9), ('y', .1)]:
            self.node.mode = 'auto'; self.node.goal_state = 'executing'
            msg = Twist(); setattr(msg.linear, field, value)
            self.node.on_auto(msg)
            self.assertEqual(self.node.mode, 'locked')
        for call in self.node.velocity.publish.call_args_list:
            msg = call.args[0]
            self.assertEqual(msg.linear.x, 0.)
            self.assertEqual(msg.angular.z, 0.)

    # ---- 保护停车 vs 人工急停：只有人工急停才允许锁存车端 /emergency_stop ----
    def test_sensor_dropout_is_protective_stop_without_hardware_estop(self):
        import time
        self.feed(); self.node.mode = 'manual'; self.node.owner = 'test'
        self.node.lease_at = self.node.manual_at = time.monotonic()
        self.node.velocity = Mock(); self.node.estop = Mock()
        bad = self.odom(); bad.twist.twist.linear.x = float('nan')
        self.node.on_odom(bad); self.node.tick()
        self.assertEqual(self.node.mode, 'locked')
        self.assertEqual(self.node.lock_kind, 'protective')
        self.assertFalse(self.node.estop.publish.called,
                         '数据断流属于保护停车，不得锁存车端硬件急停')

    def test_manual_estop_still_latches_hardware_estop(self):
        self.node.velocity = Mock(); self.node.estop = Mock()
        self.node.process({'op': 'estop', 'id': 1})
        self.assertEqual(self.node.mode, 'locked')
        self.assertEqual(self.node.lock_kind, 'estop')
        published = [c.args[0].data for c in self.node.estop.publish.call_args_list]
        self.assertIn(True, published, '人工急停必须向车端发 /emergency_stop=True')

    def test_scan_dropout_uses_configured_1_5s_window(self):
        import time
        self.feed()
        # 雷达整圈丢失 1.4 秒：仍在默认 1.5 秒安全窗口内。
        self.node.scan_health.accepted_at -= 1.4
        self.node.scan_health.received_at -= 1.4
        checks = self.node.structured_checks()
        self.assertTrue(checks['lidar']['ok'], '1.4s 整帧丢失不应判为雷达断流')
        self.node.scan_health.accepted_at -= .2   # 累计 1.6s
        self.node.scan_health.received_at -= .2
        self.assertFalse(self.node.structured_checks()['lidar']['ok'])
