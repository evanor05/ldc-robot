import copy
import math
import unittest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scripts.localization_health import LocalizationHealth


def pose(seconds=10., x=0., yaw=0.):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = round((seconds-int(seconds))*1e9)
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.z = math.sin(yaw/2)
    msg.pose.pose.orientation.w = math.cos(yaw/2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = .01
    msg.pose.covariance[35] = .01
    return msg


def odom(seconds=10., x=0., yaw=0.):
    source = pose(seconds, x, yaw)
    msg = Odometry()
    msg.header = source.header
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.pose = source.pose
    return msg


class LocalizationTests(unittest.TestCase):
    def setUp(self):
        self.health = LocalizationHealth()
        self.health.reset(10_000_000_000)
        self.health.observe_odometry(odom(), 10.)
        self.assertTrue(self.health.observe(pose(), 10_000_000_000, 10.))

    def test_stationary_amcl_is_not_expired_by_wall_time(self):
        self.health.observe_odometry(odom(200.), 200.)
        self.assertTrue(self.health.check(200_000_000_000, 200.)['ok'])
        self.assertEqual(self.health.check(200_000_000_000, 200.)['code'], 'stationary')

    def test_motion_requires_update_and_stale_duplicates_cannot_renew(self):
        self.health.observe_odometry(odom(11., .3), 11.)
        self.assertFalse(self.health.observe(pose(), 11_000_000_000, 11.))
        self.assertTrue(self.health.check(12_000_000_000, 12.)['ok'])
        self.assertEqual(self.health.check(13_000_000_000, 13.)['code'], 'motion_stale')
        self.health.observe_odometry(odom(13., .3), 13.)
        self.assertTrue(self.health.observe(pose(13., .3), 13_000_000_000, 13.))

    def test_rotation_also_requires_update(self):
        self.health.observe_odometry(odom(11., yaw=.3), 11.)
        self.assertFalse(self.health.check(13_000_000_000, 13.)['ok'])

    def test_diagonal_motion_below_each_amcl_axis_threshold_can_pause(self):
        for x, y in ((.2, .2), (-.2, .2), (.2, -.2), (-.2, -.2)):
            with self.subTest(x=x, y=y):
                self.setUp()
                sample = odom(11., x=x)
                sample.pose.pose.position.y = y
                self.health.observe_odometry(sample, 11.)
                self.assertTrue(self.health.check(20_000_000_000, 20.)['ok'])
                self.assertIsNone(self.health.pending_motion_at)

    def test_amcl_axis_threshold_is_strict_and_exceeding_either_axis_expires(self):
        for axis in ('x', 'y'):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    self.setUp()
                    sample = odom(11.)
                    setattr(sample.pose.pose.position, axis, sign*.25)
                    self.health.observe_odometry(sample, 11.)
                    self.assertTrue(self.health.check(13_000_000_000, 13.)['ok'])
                    sample.header.stamp.sec = 14
                    setattr(sample.pose.pose.position, axis, sign*.251)
                    self.health.observe_odometry(sample, 14.)
                    self.assertEqual(self.health.check(16_000_000_000, 16.)['code'], 'motion_stale')

    def test_amcl_angle_threshold_is_strict(self):
        from scripts.localization_health import angle, planar_pose
        sample = odom(11., yaw=.2)
        self.health.update_angle = abs(angle(planar_pose(sample.pose.pose)[2]))
        self.health.observe_odometry(sample, 11.)
        self.assertTrue(self.health.check(13_000_000_000, 13.)['ok'])
        self.health.observe_odometry(odom(14., yaw=.201), 14.)
        self.assertEqual(self.health.check(16_000_000_000, 16.)['code'], 'motion_stale')

    def test_delayed_pose_rechecks_newest_odometry_with_same_axis_rule(self):
        for x, y, expected in ((.2, .2, True), (.25, .0, True), (.0, .251, False)):
            with self.subTest(x=x, y=y):
                self.setUp()
                sample = odom(10.2, x=x)
                sample.pose.pose.position.y = y
                self.health.observe_odometry(sample, 10.2)
                # This AMCL scan aligns with the earlier odom at (0, 0).
                self.assertTrue(self.health.observe(pose(10.01), 10_200_000_000, 10.2))
                self.assertEqual(self.health.check(13_000_000_000, 13.)['ok'], expected)

    def test_old_session_pose_cannot_confirm_initialization(self):
        self.health.reset(11_000_000_000)
        self.assertFalse(self.health.observe(pose(), 11_000_000_000, 11.))
        self.assertFalse(self.health.check(11_000_000_000, 11.)['ok'])

    def test_invalid_pose_covariance_and_stamp_rejected(self):
        for mutate in [lambda m: setattr(m.header, 'frame_id', 'odom'),
                       lambda m: setattr(m.pose.pose.position, 'x', float('nan')),
                       lambda m: setattr(m.pose.pose.orientation, 'w', 2.),
                       lambda m: m.pose.covariance.__setitem__(0, -1.),
                       lambda m: m.pose.covariance.__setitem__(1, 1.),
                       lambda m: m.pose.covariance.__setitem__(35, 0.),
                       lambda m: setattr(m.header.stamp, 'sec', 100)]:
            self.health.reset(10_000_000_000)
            msg = pose(10.1)
            mutate(msg)
            self.assertFalse(self.health.observe(msg, 10_100_000_000, 10.1))
            self.assertFalse(self.health.check(10_100_000_000, 10.1)['ok'])

    def test_jump_latches_until_new_initial_pose(self):
        self.health.observe_odometry(odom(11.), 11.)
        self.assertFalse(self.health.observe(pose(11., 2.), 11_000_000_000, 11.))
        self.assertTrue(self.health.check(11_000_000_000, 11.)['requiresInitialPose'])
        self.assertFalse(self.health.observe(pose(11., 0.), 11_000_000_000, 11.))
        self.health.reset(11_000_000_000)
        self.assertTrue(self.health.observe(pose(11., 2.), 11_000_000_000, 11.))

    def test_odom_motion_is_removed_before_jump_check(self):
        self.health.observe_odometry(odom(13., 1.), 13.)
        self.assertTrue(self.health.observe(pose(13., 1.), 13_000_000_000, 13.))

    def test_unpaired_and_uncertain_measurements_do_not_confirm(self):
        self.assertFalse(self.health.observe(pose(11.), 11_000_000_000, 11.))
        self.health.observe_odometry(odom(11.), 11.)
        msg = pose(11.)
        msg.pose.covariance[0] = 1.
        self.assertFalse(self.health.observe(msg, 11_000_000_000, 11.))

    def test_clock_rewind_requires_reinitialization(self):
        self.assertFalse(self.health.check(9_000_000_000, 11.)['ok'])
        self.assertTrue(self.health.check(10_000_000_000, 12.)['requiresInitialPose'])


class BridgeLocalizationTests(unittest.TestCase):
    def test_localization_fault_cancels_active_goal_and_recovery_does_not_resume(self):
        import queue
        import time
        import rclpy
        from scripts.navigation_bridge import Bridge
        rclpy.init()
        node = Bridge()
        try:
            node.commands = queue.Queue()
            node.start_localized_navigation = lambda: None
            node.sensors = lambda: []
            node.fresh_transform = lambda *_: True
            node.running = True
            node.owner = 'localization-test'
            node.lease_at = time.monotonic()
            node.pose_requested = node.lease_at-.1
            now = node.get_clock().now().nanoseconds
            node.localization_health.reset(now-100_000_000)
            initial_odom = odom(now/1e9)
            node.on_odom(initial_odom)
            node.on_pose(pose(now/1e9))
            self.assertTrue(node.localization_check()['ok'])
            canceled = []
            class Goal:
                def cancel_goal_async(self): canceled.append(True)
            node.goal_handle = Goal()
            node.goal_state = 'executing'
            node.mode = 'auto'
            node.requirements = lambda: [] if node.localization_check()['ok'] else [node.localization_check()['detail']]
            invalid = pose(now/1e9+.01)
            invalid.pose.pose.position.x = float('nan')
            node.on_pose(invalid)
            node.tick()
            self.assertEqual(node.mode, 'locked')
            self.assertEqual(node.goal_state, 'canceling')
            self.assertEqual(canceled, [True])
            self.assertIn('非有限数', node.last_stop['reason'])
            node.on_pose(pose(now/1e9+.02))
            node.tick()
            self.assertTrue(node.localization_check()['ok'])
            self.assertEqual(node.mode, 'locked')
            self.assertEqual(node.points, [])
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
