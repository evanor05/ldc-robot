#!/usr/bin/env python3
"""Deterministic 2D ray-cast robot. Run only in the isolated console domain."""
import math
import os
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class Robot(Node):
    def __init__(self):
        super().__init__('offline_console_robot')
        self.x = self.y = self.yaw = 0.
        self.v = self.w = self.command_at = 0.
        self.stopped = True
        self.sensors_enabled = True
        self.replay_samples = False
        self.cached_odom = self.cached_scan = None
        signal.signal(signal.SIGUSR2, lambda *_: setattr(self, 'replay_samples', not self.replay_samples))
        signal.signal(signal.SIGUSR1, lambda *_: setattr(self, 'sensors_enabled', not self.sensors_enabled))
        self.last = time.monotonic()
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, x, z in [('base_footprint', 'base_link', 0., .08), ('base_link', 'laser_frame', .05, .07)]:
            t = TransformStamped(); t.header.frame_id = parent; t.child_frame_id = child
            t.transform.translation.x = x; t.transform.translation.z = z; t.transform.rotation.w = 1.
            transforms.append(t)
        self.static.sendTransform(transforms)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency, 10)
        self.create_timer(.05, self.tick)

    def emergency(self, msg):
        self.stopped = msg.data
        if msg.data: self.v = self.w = 0.

    def command(self, msg):
        self.command_at = time.monotonic()
        self.v, self.w = msg.linear.x, msg.angular.z

    def ray(self, x, y, angle):
        dx, dy = math.cos(angle), math.sin(angle)
        hits = []
        for wall, horizontal in [(-2.5, False), (3.5, False), (-2., True), (2.5, True)]:
            divisor = dy if horizontal else dx
            if abs(divisor) > 1e-8:
                distance = (wall - (y if horizontal else x)) / divisor
                if distance > 0: hits.append(distance)
        # Asymmetric pillar to make localization observable.
        for wall, horizontal in [(2., False), (2.4, False), (1.2, True), (1.7, True)]:
            divisor = dy if horizontal else dx
            if abs(divisor) < 1e-8: continue
            distance = (wall - (y if horizontal else x)) / divisor
            other = (x + distance * dx) if horizontal else (y + distance * dy)
            low, high = (2., 2.4) if horizontal else (1.2, 1.7)
            if distance > 0 and low <= other <= high: hits.append(distance)
        return min(hits + [8.])

    def tick(self):
        now = time.monotonic(); dt = min(.1, now - self.last); self.last = now
        v, w = (0., 0.) if self.stopped or now - self.command_at > .4 else (self.v, self.w)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        stamp = self.get_clock().now().to_msg()
        odom = Odometry(); odom.header.frame_id = 'odom'; odom.header.stamp = stamp
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2); odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom.twist.twist.linear.x = v; odom.twist.twist.angular.z = w
        if not self.sensors_enabled: return
        if not self.replay_samples or self.cached_odom is None:
            self.cached_odom = odom
        self.odom.publish(self.cached_odom)
        t = TransformStamped(); t.header = odom.header; t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x; t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation; self.tf.sendTransform(t)
        scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi; scan.angle_increment = math.pi / 180
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min = .08; scan.range_max = 8.; scan.scan_time = .05
        lx, ly = self.x + .05 * math.cos(self.yaw), self.y + .05 * math.sin(self.yaw)
        scan.ranges = [self.ray(lx, ly, self.yaw + scan.angle_min + i * scan.angle_increment) for i in range(360)]
        if not self.replay_samples or self.cached_scan is None:
            self.cached_scan = scan
        self.scan.publish(self.cached_scan)


def main():
    if os.environ.get('ROBOT_SIMULATION') != '1' or os.environ.get('ROS_DOMAIN_ID') != '197':
        raise SystemExit('Use offline_console.sh; isolated domain 197 required')
    rclpy.init(); node = Robot()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception:
        if rclpy.ok(): raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
