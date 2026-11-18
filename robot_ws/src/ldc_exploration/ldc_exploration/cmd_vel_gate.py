"""Legacy intermediate gate; never owns the vehicle's final /cmd_vel output."""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .gate_core import GateDecision, decide_velocity


class CmdVelGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_gate')
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_linear', 0.15)
        self.declare_parameter('min_linear', 0.0)
        self.declare_parameter('max_angular', 0.8)

        def value(name):
            return self.get_parameter(name).value

        self.scan_timeout = float(value('scan_timeout'))
        self.command_timeout = float(value('command_timeout'))
        self.max_linear = float(value('max_linear'))
        self.min_linear = float(value('min_linear'))
        self.max_angular = float(value('max_angular'))

        self.cmd_pub = self.create_publisher(Twist, '/exploration/cmd_vel_guarded', 10)
        self.status_pub = self.create_publisher(String, '/exploration/gate_status', 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.on_auto, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)
        self.create_subscription(Bool, '/exploration/enable', self.on_enable, 10)

        self.enabled = False
        self.estop = False
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_at = None
        self.scan_stamp_ns = None
        self.scan_at = None
        self.last_decision = None
        self.create_timer(0.05, self.on_timer)
        self.create_timer(0.25, self.publish_status)

    def on_auto(self, msg: Twist) -> None:
        if not self.enabled:
            return
        self.last_cmd_linear = float(msg.linear.x)
        self.last_cmd_angular = float(msg.angular.z)
        self.last_cmd_at = time.monotonic()
        self.step(time.monotonic())

    def on_scan(self, msg: LaserScan) -> None:
        stamp = msg.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            return
        self.scan_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self.scan_at = time.monotonic()

    def on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def on_enable(self, msg: Bool) -> None:
        # Old Bool commands may revoke authority, never grant it. Authority is
        # reserved for the Bridge-owned path, not this legacy subscription.
        if msg.data:
            return
        self.enabled = False
        self.last_decision = GateDecision(False, 0.0, 0.0, 'disabled')
        self.publish_decision(self.last_decision)
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_at = None

    def scan_age(self) -> float | None:
        if self.scan_stamp_ns is None:
            return None
        age = (self.get_clock().now().nanoseconds - self.scan_stamp_ns) / 1e9
        # Reject a badly future-dated scan (clock mismatch) rather than treating
        # it as fresh; a small DDS/processing future tolerance is accepted.
        if age < -0.25:
            return None
        return max(0.0, age)

    def step(self, now: float) -> None:
        if not self.enabled:
            return
        command_age = None if self.last_cmd_at is None else now - self.last_cmd_at
        decision = decide_velocity(
            self.last_cmd_linear, self.last_cmd_angular, self.scan_age(),
            self.estop, command_age,
            min_linear=self.min_linear, max_linear=self.max_linear,
            max_angular=self.max_angular, scan_timeout=self.scan_timeout,
            command_timeout=self.command_timeout)
        self.last_decision = decision
        self.publish_decision(decision)

    def publish_decision(self, decision) -> None:
        twist = Twist()
        twist.linear.x = decision.linear
        twist.angular.z = decision.angular
        self.cmd_pub.publish(twist)

    def on_timer(self) -> None:
        self.step(time.monotonic())

    def publish_status(self) -> None:
        now = time.monotonic()
        decision = self.last_decision
        payload = {
            'ok': bool(decision.ok) if decision and self.enabled else False,
            'enabled': bool(self.enabled),
            'reason': (decision.reason if decision else 'no_command') if self.enabled else 'disabled',
            'scan_age': None if self.scan_age() is None else round(self.scan_age(), 3),
            'estop': bool(self.estop),
            'last_cmd_age': None if self.last_cmd_at is None else round(now - self.last_cmd_at, 3),
            'linear': round(decision.linear, 3) if decision else 0.0,
            'angular': round(decision.angular, 3) if decision else 0.0,
            'scan_timeout': self.scan_timeout,
            'max_linear': self.max_linear,
            'max_angular': self.max_angular,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
