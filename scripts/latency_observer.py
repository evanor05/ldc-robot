#!/usr/bin/env python3
"""D 工单独立零速观察者：独立进程订阅 /cmd_vel，记录每条消息的单调到达时刻。

与 Bridge 无共享内存/无共享时钟对象；time.monotonic() 为同机 CLOCK_MONOTONIC，
与探针/编排器时间戳直接可比。四时刻中的 t3 = 停止请求(t0)之后首条零速的到达时刻，
由编排器在本输出上判定（先建立非零运动，避免把之前的周期零速计入）。
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Observer(Node):
    def __init__(self, out_path):
        super().__init__('latency_observer')
        self._handle = open(out_path, 'a', buffering=1, encoding='utf-8')
        # QoS 与 Bridge 的 /cmd_vel 发布者一致（depth=10，默认 reliable/volatile）。
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)

    def _write(self, record):
        try:
            self._handle.write(json.dumps(record, ensure_ascii=False) + '\n')
        except (OSError, ValueError):
            pass

    def on_cmd(self, msg):
        at = time.monotonic()
        self._write({'topic': '/cmd_vel', 'mono': at, 'wall': time.time(),
                     'lx': msg.linear.x, 'ly': msg.linear.y, 'lz': msg.linear.z,
                     'ax': msg.angular.x, 'ay': msg.angular.y, 'az': msg.angular.z})

    def on_estop(self, msg):
        at = time.monotonic()
        self._write({'topic': '/emergency_stop', 'mono': at, 'wall': time.time(),
                     'data': bool(msg.data)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Observer(args.out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()