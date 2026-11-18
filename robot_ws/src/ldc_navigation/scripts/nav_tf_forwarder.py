#!/usr/bin/env python3
"""每轮 Nav2 的公共→私有 TF 转发器（裁决 Q3：launch 拥有的独立进程，生命周期跟随轮次）。

公共 /tf、/tf_static → 私有 NS/tf、NS/tf_static，逐 Transform 精确 pair 筛选：
动态仅 odom→base_footprint；静态仅 base_footprint→base_link、base_link→laser_frame。
map 父帧硬拒绝（旧 AMCL/GMapping 的 map→odom 无法混入本轮）；保持原 stamp/frame/数据，
不刷新重播陈旧动态变换。公共订阅用绝对 /tf、/tf_static 防 remap 回环；static 两端
TRANSIENT_LOCAL，缓存允许静态边每次合并完整发布（覆盖晚加入订阅者与分批到达）。
转发器退出由 launch OnProcessExit 触发整轮 Shutdown（Bridge 侧 TF 断供→撤权零速）。
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

DYNAMIC_DEFAULT = ['odom:base_footprint']
STATIC_DEFAULT = ['base_footprint:base_link', 'base_link:laser_frame']


def parse_pairs(items):
    pairs = set()
    for item in items:
        parent, _, child = item.partition(':')
        if parent and child:
            pairs.add((parent, child))
    return pairs


class TfForwarder(Node):
    def __init__(self, namespace, dynamic_pairs, static_pairs, static_period=10.0):
        super().__init__('nav_tf_forwarder', namespace=namespace)
        self.dynamic_pairs = dynamic_pairs
        self.static_pairs = static_pairs
        self.static_cache = {}
        dynamic_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_pub = self.create_publisher(TFMessage, 'tf', dynamic_qos)
        self.static_pub = self.create_publisher(TFMessage, 'tf_static', static_qos)
        self.create_subscription(TFMessage, '/tf', self.on_tf, dynamic_qos)
        self.create_subscription(TFMessage, '/tf_static', self.on_static, static_qos)
        self.create_timer(static_period, self.rebroadcast_static)
        self.forwarded_dynamic = 0
        self.forwarded_static = 0
        self.blocked_map = 0
        self.get_logger().info(
            f'TF forwarder: public -> {namespace}/tf|tf_static; '
            f'dynamic={sorted(dynamic_pairs)} static={sorted(static_pairs)}')

    def _filter(self, msg, pairs):
        kept = []
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            if parent == 'map':
                self.blocked_map += 1
                continue  # 硬拒绝：任何 map 父帧都不进私有总线
            if (parent, child) in pairs:
                kept.append(transform)
        return kept

    def on_tf(self, msg):
        kept = self._filter(msg, self.dynamic_pairs)
        if kept:
            out = TFMessage()
            out.transforms = kept
            self.tf_pub.publish(out)
            self.forwarded_dynamic += len(kept)

    def on_static(self, msg):
        kept = self._filter(msg, self.static_pairs)
        if not kept:
            return
        for transform in kept:
            key = (transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/'))
            self.static_cache[key] = transform
        # Q3.4：每次合并完整缓存发布（两条允许静态边可能分批到达；
        # 晚加入订阅者靠 TRANSIENT_LOCAL latched 拿到全集，不 depth1 逐条）。
        out = TFMessage()
        out.transforms = list(self.static_cache.values())
        self.static_pub.publish(out)
        self.forwarded_static += len(kept)

    def rebroadcast_static(self):
        if not self.static_cache:
            return
        out = TFMessage()
        out.transforms = list(self.static_cache.values())
        self.static_pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--dynamic', nargs='*', default=DYNAMIC_DEFAULT)
    parser.add_argument('--static', nargs='*', default=STATIC_DEFAULT)
    parser.add_argument('--static-period', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args or None)
    node = TfForwarder(args.namespace, parse_pairs(args.dynamic),
                       parse_pairs(args.static), args.static_period)
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