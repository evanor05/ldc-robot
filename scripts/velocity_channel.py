"""A fresh ROS velocity input for each prepared Nav2 run.

The ROS owner calls rotate/close on its executor thread. Topic identity rejects
old producers; the callback generation also rejects already queued old samples.
This is run isolation, not an epoch stamped onto an ordinary Twist.
"""
import uuid

from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class VelocityChannel:
    def __init__(self, node, callback, boot_id=None):
        self.node = node
        self.callback = callback
        # bootId 单源（裁决 Q1）：Bridge 在拿域级锁前生成一次并注入，
        # 避免锁身份与速度通道身份出现两个 ID。缺省仍自生成，向后兼容。
        if boot_id is not None:
            if (not isinstance(boot_id, str) or len(boot_id) != 32
                    or any(c not in '0123456789abcdef' for c in boot_id)):
                raise ValueError('boot_id 必须是 32 位小写十六进制（uuid4().hex）')
        self.boot_id = boot_id if boot_id is not None else uuid.uuid4().hex
        self.epoch = None
        self.topic = None
        self.subscription = None

    def rotate(self):
        self.close()
        epoch = uuid.uuid4().hex
        topic = f'/ldc/velocity/b{self.boot_id}/r{epoch}'
        self.epoch, self.topic = epoch, topic

        def receive(message):
            if self.epoch == epoch:
                self.callback(message)

        try:
            self.subscription = self.node.create_subscription(
                Twist, topic, receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
        except Exception:
            self.epoch = self.topic = None
            raise
        return topic

    def close(self):
        # Invalidate callbacks before destroying their DDS reader.
        self.epoch = self.topic = None
        subscription, self.subscription = self.subscription, None
        if subscription is not None:
            self.node.destroy_subscription(subscription)

    def snapshot(self):
        return {'bootId': self.boot_id, 'epoch': self.epoch, 'topic': self.topic}
