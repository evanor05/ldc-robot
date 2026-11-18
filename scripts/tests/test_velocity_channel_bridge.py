import unittest
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import Twist
from scripts.navigation_bridge import Bridge


class ChannelBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.callback = Mock()
        with patch.object(Bridge, 'on_auto', self.callback):
            self.node = Bridge(journal_path=':memory:')
        self.responses = []
        self.node.respond = lambda request, **reply: self.responses.append(reply)

    def tearDown(self):
        self.node.destroy_node()

    def begin(self):
        self.node.process({'op': 'begin', 'mapPath': '/isolated/fixture', 'sessionId': 'channel-test'})
        return self.responses[-1]['velocityTopic']

    def test_bridge_does_not_subscribe_to_shared_legacy_velocity_input(self):
        self.assertFalse(any(s.topic_name == '/cmd_vel_auto' for s in self.node.subscriptions))

    def test_begin_rotates_channel_and_rejects_queued_old_callback(self):
        first = self.begin()
        old = self.node.velocity_channel.subscription.callback
        second = self.begin()
        self.assertNotEqual(first, second)
        for _ in range(20):
            old(Twist())
        self.callback.assert_not_called()
        self.node.velocity_channel.subscription.callback(Twist())
        self.callback.assert_called_once()
        self.assertEqual(self.node.status()['velocityChannel']['topic'], second)

    def test_stop_and_shutdown_revoke_channel(self):
        for op in ('stop', 'shutdown'):
            with self.subTest(op=op):
                self.begin()
                callback = self.node.velocity_channel.subscription.callback
                self.node.process({'op': op})
                callback(Twist())
                self.callback.assert_not_called()
                self.assertIsNone(self.node.velocity_channel.topic)
