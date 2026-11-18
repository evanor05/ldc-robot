import unittest
from unittest.mock import Mock
from rclpy.validate_full_topic_name import validate_full_topic_name

from scripts.velocity_channel import VelocityChannel


class Node:
    def __init__(self):
        self.readers = []
        self.destroyed = []

    def create_subscription(self, kind, topic, callback, qos):
        subscription = {'topic': topic, 'callback': callback, 'qos': qos}
        self.readers.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


class VelocityChannelTests(unittest.TestCase):
    def test_no_input_before_prepare_and_fresh_topic_each_run(self):
        node, callback = Node(), Mock()
        channel = VelocityChannel(node, callback)
        self.assertIsNone(channel.topic)
        self.assertEqual(node.readers, [])
        first = channel.rotate()
        second = channel.rotate()
        self.assertNotEqual(first, second)
        self.assertRegex(second, r'^/ldc/velocity/b[a-f0-9]{32}/r[a-f0-9]{32}$')
        validate_full_topic_name(second)
        self.assertEqual(len(node.destroyed), 1)
        self.assertEqual(node.readers[-1]['qos'].depth, 1)

    def test_queued_old_callback_cannot_enter_new_run_or_survive_close(self):
        node, callback = Node(), Mock()
        channel = VelocityChannel(node, callback)
        channel.rotate()
        old = node.readers[-1]['callback']
        old('first')
        channel.rotate()
        current = node.readers[-1]['callback']
        for _ in range(20):
            old('replayed')
        current('current')
        channel.close()
        current('after-stop')
        self.assertEqual([c.args[0] for c in callback.call_args_list], ['first', 'current'])
        self.assertIsNone(channel.snapshot()['epoch'])

    def test_restarted_owner_has_a_different_topic_even_for_its_first_run(self):
        first, second = VelocityChannel(Node(), Mock()), VelocityChannel(Node(), Mock())
        self.assertNotEqual(first.rotate(), second.rotate())
        self.assertNotEqual(first.boot_id, second.boot_id)

    def test_reader_creation_failure_keeps_old_callbacks_revoked(self):
        node, callback = Node(), Mock()
        channel = VelocityChannel(node, callback)
        channel.rotate()
        old = node.readers[-1]['callback']
        node.create_subscription = Mock(side_effect=RuntimeError('DDS failure'))
        with self.assertRaises(RuntimeError):
            channel.rotate()
        old('old')
        callback.assert_not_called()
        self.assertIsNone(channel.topic)
