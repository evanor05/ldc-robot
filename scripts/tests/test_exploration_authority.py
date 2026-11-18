"""Exploration must stay passive at startup and revoke late accepted goals."""
import concurrent.futures
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import rclpy
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_ws/src/ldc_exploration'))
from ldc_exploration.frontier_explorer import FrontierExplorer
from ldc_exploration.cmd_vel_gate import CmdVelGate


class ExplorationAuthorityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_frontier_startup_does_not_authorize_motion(self):
        node = FrontierExplorer()
        try:
            self.assertFalse(node.enabled, 'Starting a process must not authorize exploration')
            node.on_enable(Bool(data=True))
            node.on_stop(Bool(data=False))
            self.assertFalse(node.enabled, 'Legacy Bool commands may not grant authority')
            node.bootstrap_twist_pub = Mock()
            node.bootstrap_active = True
            node.bootstrap_tick()
            node.bootstrap_twist_pub.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_standalone_gate_startup_does_not_authorize_motion(self):
        node = CmdVelGate()
        try:
            self.assertFalse(node.enabled, 'A gate must start locked, including direct ros2 launch')
            node.on_enable(Bool(data=True))
            self.assertFalse(node.enabled)
            self.assertNotEqual(node.cmd_pub.topic_name, '/cmd_vel')
        finally:
            node.destroy_node()

    def test_canceled_frontier_revokes_late_accepted_action(self):
        node = FrontierExplorer()
        try:
            old_token = node.goal_token
            node.cancel_current()
            handle = Mock(accepted=True)
            pending = concurrent.futures.Future()
            pending.set_result(handle)
            node.on_goal_accepted(pending, old_token)
            handle.cancel_goal_async.assert_called_once()
            self.assertIsNone(node.goal_handle, 'A revoked callback cannot reattach its goal')
        finally:
            node.destroy_node()
