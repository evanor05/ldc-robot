import concurrent.futures
import importlib.util
from pathlib import Path
import types
import time
import unittest
import rclpy
from nav_msgs.msg import OccupancyGrid

spec = importlib.util.spec_from_file_location('scripts.navigation_bridge', Path(__file__).resolve().parents[1] / 'navigation_bridge.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


def free_map(width=200, height=200, resolution=.05, origin_x=-5., origin_y=-5.):
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = width, height
    msg.info.resolution = resolution
    msg.info.origin.position.x, msg.info.origin.position.y = origin_x, origin_y
    msg.info.origin.orientation.w = 1.
    msg.data = [0] * (width * height)
    return msg


def set_map_value(msg, x, y, value):
    info = msg.info
    mx = int((x - info.origin.position.x) / info.resolution)
    my = int((y - info.origin.position.y) / info.resolution)
    msg.data[my * info.width + mx] = value

class Handle:
    accepted = True
    def __init__(self): self.result = concurrent.futures.Future(); self.canceled = False
    def get_result_async(self): return self.result
    def cancel_goal_async(self): self.canceled = True

class Client:
    def __init__(self): self.sent = []
    def send_goal_async(self, goal, feedback_callback):
        pending = concurrent.futures.Future(); self.sent.append((pending, feedback_callback)); return pending
    def server_is_ready(self): return True

class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()
    SESSION = 'unit-test-session'
    def setUp(self):
        self.node = module.Bridge(); self.node.action.destroy()
        self.node.action = Client(); self.node.requirements = lambda: []; self.node.sensors = lambda: []
        self.node.running = True; self.replies = []
        self.node.respond = lambda request, **data: self.replies.append((request, data))
        self.node.on_map(free_map())
        # 第二阶段：运动/目标操作绑定控制会话，测试页面先取得控制权。
        self.node.process({'op': 'claim', 'id': 0, 'sessionId': self.SESSION})
        self.replies.clear()
    def tearDown(self): self.node.destroy_node()
    def submit(self, points=None):
        self.node.process({'op':'goals', 'id':1, 'sessionId': self.SESSION, 'points': points or [{'x':1, 'y':0}]})
        return self.node.action.sent[-1][0]
    def result(self, handle, status=4, code=0):
        handle.result.set_result(types.SimpleNamespace(status=status, result=types.SimpleNamespace(error_code=code,error_msg='aborted' if code else '')))
    def test_goal_heading_is_encoded_in_action_quaternion(self):
        import math
        captured = []
        original = self.node.action.send_goal_async
        def send(goal, feedback_callback):
            captured.append(goal)
            return original(goal, feedback_callback)
        self.node.action.send_goal_async = send
        self.submit([{'x': 1.2, 'y': -.4, 'yaw': math.pi / 2}])
        goal = captured[0].pose
        self.assertEqual(goal.header.frame_id, 'map')
        self.assertAlmostEqual(goal.pose.position.x, 1.2)
        self.assertAlmostEqual(goal.pose.orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(goal.pose.orientation.w, math.cos(math.pi / 4))

    def test_rejection_propagates_and_clears_queue(self):
        pending = self.submit([{'x':1,'y':0},{'x':2,'y':0}])
        handle = Handle(); handle.accepted = False; pending.set_result(handle)
        self.assertEqual(self.node.goal_state,'failed'); self.assertEqual(self.node.points,[])
        self.assertFalse(self.replies[-1][1]['success'])
    def test_distance_feedback_never_advances_and_old_result_ignored(self):
        pending = self.submit([{'x':1,'y':0},{'x':2,'y':0}]); handle=Handle(); pending.set_result(handle)
        old_id = self.node.goal_id
        self.node.action.sent[0][1](types.SimpleNamespace(feedback=types.SimpleNamespace(distance_remaining=.001,estimated_time_remaining=types.SimpleNamespace(sec=0,nanosec=0))))
        self.assertEqual(len(self.node.action.sent),1)
        self.result(handle); self.assertEqual(len(self.node.action.sent),2)
        new_id=self.node.goal_id
        self.node.completed(concurrent.futures.Future(),old_id)
        self.assertEqual(self.node.goal_id,new_id)
    def test_cancel_before_acceptance_waits_for_terminal(self):
        pending=self.submit(); self.node.cancel({'id':2})
        self.assertFalse(self.replies)
        handle=Handle(); pending.set_result(handle)
        self.assertTrue(handle.canceled); self.assertEqual(self.node.goal_state,'canceling')
        self.assertFalse(any(r.get('id')==2 for r,d in self.replies))
        self.result(handle,status=5)
        self.assertEqual(self.node.goal_state,'canceled'); self.assertEqual(self.node.mode,'locked')
        self.assertTrue(any(r.get('id')==2 and d['success'] for r,d in self.replies))
    def test_abort_never_advances_queue(self):
        pending=self.submit([{'x':1,'y':0},{'x':2,'y':0}]); handle=Handle(); pending.set_result(handle)
        self.result(handle,status=6,code=1)
        self.assertEqual(len(self.node.action.sent),1); self.assertEqual(self.node.points,[])
        self.assertEqual(self.node.mode,'locked')
    def test_lost_transition_reply_uses_observed_state_before_next_step(self):
        calls = []
        class TransitionClient:
            def service_is_ready(self): return True
            def call_async(self, request):
                calls.append(request.transition.id)
                return concurrent.futures.Future()
            def remove_pending_request(self, future): pass
        self.node.transition_clients = {name: TransitionClient() for name in module.NODES}
        self.node.nav_started = time.monotonic()
        self.node.lifecycle['map_server'] = (1, time.monotonic())
        self.node.start_localized_navigation()
        self.assertEqual(calls, [1])
        # The response was lost, but GetState confirms configure completed.
        self.node.lifecycle['map_server'] = (2, time.monotonic())
        self.node.start_localized_navigation()
        self.assertEqual(calls, [1, 3])
        self.node.lifecycle = {name: (3, time.monotonic()) for name in module.NODES[:2]}
        self.node.start_localized_navigation()
        self.assertFalse(self.node.startup_done)  # no navigation before user pose
    def test_startup_timeout_excludes_human_pose_wait(self):
        self.node.nav_started = time.monotonic() - 100
        self.node.lifecycle = {name: (3, time.monotonic()) for name in module.NODES[:2]}
        self.node.start_localized_navigation()
        self.assertFalse(self.node.startup_failed)
        self.node.lifecycle = {}
        self.node.start_localized_navigation()
        self.assertTrue(self.node.startup_failed)
        self.assertEqual(self.node.mode, 'locked')

    def test_invalid_coordinates_and_manual_session(self):
        with self.assertRaises(ValueError): module.valid_points([{'x':float('nan'),'y':0}])
        self.node.process({'op':'manual_enable','id':1,'sessionId':self.SESSION}); token=self.node.control_id
        self.node.process({'op':'velocity','sessionId':self.SESSION,'controlId':token,'sequence':2,'linearX':.1,'angularZ':0})
        with self.assertRaises(ValueError): self.node.process({'op':'velocity','sessionId':self.SESSION,'controlId':token,'sequence':1,'linearX':.1,'angularZ':0})
        self.node.lock()
        with self.assertRaises(ValueError): self.node.process({'op':'velocity','sessionId':self.SESSION,'controlId':token,'sequence':3,'linearX':.1,'angularZ':0})

    def assert_goal_rejected(self, points, token):
        with self.assertRaises(ValueError):
            self.node.process({'op': 'goals', 'id': 90, 'sessionId': self.SESSION, 'points': points})
        self.assertEqual(self.node.action.sent, [])
        self.assertTrue(any(e['type'] == 'goal_rejected' and token in e['reason'] for e in self.node.events))

    def test_unknown_occupied_and_out_of_bounds_goals_are_rejected_before_action(self):
        grid = free_map()
        set_map_value(grid, 1., 0., -1)       # unknown
        set_map_value(grid, 2., 0., 100)      # occupied
        self.node.on_map(grid)
        self.assert_goal_rejected([{'x': 1., 'y': 0.}], '未知区域')
        self.assert_goal_rejected([{'x': 2., 'y': 0.}], '障碍区域')
        self.assert_goal_rejected([{'x': 9., 'y': 0.}], '超出地图范围')
        self.assertEqual(self.node.mode, 'locked')

    def test_goals_never_publish_estop_false(self):
        from unittest.mock import Mock
        self.node.estop = Mock()
        self.submit()
        self.assertFalse(self.node.estop.publish.called,
                         'goals 不得隐式清除人工急停锁存')

    def test_estop_blocks_goals_until_explicit_reset(self):
        from unittest.mock import Mock
        self.node.estop = Mock()
        self.node.process({'op': 'estop', 'id': 91, 'sessionId': self.SESSION})
        self.assertEqual(self.node.lock_kind, 'estop')
        with self.assertRaises(ValueError):
            self.node.process({'op': 'goals', 'id': 92, 'sessionId': self.SESSION,
                               'points': [{'x': 1., 'y': 0.}]})
        self.assertEqual(self.node.action.sent, [])
        self.node.process({'op': 'reset_estop', 'id': 93, 'sessionId': self.SESSION})
        self.assertEqual(self.node.lock_kind, 'idle')
        self.submit()

    def test_cancel_with_unresolved_acceptance_future_reaches_terminal(self):
        self.node.startup_done = True
        pending = self.submit()
        self.assertFalse(pending.done())
        cancel = {'op': 'cancel', 'id': 94, 'sessionId': self.SESSION}
        self.node.cancel(cancel)
        self.assertEqual(self.node.goal_state, 'canceling')
        self.node.cancel_waiters[0] = (cancel, time.monotonic() - 1, None)
        self.node.cancel_deadline = time.monotonic() - 1
        while not self.node.commands.empty():
            self.node.commands.get_nowait()
        self.node.tick()
        self.assertEqual(self.node.goal_state, 'canceled')
        self.assertIsNone(self.node.goal_id)
        self.assertTrue(any(e['type'] == 'cancel_timeout_terminal' for e in self.node.events))

if __name__ == '__main__': unittest.main()
