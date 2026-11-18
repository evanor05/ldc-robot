"""D：无位移失败在导航桥里的端到端行为（离线，不连车）。

验证三件事：
1. 贴墙式失败（目标期间几乎没动）→ 清空队列、零速锁定、事件带原因、清一次 costmap；
2. 连续两次同样失败 → stuck 锁存，之后发目标被明确拒绝（不再循环喂目标）；
3. 有位移的失败 → 走原来的普通失败路径，不触发停机策略，也不清 costmap。
"""
import concurrent.futures
import importlib.util
import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Odometry

spec = importlib.util.spec_from_file_location(
    'scripts.navigation_bridge', Path(__file__).resolve().parents[1] / 'navigation_bridge.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

SESSION = 'unit-test-session'


def free_map(width=200, height=200, resolution=.05, origin_x=-5., origin_y=-5.):
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = width, height
    msg.info.resolution = resolution
    msg.info.origin.position.x, msg.info.origin.position.y = origin_x, origin_y
    msg.info.origin.orientation.w = 1.
    msg.data = [0] * (width * height)
    return msg


class Handle:
    accepted = True

    def __init__(self):
        self.result = concurrent.futures.Future()
        self.canceled = False

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.canceled = True


class Client:
    def __init__(self):
        self.sent = []

    def send_goal_async(self, goal, feedback_callback):
        pending = concurrent.futures.Future()
        self.sent.append((pending, feedback_callback))
        return pending

    def server_is_ready(self):
        return True


class StuckPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = module.Bridge()
        self.node.action.destroy()
        self.node.action = Client()
        self.node.requirements = lambda: []
        self.node.sensors = lambda: []
        self.node.running = True
        self.node.respond = lambda request, **data: self.replies.append((request, data))
        self.replies = []
        self.node.on_map(free_map())
        self.node.process({'op': 'claim', 'id': 0, 'sessionId': SESSION})
        self.replies.clear()
        self.clears = []
        self.node.clear_costmaps_once = lambda: (self.clears.append(1), True)[1]

    def tearDown(self):
        self.node.destroy_node()

    # ---- helpers ----
    def submit(self, points=None):
        self.node.process({'op': 'goals', 'id': 1, 'sessionId': SESSION,
                           'points': points or [{'x': 1., 'y': 0.}, {'x': 2., 'y': 0.}]})
        return self.node.action.sent[-1][0]

    def fail(self, pending, status=GoalStatus.STATUS_ABORTED, code=0,
             message='No valid trajectories (BaseObstacle 100%)'):
        handle = Handle()
        pending.set_result(handle)
        handle.result.set_result(types.SimpleNamespace(
            status=status, result=types.SimpleNamespace(error_code=code, error_msg=message)))

    def make_stuck(self):
        """让当前目标看起来已经执行了 5 秒。"""
        self.node.stuck.goal_started_at = time.monotonic() - 5

    # ---- 用例 ----
    def test_no_progress_failure_halts_queue_and_reports_reason(self):
        pending = self.submit()
        self.make_stuck()
        self.node.odom_pose = (0.02, 0.0)          # 贴墙 2cm 抖动
        self.fail(pending)
        self.assertEqual(self.node.points, [], '剩余队列必须清空，不能继续喂目标')
        self.assertEqual(self.node.mode, 'locked')
        self.assertEqual(self.node.lock_kind, 'protective')
        self.assertIn('无位移', self.node.error)
        self.assertIn('贴墙', self.node.error)
        self.assertEqual(len(self.clears), 1, '只清一次 costmap')
        events = [event for event in self.node.events
                  if event['type'] == 'goal_failed_no_progress']
        self.assertTrue(events, '必须记录带原因的失败事件')
        self.assertEqual(events[-1]['costmapCleared'], True)
        self.assertIn('displacement', events[-1]['classification'])
        # 失败原因必须能通过 status()/last_stop 拿到（前端轮询就是这两条通道）
        status = self.node.status()
        self.assertEqual(status['goalState'], 'failed')
        self.assertEqual(status['phase'], 'failed')
        self.assertIn('无位移', status['error'])
        self.assertEqual(self.node.stuck.snapshot()['consecutive'], 1)
        self.assertEqual(self.node.last_stop['kind'], 'protective')

    def test_second_no_progress_failure_latches_stuck_and_rejects_goals(self):
        self.submit()
        self.make_stuck()
        self.node.odom_pose = (0.0, 0.0)
        self.fail(self.node.action.sent[-1][0])
        # 第二次：重新发目标，同样没有位移
        pending = self.submit([{'x': 1.5, 'y': 0.}])
        self.make_stuck()
        self.node.odom_pose = (0.0, 0.0)
        self.fail(pending)
        self.assertTrue(self.node.stuck.latched)
        self.assertEqual(self.node.lock_kind, 'stuck')
        self.assertIn('连续', self.node.error)
        # 第三次发目标必须被明确拒绝，并且提示人工处理
        with self.assertRaises(ValueError) as ctx:
            self.submit([{'x': 1.0, 'y': 0.}])
        self.assertIn('无位移失败已锁存', str(ctx.exception))
        self.assertIn('启用遥控', str(ctx.exception))
        # 人工接管后解除锁存
        self.node.process({'op': 'manual_enable', 'id': 9, 'sessionId': SESSION})
        self.assertFalse(self.node.stuck.latched)

    def test_failure_with_movement_keeps_normal_path(self):
        pending = self.submit()
        self.make_stuck()
        for step in range(1, 11):                  # 移动 1m 后失败
            self.node.odom_pose = (step * 0.1, 0.0)
            self.node.stuck.observe_odom(self.node.odom_pose)
        self.fail(pending, message='patience exceeded')
        self.assertEqual(self.clears, [], '有位移的失败不得清 costmap')
        self.assertFalse(self.node.stuck.latched)
        self.assertEqual(self.node.stuck.consecutive, 0)
        self.assertTrue(any(event['type'] == 'goal_failed' for event in self.node.events))
        self.assertFalse(any(event['type'] == 'goal_failed_no_progress'
                             for event in self.node.events))

    def test_effective_params_are_marked_unverified_when_services_missing(self):
        self.node.request_effective_params()
        params = self.node.effective_params
        self.assertIsNotNone(params)
        # 离线/无 Nav2 时必须如实标 unverified，绝不能拿 yaml 值冒充生效值
        self.assertEqual(params['values'], {})
        self.assertEqual(len(params['unverified']), 3)
        self.assertTrue(all(item['reason'] == '参数服务不可用' for item in params['unverified']))
        self.assertIn('stuckPolicy', self.node.status())

    def test_effective_params_recorded_when_service_responds(self):
        class Future:
            def __init__(self, values):
                self._values = values

            def result(self):
                return types.SimpleNamespace(values=self._values)

            def add_done_callback(self, callback):
                callback(self)

        def value(kind, **kwargs):
            base = dict(type=kind, string_value='', double_value=0., integer_value=0,
                        bool_value=False, string_array_value=[], double_array_value=[],
                        integer_array_value=[], bool_array_value=[])
            base.update(kwargs)
            return types.SimpleNamespace(**base)

        from rcl_interfaces.msg import ParameterType
        self.node.effective_params = {'readAt': 0, 'values': {}, 'missing': [],
                                      'unverified': [], 'servers': []}
        self.node._store_effective_params(
            '/controller_server',
            ('FollowPath.min_vel_x', 'progress_checker.plugin', 'FollowPath.max_vel_x'),
            Future([value(ParameterType.PARAMETER_DOUBLE, double_value=0.0),
                    value(ParameterType.PARAMETER_STRING, string_value='progress_checker'),
                    value(ParameterType.PARAMETER_NOT_SET)]))
        values = self.node.effective_params['values']
        self.assertEqual(values['/controller_server:FollowPath.min_vel_x'], 0.0)
        self.assertEqual(values['/controller_server:progress_checker.plugin'], 'progress_checker')
        self.assertIn('/controller_server:FollowPath.max_vel_x',
                      self.node.effective_params['missing'])


if __name__ == '__main__':
    unittest.main()
