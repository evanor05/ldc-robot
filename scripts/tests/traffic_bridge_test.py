"""交通灯接入 NavigationBridge 的集成测试（真实桥代码路径，fake Action client）。

验证用户核心用例在桥上的行为：
A→B 途中遇红灯 → 桥快照并取消（零速）→ 等待超过无进展超时 → 短暂遮灯 →
同一盏灯稳定绿 → 只用新 UUID 重派一次 A → 到达 A 后才派发 B。
另测：普通路段不干预、旁侧绿灯、人工接管后绿灯不自启、安全取消丢弃快照。
"""
import json
import sys
import types
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'scripts/tests'))

import rclpy  # noqa: E402
import navigation_bridge as module  # noqa: E402
from test_navigation_bridge import Client, Handle, free_map  # noqa: E402

SESSION = 'traffic-test-session'


class TrafficBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = module.Bridge(journal_path=':memory:')
        self.node.action.destroy()
        self.node.action = Client()
        self.node.requirements = lambda: []
        self.node.sensors = lambda: []
        self.node.running = True
        self.replies = []
        self.node.respond = lambda request, **data: self.replies.append((request, data))
        self.node.on_map(free_map())
        # begin 会 rotate 速度通道；单元测试直接造出本轮身份（恢复必须校验 epoch）。
        self.node.velocity_channel.rotate()
        self.node.process({'op': 'claim', 'id': 0, 'sessionId': SESSION})
        self.replies.clear()
        # 手工装一个等灯状态机（参数在构造时读取，单元测试直接注入）。
        import traffic_light_policy as tp
        self.tp = tp

        class Clock:
            now = 1000.0

            def __call__(self):
                return self.now

            def advance(self, seconds):
                self.now += seconds
        self.tp_clock = Clock()
        self.node.traffic = tp.TrafficLightPolicy(
            [tp.Intersection(name='I', x=1.0, y=0.0, radius=3.0, speed_limit_near=0.15)],
            clock=self.tp_clock)
        self.node.traffic_mode = 'test'

    def tearDown(self):
        self.node.destroy_node()

    # ---- 工具 ----
    def submit(self, points):
        self.node.process({'op': 'goals', 'id': 1, 'sessionId': SESSION, 'points': points})
        pending, _feedback = self.node.action.sent[-1]
        handle = Handle()
        pending.set_result(handle)          # 受理：goal_state → executing
        return handle

    def observation(self, color, track_id=1, eligible=None, kind='frame'):
        lights = []
        if kind == 'frame' and color is not None:
            lights = [{'track_id': track_id, 'name': 'traffic light', 'score': .9,
                       'xyxy': [0, 0, 8, 8], 'missed': 0, 'color': color,
                       'color_pass_eligible': (color == 'green') if eligible is None else eligible}]
        return types.SimpleNamespace(data=json.dumps({'kind': kind, 'lights': lights,
                                                      'light_count': len(lights)}))

    def feed(self, message, advance=0.25):
        """按真实帧率推进状态机时钟，再喂观测（防重复帧判定需要间隔）。"""
        self.tp_clock.advance(advance)
        self.node.on_traffic_observation(message)

    def finish_terminal(self, handle, status=module.GoalStatus.STATUS_CANCELED):
        """走 supervisor 的真实终态路径（on_result → 落盘 → 解除 pending）。"""
        handle.result.set_result(types.SimpleNamespace(status=status))

    def tick_with_pose(self, x=1.0, y=0.0):
        # 与产品一致：planar_pose 返回三元组 (x, y, yaw)。
        self.node.odom_pose = (x, y, 0.0)
        self.node._traffic_tick(__import__('time').monotonic())

    # ---- 核心用例 ----
    def test_red_light_waits_zero_then_green_dispatches_A_once_then_B(self):
        handle = self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}, {'x': 1.5, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(1.0, 0.0)                    # 进入路口范围
        self.feed(self.observation('red'))               # 红灯
        # 桥应快照+取消
        status = self.node.traffic.snapshot_status()
        self.assertTrue(status['hasSnapshot'], status)
        self.assertEqual(self.node.mode, 'locked')
        self.assertEqual(self.node.points, [], '取消清空队列')
        sent_before = len(self.node.action.sent)

        # 等待期间持续观测红灯/无灯，不得派发
        for index in range(20):
            self.feed(self.observation('red', kind='frame'))
        self.assertEqual(len(self.node.action.sent), sent_before, '等待期间不得派发')

        # 短暂遮灯 + 同一盏灯稳定绿
        self.feed(self.observation(None, kind='frame'))
        for index in range(4):
            self.feed(self.observation('green'))
        # 旧 Action 尚未确认终态 → 不恢复
        self.tick_with_pose(1.0, 0.0)
        self.assertEqual(len(self.node.action.sent), sent_before, '旧终态未确认不得恢复')

        # 旧 Action 终态确认（supervisor 收到远端取消终态 → 解除 pending）
        self.finish_terminal(handle)
        self.node.owner = SESSION
        import time as _t
        self.node.lease_at = _t.monotonic()
        # 清空发送记录，只观察恢复这一次派发
        self.node.action.sent.clear()
        self.tick_with_pose(1.0, 0.0)
        self.assertEqual(len(self.node.action.sent), 1, '恢复只重派一次 A')
        self.assertEqual(self.node.points, [{'x': 1.5, 'y': 0.0, 'yaw': 0.0}], 'B 仍在队列')

        # 再次 tick 不重复派发
        self.tick_with_pose(1.0, 0.0)
        self.assertEqual(len(self.node.action.sent), 1, '恢复只能一次')

    def test_new_uuid_on_resume(self):
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        first_uuid = self.node.goal_id
        self.tick_with_pose(1.0, 0.0)
        self.feed(self.observation('red'))
        for _ in range(4):
            self.feed(self.observation('green'))
        self.node.owner = SESSION
        import time as _t
        self.node.lease_at = _t.monotonic()
        self.node.actions._pending.clear()   # 模拟旧终态已确认
        self.tick_with_pose(1.0, 0.0)
        self.assertNotEqual(self.node.goal_id, first_uuid, '恢复必须使用新 UUID')

    def test_ordinary_road_ignores_light(self):
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(20.0, 20.0)     # 远离路口
        self.feed(self.observation('red'))
        self.assertEqual(self.node.mode, 'auto')
        self.assertFalse(self.node.traffic.snapshot_status()['hasSnapshot'], '普通路段照常导航')

    def test_side_green_never_resumes(self):
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(1.0, 0.0)
        self.feed(self.observation('red'))
        for _ in range(6):
            self.feed(self.observation('green', eligible=False))
        self.assertEqual(self.node.mode, 'locked', '旁侧绿的车辆不放行')
        self.assertTrue(self.node.traffic.snapshot_status()['hasSnapshot'])

    def test_manual_takeover_discards_snapshot(self):
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(1.0, 0.0)
        self.feed(self.observation('red'))
        self.assertTrue(self.node.traffic.snapshot_status()['hasSnapshot'])
        # 人工接管（安全类取消）
        self.node.cancel(reason='人工接管')
        self.assertFalse(self.node.traffic.snapshot_status()['hasSnapshot'], '接管必须丢弃快照')
        for _ in range(6):
            self.feed(self.observation('green'))
        self.node.owner = SESSION
        import time as _t
        self.node.lease_at = _t.monotonic()
        self.tick_with_pose(1.0, 0.0)
        self.assertEqual(self.node.mode, 'locked', '接管后绿灯不得自动重启')

    def test_not_at_intersection_light_is_recorded_not_enforced(self):
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        # 不进路口范围（默认 IDLE）
        self.feed(self.observation('red'))
        self.assertEqual(self.node.mode, 'auto')


    def test_nav2_abort_during_wait_does_not_clear_queue_or_latch_stuck(self):
        """红灯取消后 Nav2 以 aborted 结束时：仍保留队列，等稳定绿后可恢复。"""
        handle = self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}, {'x': 1.5, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(1.0, 0.0)
        self.feed(self.observation('red'))
        self.assertTrue(self.node.traffic.snapshot_status()['hasSnapshot'])
        # 远端以 ABORTED 结束（红灯取消导致的真实观测）
        self.node.handle_goal_failure('取消期间远端中止', error_code=0)
        self.assertNotEqual(self.node.goal_state, 'failed', '等灯期间不得判失败')
        self.assertFalse(self.node.stuck.latched, '等灯期间不得锁存无进展')
        # 稳定绿 + 终态确认后恢复
        for _ in range(4):
            self.feed(self.observation('green'))
        self.finish_terminal(handle)          # supervisor 收到远端终态
        self.node.owner = SESSION
        import time as _t
        self.node.lease_at = _t.monotonic()
        self.node.action.sent.clear()
        self.tick_with_pose(1.0, 0.0)
        self.assertEqual(len(self.node.action.sent), 1, '稳定绿后只重派一次 A')


    def test_on_auto_speed_limit_uses_intersection_not_policy(self):
        """回归：限速必须读 Intersection.speed_limit_near（曾误读 Policy → 桥崩溃）。"""
        from geometry_msgs.msg import Twist
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(1.0, 0.0)          # 进入路口 → APPROACH
        self.assertEqual(self.node.traffic.state, 'approach')
        published = []
        self.node.velocity.publish = lambda msg: published.append(msg)
        self.node.owner = SESSION
        import time as _t
        self.node.lease_at = _t.monotonic()
        message = Twist(); message.linear.x = 0.30
        self.node.on_auto(message)             # 不得抛异常
        self.assertTrue(published, '接近路口应转发（限速后）')
        self.assertLessEqual(abs(published[-1].linear.x), 0.15 + 1e-9, '应被限速削到路口限速')

    def test_on_auto_without_intersection_passes_through(self):
        from geometry_msgs.msg import Twist
        self.submit([{'x': 0.8, 'y': 0.0, 'yaw': 0.0}])
        self.tick_with_pose(20.0, 20.0)        # 远离路口
        published = []
        self.node.velocity.publish = lambda msg: published.append(msg)
        self.node.owner = SESSION
        import time as _t
        self.node.lease_at = _t.monotonic()
        message = Twist(); message.linear.x = 0.30
        self.node.on_auto(message)
        self.assertAlmostEqual(published[-1].linear.x, 0.30, msg='未进入路口不得改动速度')


if __name__ == '__main__':
    unittest.main()