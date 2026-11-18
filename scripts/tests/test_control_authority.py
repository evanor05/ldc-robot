"""Control-authority regression tests: motion lease must belong to one page session.

这些测试描述第二阶段要求的行为（租约归属、显式接管、旧租约失效、事件记录、
begin 清理残留状态）。在修复前的旧代码上运行会失败——失败输出保留在审计目录
作为“先证明问题”的证据；修复后它们与既有单元测试一起构成回归防线。
"""
import concurrent.futures
import importlib.util
from pathlib import Path
import time
import types
import unittest
import rclpy

spec = importlib.util.spec_from_file_location('scripts.navigation_bridge', Path(__file__).resolve().parents[1] / 'navigation_bridge.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)

SESSION_A = 'page-session-aaaa'
SESSION_B = 'page-session-bbbb'


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


class ControlAuthorityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def setUp(self):
        self.node = module.Bridge(); self.node.action.destroy()
        self.node.action = Client(); self.node.requirements = lambda: []; self.node.sensors = lambda: []
        self.node.running = True; self.node.startup_done = True
        self.node.nav_started = time.monotonic()
        from nav_msgs.msg import OccupancyGrid
        grid = OccupancyGrid(); grid.info.width = grid.info.height = 200
        grid.info.resolution = .05
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 40000
        self.node.on_map(grid)
        self.replies = []
        self.node.respond = lambda request, **data: self.replies.append((request, data))

    def tearDown(self): self.node.destroy_node()

    def tick(self):
        """tick 且隔离测试进程 stdin EOF 触发的 shutdown 命令，只考察租约逻辑。"""
        while not self.node.commands.empty():
            item = self.node.commands.get_nowait()
            if item.get('op') != 'shutdown':
                self.node.process(item)
        self.node.shutdown_requested = False
        self.node.tick()

    def reply(self, request_id):
        for request, data in self.replies:
            if request.get('id') == request_id:
                return data
        return None

    def claim(self, session, request_id=100):
        self.node.process({'op': 'claim', 'id': request_id, 'sessionId': session})

    def submit(self, session=SESSION_A, points=None, request_id=1):
        self.node.process({'op': 'goals', 'id': request_id, 'sessionId': session,
                           'requestId': f'req-{request_id}',
                           'points': points or [{'x': 1, 'y': 0}]})
        return self.node.action.sent[-1][0]

    def result(self, handle, status=4, code=0):
        handle.result.set_result(types.SimpleNamespace(
            status=status, result=types.SimpleNamespace(error_code=code, error_msg='aborted' if code else '')))

    # ---- 归属与心跳 ----

    def test_claim_grants_owner_and_status_reports_it(self):
        self.claim(SESSION_A)
        self.assertTrue(self.reply(100)['success'])
        self.assertEqual(self.node.status()['controlOwner'], SESSION_A)
        # 同一会话重复 claim 幂等；其他会话 claim 被拒绝。
        self.claim(SESSION_A, 101)
        self.assertTrue(self.reply(101)['success'])
        with self.assertRaises(ValueError):
            self.claim(SESSION_B, 102)
        self.assertEqual(self.node.status()['controlOwner'], SESSION_A)

    def test_stranger_heartbeat_never_renews_motion_lease(self):
        """核心竞态证明：另一个页面的心跳不得为运动租约续期。"""
        self.claim(SESSION_A)
        self.submit()
        handle = Handle(); self.node.action.sent[-1][0].set_result(handle)
        self.assertEqual(self.node.mode, 'auto')
        # 控制页 A 失联：租约已过期。
        self.node.lease_at = time.monotonic() - (module.LEASE_TTL_SECONDS + .1)
        # 观察页 B 的心跳（旧代码会无条件续期）必须被拒绝。
        with self.assertRaises(ValueError):
            self.node.process({'op': 'heartbeat', 'id': 7, 'sessionId': SESSION_B})
        self.tick()
        self.assertEqual(self.node.mode, 'locked', '陌生心跳续期导致旧运动继续')
        self.assertEqual(self.node.points, [])

    def test_owner_heartbeat_renews_lease(self):
        self.claim(SESSION_A)
        self.submit()
        self.node.lease_at = time.monotonic() - (module.LEASE_TTL_SECONDS - .1)
        self.node.process({'op': 'heartbeat', 'id': 8, 'sessionId': SESSION_A})
        self.assertTrue(self.reply(8)['success'])
        self.tick()
        self.assertEqual(self.node.mode, 'auto')

    def test_anonymous_heartbeat_never_renews(self):
        self.claim(SESSION_A)
        self.submit()
        self.node.lease_at = time.monotonic() - (module.LEASE_TTL_SECONDS + .1)
        with self.assertRaises(ValueError):
            self.node.process({'op': 'heartbeat', 'id': 9})
        self.tick()
        self.assertEqual(self.node.mode, 'locked')

    # ---- 运动/目标操作的归属校验 ----

    def test_non_owner_cannot_send_goals(self):
        self.claim(SESSION_A)
        with self.assertRaises(ValueError):
            self.node.process({'op': 'goals', 'id': 2, 'sessionId': SESSION_B, 'points': [{'x': 1, 'y': 0}]})
        self.assertEqual(self.node.action.sent, [])

    def test_non_owner_cannot_enable_manual_or_send_velocity(self):
        self.claim(SESSION_A)
        with self.assertRaises(ValueError):
            self.node.process({'op': 'manual_enable', 'id': 3, 'sessionId': SESSION_B})
        self.node.process({'op': 'manual_enable', 'id': 4, 'sessionId': SESSION_A})
        token = self.reply(4)['controlId']
        # 正确 controlId 但错误会话：拒绝（旧租约不能发速度）。
        with self.assertRaises(ValueError):
            self.node.process({'op': 'velocity', 'sessionId': SESSION_B, 'controlId': token,
                               'sequence': 1, 'linearX': .1, 'angularZ': 0})
        self.node.process({'op': 'velocity', 'id': 5, 'sessionId': SESSION_A, 'controlId': token,
                           'sequence': 2, 'linearX': .1, 'angularZ': 0})
        self.assertTrue(self.reply(5)['success'])

    def test_non_owner_cannot_set_initial_pose(self):
        self.claim(SESSION_A)
        self.node.lifecycle = {'amcl': (3, time.monotonic())}
        with self.assertRaises(ValueError):
            self.node.process({'op': 'initial_pose', 'id': 5, 'sessionId': SESSION_B,
                               'x': 0., 'y': 0., 'yaw': 0.})

    def test_estop_and_cancel_allowed_from_any_page(self):
        self.claim(SESSION_A)
        self.submit()
        self.node.process({'op': 'estop', 'id': 6, 'sessionId': SESSION_B})
        self.assertEqual(self.node.mode, 'locked')
        self.assertEqual(self.node.points, [])
        # 急停不改变归属：控制页仍是 A。
        self.assertEqual(self.node.status()['controlOwner'], SESSION_A)

    # ---- 显式接管 ----

    def test_takeover_cancels_confirms_then_grants_new_lease(self):
        self.claim(SESSION_A)
        pending = self.submit()
        self.node.process({'op': 'takeover', 'id': 10, 'sessionId': SESSION_B})
        # 取消未确认前不得移交。
        self.assertIsNone(self.reply(10))
        self.assertEqual(self.node.status()['controlOwner'], SESSION_A)
        handle = Handle(); pending.set_result(handle)
        self.assertTrue(handle.canceled)
        self.result(handle, status=5)
        takeover = self.reply(10)
        self.assertIsNotNone(takeover)
        self.assertTrue(takeover['success'])
        self.assertEqual(self.node.status()['controlOwner'], SESSION_B)
        self.assertEqual(self.node.mode, 'locked', '接管后不得自动恢复运动')
        # 旧租约不能续期、不能发目标。
        with self.assertRaises(ValueError):
            self.node.process({'op': 'heartbeat', 'id': 11, 'sessionId': SESSION_A})
        with self.assertRaises(ValueError):
            self.node.process({'op': 'goals', 'id': 12, 'sessionId': SESSION_A, 'points': [{'x': 1, 'y': 0}]})

    def test_release_by_owner_frees_control(self):
        self.claim(SESSION_A)
        with self.assertRaises(ValueError):
            self.node.process({'op': 'release', 'id': 13, 'sessionId': SESSION_B})
        self.node.process({'op': 'release', 'id': 14, 'sessionId': SESSION_A})
        self.assertIsNone(self.node.status()['controlOwner'])
        self.claim(SESSION_B, 103)
        self.assertEqual(self.node.status()['controlOwner'], SESSION_B)

    def test_lease_expiry_clears_owner_and_locks(self):
        self.claim(SESSION_A)
        self.submit()
        self.node.lease_at = time.monotonic() - (module.LEASE_TTL_SECONDS + .1)
        self.tick()
        self.assertEqual(self.node.mode, 'locked')
        self.assertIsNone(self.node.status()['controlOwner'], '失联页面的租约必须释放')
        # 恢复后不自动续跑；新页面可显式取得控制。
        self.claim(SESSION_B, 104)
        self.assertEqual(self.node.status()['controlOwner'], SESSION_B)
        self.assertEqual(self.node.mode, 'locked')

    # ---- begin 清理残留 ----

    def test_begin_resets_residual_run_state(self):
        self.claim(SESSION_A)
        pending = self.submit()
        handle = Handle(); pending.set_result(handle)
        self.node.action.sent[-1][1](types.SimpleNamespace(feedback=types.SimpleNamespace(
            distance_remaining=.5, estimated_time_remaining=types.SimpleNamespace(sec=3, nanosec=0))))
        self.assertIsNotNone(self.node.feedback)
        self.result(handle)  # succeeded → 下一目标出队
        self.node.process({'op': 'stop', 'id': 20, 'sessionId': SESSION_A})
        self.node.process({'op': 'begin', 'id': 21, 'sessionId': SESSION_A, 'mapPath': '/tmp/maps/next'})
        state = self.node.status()
        self.assertIsNone(state['feedback'], 'begin 后残留上一轮反馈')
        self.assertIsNone(state['current'])
        self.assertEqual(state['queue'], [])
        self.assertIsNone(state['goalId'])
        self.assertEqual(state['goalState'], 'idle')
        self.assertEqual(state['mode'], 'locked')

    # ---- 事件记录 ----

    def test_events_record_run_goal_and_reason(self):
        self.claim(SESSION_A)
        pending = self.submit(request_id=30)
        handle = Handle(); pending.set_result(handle)
        self.result(handle)
        self.node.process({'op': 'events', 'id': 31, 'since': 0})
        payload = self.reply(31)
        self.assertTrue(payload['success'])
        types_seen = [event['type'] for event in payload['events']]
        for expected in ('control_claimed', 'goal_submitted', 'goal_accepted', 'goal_succeeded'):
            self.assertIn(expected, types_seen)
        goal_events = [e for e in payload['events'] if e.get('goalId')]
        self.assertTrue(goal_events)
        self.assertTrue(all(e['goalId'] == self.node.goal_id for e in goal_events))
        self.assertTrue(all('runId' in e for e in payload['events']))
        submitted = [e for e in payload['events'] if e['type'] == 'goal_submitted']
        self.assertEqual(submitted[0].get('requestId'), 'req-30')
        # since 过滤：只返回更新的事件。
        last_seq = payload['events'][-1]['seq']
        self.node.process({'op': 'estop', 'id': 32, 'sessionId': SESSION_B})
        self.node.process({'op': 'events', 'id': 33, 'since': last_seq})
        newer = self.reply(33)['events']
        self.assertTrue(newer and all(e['seq'] > last_seq for e in newer))
        self.assertIn('estop', [e['type'] for e in newer])

    def test_lock_reasons_recorded_as_events(self):
        self.claim(SESSION_A)
        self.submit()
        self.node.lease_at = time.monotonic() - (module.LEASE_TTL_SECONDS + .1)
        self.tick()
        self.node.process({'op': 'events', 'id': 40, 'since': 0})
        events = self.reply(40)['events']
        lease_events = [e for e in events if e['type'] == 'lease_expired']
        self.assertTrue(lease_events, '心跳失联必须留下事件')
        self.assertTrue(lease_events[0]['reason'])


if __name__ == '__main__':
    unittest.main()
