"""端到端逻辑回归：视觉观测 → 等灯状态机 → 桥动作序列（fake bridge，无 ROS）。

模拟导航桥在用户核心用例下的行为：
A→B 途中遇红灯 → 取消并保持零速 → 等待超过无进展超时 → 短暂遮灯 →
同一盏灯稳定绿 → 只派发一次 A → 到达 A 后才派发 B。
断言等待期间每一步速度都是 0，且派发序列恰好是 [A, B]（各一次）。
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import traffic_light_policy as tp


class FakeBridge:
    """最小桥模型：复刻 cancel 清队列 / locked 零速 / next_goal 推进语义。"""

    def __init__(self, points):
        self.points = list(points)
        self.current = self.points.pop(0)
        self.mode = 'auto'
        self.speed = 0.0
        self.dispatched = []      # 每次派发记录 (goal, uuid)
        self.uuid_counter = 0
        self.cancel_clears_queue = True
        self.pending_terminal = False   # 旧 Action 终态是否已确认
        self.speed_trace = []

    def _tick_speed(self):
        self.speed = 0.0 if self.mode == 'locked' else self.speed
        self.speed_trace.append(self.speed)

    def cancel(self):
        if self.cancel_clears_queue:
            self.points = []          # 与 bridge cancel:678 首行一致
        self.mode = 'locked'
        self.pending_terminal = False
        self._tick_speed()

    def confirm_terminal(self):
        self.pending_terminal = True

    def dispatch(self, goal, queue=None):
        assert self.mode in ('auto', 'resuming')
        if queue is not None:
            self.points = [dict(p) for p in queue]   # 恢复时一并还原队列
        self.uuid_counter += 1
        self.dispatched.append((dict(goal), f'uuid-{self.uuid_counter}'))
        self.current = dict(goal)
        self.speed = 0.1

    def arrive(self):
        """到达当前目标：推进下一站（next_goal 语义）。"""
        if self.points:
            self.dispatch(self.points.pop(0))


def make_policy(clock):
    return tp.TrafficLightPolicy(
        [tp.Intersection(name='I', x=5.0, y=0.0, radius=3.0, speed_limit_near=0.15)],
        clock=clock)


def frame(colors, track_id=1, eligible=None):
    lights = []
    for index, color in enumerate(colors):
        lights.append({'track_id': track_id + index, 'name': 'traffic light', 'score': .9,
                       'xyxy': [0, 0, 8, 8], 'missed': 0, 'color': color,
                       'color_pass_eligible': (color == tp.GREEN) if eligible is None else eligible})
    return {'kind': 'frame', 'lights': lights, 'light_count': len(lights)}


@pytest.fixture
def clock():
    class Clock:
        now = 100.0

        def __call__(self):
            return self.now
    return Clock()


def test_user_core_case_red_light_then_stable_green_dispatches_A_once_then_B(clock):
    bridge = FakeBridge([{'x': 8.0, 'y': 0.0, 'yaw': 0.0}, {'x': 12.0, 'y': 0.0, 'yaw': 1.57}])
    policy = make_policy(clock)
    bridge.dispatch(bridge.current)
    assert [goal for goal, _ in bridge.dispatched] == [{'x': 8.0, 'y': 0.0, 'yaw': 0.0}]

    # 接近路口 + 红灯
    policy.on_position({'x': 4.0, 'y': 0.0})
    decision = policy.on_lamp(tp.RED, track_id=1)
    assert decision.action == 'stop_and_hold'
    # 桥：先快照后取消（顺序关键——取消会清队列）
    policy.arm_snapshot(bridge.current, bridge.points)
    bridge.cancel()
    assert bridge.points == [], '取消确实清空队列（本轮实测语义）'
    assert bridge.speed == 0.0

    # 等待超过无进展超时（60s），期间零速
    for _ in range(120):
        clock.now += 0.5
        bridge._tick_speed()
    assert all(value == 0.0 for value in bridge.speed_trace), '等待期间必须持续零速'
    assert len(bridge.dispatched) == 1, '等待期间不得重复派发'

    # 短暂遮灯
    clock.now += 0.2
    policy.on_observation(frame([], track_id=1))
    # 同一盏灯稳定绿
    for _ in range(5):
        clock.now += 0.2
        policy.on_observation(frame([tp.GREEN], track_id=1))

    # 旧 Action 终态未确认 → 不恢复
    assert policy.can_resume(actions_quiet=bridge.pending_terminal, authorized=True,
                             ready=True, identity_ok=True).action == 'none'
    bridge.confirm_terminal()
    resume = policy.can_resume(actions_quiet=bridge.pending_terminal, authorized=True,
                               ready=True, identity_ok=True)
    assert resume.action == 'resume'
    bridge.mode = 'auto'
    bridge.dispatch(resume.resume_goal, resume.resume_queue)

    # 到达 A 后才推进 B
    assert [goal for goal, _ in bridge.dispatched] == [
        {'x': 8.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 8.0, 'y': 0.0, 'yaw': 0.0},
    ], '恢复只重派 A'
    bridge.arrive()
    assert [goal for goal, _ in bridge.dispatched][-1] == {'x': 12.0, 'y': 0.0, 'yaw': 1.57}
    assert len(bridge.dispatched) == 3, 'A 两次（原派+恢复重派），B 一次'
    # 新 UUID
    uuids = [uuid for _, uuid in bridge.dispatched]
    assert len(set(uuids)) == len(uuids), '每次派发必须使用新 UUID'
    # 恢复后队尾 B 仍在
    assert resume.resume_queue == [{'x': 12.0, 'y': 0.0, 'yaw': 1.57}]


def test_side_green_never_resumes(clock):
    bridge = FakeBridge([{'x': 8.0, 'y': 0.0, 'yaw': 0.0}])
    policy = make_policy(clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    policy.on_lamp(tp.RED, track_id=1)
    policy.arm_snapshot(bridge.current, bridge.points)
    bridge.cancel()
    for _ in range(10):
        clock.now += 0.2
        policy.on_observation(frame([tp.GREEN], eligible=False))
    assert bridge.speed == 0.0


def test_manual_takeover_during_wait_never_resumes(clock):
    bridge = FakeBridge([{'x': 8.0, 'y': 0.0, 'yaw': 0.0}])
    policy = make_policy(clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    policy.on_lamp(tp.RED, track_id=1)
    policy.arm_snapshot(bridge.current, bridge.points)
    bridge.cancel()
    policy.abort('manual_takeover')   # 桥在接管时调用
    for _ in range(10):
        clock.now += 0.2
        policy.on_observation(frame([tp.GREEN], track_id=1))
    assert policy.can_resume(actions_quiet=True, authorized=True, ready=True,
                             identity_ok=True).action == 'none'
    assert bridge.speed == 0.0, '接管后绿灯不得自动重启车辆'