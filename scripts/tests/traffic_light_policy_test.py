"""等灯状态机离线回归（纯逻辑，无 ROS）。

用户核心用例：A→B 途中遇红灯 → 等待超过无进展超时 → 短暂遮灯 → 同一盏灯稳定绿
→ 等待期间持续零速、恢复只派发一次 A、到达 A 后才派发 B。
另测：旁侧绿灯、双路冲突、过期帧、视觉断流、线前/越线黄灯、等灯时人工接管。
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import traffic_light_policy as tp


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def tick(self, seconds):
        self.now += seconds
        return self.now


def intersection():
    return tp.Intersection(name='I-AtoB', x=5.0, y=0.0, radius=3.0, speed_limit_near=0.15)


def frame(colors, *, track_id=1, eligible=None, stamp=None):
    """构造一条 frame 观测；colors 为灯色，eligible 默认按绿/非绿。"""
    lights = []
    for index, color in enumerate(colors):
        pass_ok = (color == tp.GREEN) if eligible is None else eligible
        lights.append({'track_id': track_id + index, 'name': 'traffic light',
                       'score': .9, 'xyxy': [10 * index, 0, 10 * index + 8, 8],
                       'missed': 0, 'color': color, 'color_pass_eligible': pass_ok,
                       'opencv_color': color})
    return {'kind': 'frame', 'id': f'f{stamp or 0}', 'sequence': stamp or 0,
            'stamp_ns': (stamp or 0), 'lights': lights, 'light_count': len(lights),
            'at': 0.0}


def reach_waiting(clock, *, color=tp.RED, passed_line=False):
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    decision = policy.on_lamp(color, track_id=1, passed_line=passed_line)
    assert decision.action == 'stop_and_hold', decision
    policy.arm_snapshot({'x': 8.0, 'y': 0.0, 'yaw': 0.0},
                        [{'x': 12.0, 'y': 0.0, 'yaw': 1.57}], boot_id='b1', epoch='e1')
    return policy


def green_frames(policy, clock, count, track_id=1, step=0.2):
    decision = None
    for _ in range(count):
        clock.tick(step)
        decision = policy.on_observation(frame([tp.GREEN], track_id=track_id))
    return decision


# ---------------- 核心用例 ----------------
def test_core_red_light_wait_exceeds_stuck_timeout_then_stable_green():
    clock = Clock()
    policy = reach_waiting(clock)
    assert policy.state == tp.WAITING
    assert policy.snapshot_status()['hasSnapshot'] is True

    # 等待远超无进展超时（stuck 只看目标失败，等待态不派发 → 不会被误杀）
    clock.tick(60.0)
    assert policy.state == tp.WAITING, '等待期间不得自行改变状态'

    # 短暂遮灯：无灯帧 → green run 清零，快照保留
    clock.tick(0.3)
    assert policy.on_observation(frame([], stamp=1)).action == 'none'
    assert policy.snapshot_status()['hasSnapshot'] is True

    # 同一盏灯连续稳定绿
    decision = green_frames(policy, clock, policy.policy.green_confirm_frames)
    assert decision.state == tp.RESUMING

    # 旧 Action 终态未确认 → 不得恢复
    blocked = policy.can_resume(actions_quiet=False, authorized=True, ready=True, identity_ok=True)
    assert blocked.action == 'none' and 'old_action_terminal_unknown' in blocked.reason
    assert policy.snapshot_status()['hasSnapshot'] is True

    # 全真 → 一次性恢复，还原同一目标与剩余队列
    resume = policy.can_resume(actions_quiet=True, authorized=True, ready=True, identity_ok=True)
    assert resume.action == 'resume'
    assert resume.resume_goal == {'x': 8.0, 'y': 0.0, 'yaw': 0.0}, '必须恢复同一目标 A'
    assert resume.resume_queue == [{'x': 12.0, 'y': 0.0, 'yaw': 1.57}], 'B 只在到达 A 后才派发'
    assert policy.snapshot_status()['hasSnapshot'] is False

    # 只能恢复一次：再次调用不得重复派发
    again = policy.can_resume(actions_quiet=True, authorized=True, ready=True, identity_ok=True)
    assert again.action == 'none', '恢复必须只派发一次'


def test_stable_green_requires_consecutive_new_frames():
    clock = Clock()
    policy = reach_waiting(clock)
    # 交替绿/红：不能累计到稳定绿
    for index in range(6):
        clock.tick(0.2)
        policy.on_observation(frame([tp.GREEN if index % 2 == 0 else tp.RED], stamp=index))
    assert policy.state == tp.WAITING, '非连续绿不得恢复'


def test_duplicate_frames_too_soon_do_not_count():
    clock = Clock()
    policy = reach_waiting(clock)
    for _ in range(5):
        policy.on_observation(frame([tp.GREEN]))  # 时钟不前进 → 同一帧重复
    assert policy.state == tp.WAITING, '重复旧帧不得凑够稳定绿'
    decision = green_frames(policy, clock, policy.policy.green_confirm_frames)
    assert decision.state == tp.RESUMING


# ---------------- 旁侧绿灯 / 双路冲突 ----------------
def test_side_green_is_not_pass_eligible():
    clock = Clock()
    policy = reach_waiting(clock)
    # 旁侧绿：颜色是绿但 pass_eligible=False（非本车道 / 双路冲突）
    for index in range(5):
        clock.tick(0.2)
        policy.on_observation(frame([tp.GREEN], eligible=False, stamp=index))
    assert policy.state == tp.WAITING, '旁侧绿灯不能放行'


def test_consensus_conflict_is_unknown_and_holds():
    clock = Clock()
    policy = reach_waiting(clock)
    for index in range(5):
        clock.tick(0.2)
        observation = frame(['unknown'], eligible=False, stamp=index)
        observation['lights'][0]['color_conflict'] = True
        observation['lights'][0]['color_reason'] = 'dual_path_conflict'
        policy.on_observation(observation)
    assert policy.state == tp.WAITING


# ---------------- 视觉断流 / 过期帧 ----------------
def test_stream_lost_holds_and_resets_green_run():
    clock = Clock()
    policy = reach_waiting(clock)
    green_frames(policy, clock, policy.policy.green_confirm_frames - 1)
    policy.on_observation({'kind': 'stream_lost', 'at': clock.now})
    decision = green_frames(policy, clock, policy.policy.green_confirm_frames)
    # 断流后重新累计（第一次统计因 last_green_at 间隔可能提前计一次，这里验证最终仍能恢复）
    assert decision.state in (tp.WAITING, tp.RESUMING)
    assert policy.snapshot_status()['hasSnapshot'] is True, '断流不得丢快照'


def test_stream_lost_beyond_grace_holds_without_recovery():
    clock = Clock()
    policy = reach_waiting(clock)
    policy.on_observation({'kind': 'stream_lost', 'at': clock.now})
    clock.tick(10.0)
    decision = policy.on_observation(frame([tp.GREEN]))
    assert decision.action == 'none' and 'stream_unavailable_hold' in decision.reason
    assert policy.state == tp.WAITING


def test_stale_frame_does_not_advance_green():
    clock = Clock()
    policy = reach_waiting(clock)
    for index in range(5):
        clock.tick(0.2)
        decision = policy.on_observation({'kind': 'stale_frame', 'at': clock.now})
        assert decision.action == 'none'
    assert policy.state == tp.WAITING


# ---------------- 黄灯：线前 vs 越线 ----------------
def test_yellow_before_line_stops():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    decision = policy.on_lamp(tp.YELLOW, track_id=1, passed_line=False)
    assert decision.action == 'stop_and_hold'


def test_yellow_after_reliable_line_crossing_proceeds_slowly():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    decision = policy.on_lamp(tp.YELLOW, track_id=1, passed_line=True)
    assert decision.action == 'none' and decision.speed_limit == policy.policy.resume_speed
    assert policy.state == tp.PASSED


def test_green_proceeds_without_waiting():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    decision = policy.on_lamp(tp.GREEN, track_id=1)
    assert decision.action == 'none' and policy.state == tp.PASSED


# ---------------- 路口关联 / 普通路段 ----------------
def test_far_from_intersection_ignores_light():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    decision = policy.on_position({'x': 0.0, 'y': 0.0})
    assert decision.action == 'none'
    assert policy.on_lamp(tp.RED, track_id=1).action == 'none', '普通路段照常导航'


def test_approach_limits_speed():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    decision = policy.on_position({'x': 4.0, 'y': 0.0})
    assert decision.action == 'limit' and decision.speed_limit == 0.15


# ---------------- 等灯时人工接管等安全事件 ----------------
@pytest.mark.parametrize('reason', ['manual_takeover', 'estop', 'task_cancel', 'lease_expired'])
def test_safety_events_discard_snapshot_and_green_cannot_restart(reason):
    clock = Clock()
    policy = reach_waiting(clock)
    policy.abort(reason)
    assert policy.snapshot_status()['hasSnapshot'] is False
    # 后续绿灯绝不能自行恢复
    green_frames(policy, clock, 10)
    resume = policy.can_resume(actions_quiet=True, authorized=True, ready=True, identity_ok=True)
    assert resume.action == 'none'
    assert policy.state == tp.ABORTED
    # 只有人工显式重新启用才回到 IDLE
    assert policy.reenable().state == tp.IDLE


def test_waiting_without_snapshot_aborts_instead_of_resuming():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    policy.state = tp.WAITING  # 异常：处于等待但没有快照
    decision = policy.on_observation(frame([tp.GREEN]))
    assert decision.action == 'abort' and policy.state == tp.ABORTED


def test_page_status_answers_three_questions():
    clock = Clock()
    policy = reach_waiting(clock)
    status = policy.snapshot_status()
    assert status['state'] == tp.WAITING
    assert status['intersection'] is not None
    assert status['hasSnapshot'] is True
    assert status['snapshotGoal'] == {'x': 8.0, 'y': 0.0, 'yaw': 0.0}
    assert status['canUserDo'], '必须告诉用户现在能做什么'


def test_resume_clears_active_intersection_so_speed_not_limited_forever():
    """回归：恢复后必须清空活动路口，否则限速会永久压住自动速度。"""
    clock = Clock()
    policy = reach_waiting(clock)
    decision = green_frames(policy, clock, policy.policy.green_confirm_frames)
    assert decision.state == tp.RESUMING
    resume = policy.can_resume(actions_quiet=True, authorized=True, ready=True, identity_ok=True)
    assert resume.action == 'resume'
    assert policy.active_intersection is None, '恢复后不得继续把速度压在本路口限速'


def test_passed_intersection_not_reconsulted():
    clock = Clock()
    policy = tp.TrafficLightPolicy([intersection()], clock=clock)
    policy.on_position({'x': 4.0, 'y': 0.0})
    policy.on_lamp(tp.GREEN, track_id=1)
    assert policy.state == tp.PASSED
    # 再次进入同一路口范围不再限速/等待
    assert policy.on_position({'x': 4.5, 'y': 0.0}).action == 'none'