"""等灯状态机（纯逻辑，无 ROS 副作用）。

职责：把「交通灯观测」翻译成「导航桥该做什么」，并把等待期间的任务状态安全保存与恢复。
ROS/桥 只提供 transport（订阅、取消、派发、服务检查），本模块不 import rclpy。

设计依据（本轮实测语义，见 F-traffic-light/cancel-queue-semantics-20261118.md）：
- 取消会清空队列（bridge cancel 首行 self.points=[]），因此等待前必须**快照**
  （当前目标 x/y/yaw + 剩余队列），恢复时一次性还原。
- 成功回调经 next_goal() 推进下一站；迟到回调有 token 守卫。
- NoProgressPolicy 只在目标失败时判定，保持目标静止会被 Nav2 progress_checker
  判失败 → 因此等灯采用**取消+重发**，并让桥在等灯态豁免 no-progress。

核心时序（用户规格）：
1. 线前红/黄灯 → 取消当前目标、车辆锁定零速、快照任务。
2. 等待期间：任何灯色变化都不派发；视觉断流保持等待；短暂遮挡不清空快照。
3. 恢复条件（全部满足）：同一盏灯在**连续新画面**里稳定为绿
   ∧ 控制权/定位/导航就绪 ∧ 旧 Action 终态已确认（actions quiet）
   ∧ 恢复模块放行（recovery.motion_allowed）∧ 运行身份一致。
4. 恢复动作：还原快照 → 以**新 UUID** 派发同一目标**一次**；若恢复前被取消/接管/
   急停/租约失效，则丢弃快照、永不自行重启。
"""
from dataclasses import dataclass, field, replace
import time

# 灯色
RED, YELLOW, GREEN, UNKNOWN = 'red', 'yellow', 'green', 'unknown'

# 状态机状态
IDLE = 'idle'                    # 普通行驶，尚无路口约束
APPROACH = 'approach'            # 接近受控路口（限速中）
WAITING = 'waiting'              # 已取消目标、零速等灯，持有快照
RESUMING = 'resuming'            # 稳定绿已达成，等待旧终态确认/授权后派发
PASSED = 'passed'                # 已越过该路口（本次路口不再约束）
ABORTED = 'aborted'              # 快照被丢弃（安全事件），绿灯不得自启


@dataclass(frozen=True)
class Intersection:
    """受控路口配置。赛道坐标/停止线留现场标定（默认值仅为占位）。"""
    name: str
    x: float
    y: float
    radius: float = 3.0            # 路口关联半径（米）
    stop_line_offset: float = None  # 沿行驶方向的停止线距离；None=无可靠数据
    speed_limit_near: float = 0.15  # 接近路口限速（m/s）


@dataclass
class Snapshot:
    """等待前保存的任务状态；恢复时凭此一次性还原。"""
    goal: dict                     # {'x','y','yaw'}
    queue: list                    # 剩余目标列表（原顺序拷贝）
    intersection: str
    created_at: float
    boot_id: str = ''
    epoch: str = ''


@dataclass
class Policy:
    green_confirm_frames: int = 3   # 同一盏灯连续新画面的稳定绿帧数
    min_green_interval: float = 0.15  # 相邻确认帧的最小间隔（秒），防旧帧重复计数
    stale_after: float = 1.0        # 观测超过此秒数视为过期
    stream_lost_grace: float = 2.0  # 视觉断流容差：此期间保持等待不清快照
    unknown_hold: float = 0.0       # 灯色不清晰保持等待（0=无限等待，安全优先）
    yellow_passed_tolerance: float = 0.0  # 越线判定容差（米）
    resume_speed: float = 0.15      # 恢复后低速驶离

    def __post_init__(self):
        if self.green_confirm_frames < 1:
            raise ValueError('green_confirm_frames 必须 >= 1')


@dataclass
class Decision:
    """状态机对桥的指令（桥只执行，不做判断）。"""
    action: str                    # 'none'|'limit'|'stop_and_hold'|'resume'|'abort'
    reason: str = ''
    speed_limit: float = None
    resume_goal: dict = None
    resume_queue: list = None
    state: str = IDLE


class TrafficLightPolicy:
    """等灯状态机。所有输入都来自观测与桥的状态查询（只读）。"""

    def __init__(self, intersections=(), policy=None, clock=time.monotonic):
        self.intersections = list(intersections)
        self.policy = policy or Policy()
        self.clock = clock
        self.state = IDLE
        self.snapshot = None
        self.active_intersection = None
        self._green_run = 0
        self._last_green_at = 0.0
        self._stream_lost_at = None
        self._passed = set()
        self.last_lamp = None          # 最近一次关联到的灯（track_id/颜色）
        self.events = []

    # ---------- 工具 ----------
    def _record(self, kind, reason='', **extra):
        self.events.append({'at': self.clock(), 'kind': kind, 'reason': reason,
                            'state': self.state, **extra})

    def _nearest(self, pose):
        if not self.intersections or pose is None:
            return None
        best, distance = None, None
        for item in self.intersections:
            d = ((pose['x'] - item.x) ** 2 + (pose['y'] - item.y) ** 2) ** 0.5
            if distance is None or d < distance:
                best, distance = item, d
        if distance is None or distance > best.radius:
            return None
        return best

    # ---------- 观测输入 ----------
    def on_observation(self, observation, now=None):
        """喂一条 /vision/traffic_lights 观测（原始 JSON dict）。返回 Decision。"""
        now = self.clock() if now is None else now
        if not isinstance(observation, dict):
            return self._decide('none', 'malformed')
        kind = observation.get('kind')
        if kind == 'stream_lost':
            self._stream_lost_at = self._stream_lost_at or now
            if self.state == WAITING:
                self._green_run = 0
            return self._decide('none', 'stream_lost')
        if kind == 'stream_recovered':
            self._stream_lost_at = None
            self._green_run = 0  # 恢复后需重新累计连续绿帧
            self._record('stream_recovered')
            return self._decide('none', 'stream_recovered')
        if kind == 'stale_frame':
            return self._decide('none', 'stale_frame')
        if kind != 'frame':
            return self._decide('none', f'ignored:{kind}')
        return self._on_frame(observation, now)

    def _on_frame(self, frame, now):
        lights = frame.get('lights') or []
        if self.state in (IDLE, APPROACH):
            return self._decide('none', 'no_intersection_context')
        if self.state == WAITING:
            return self._evaluate_waiting(lights, now)
        if self.state == RESUMING:
            return self._decide('none', 'awaiting_quiet')
        return self._decide('none', self.state)

    # ---------- 路口进入：由桥在规划/行驶中调用 ----------
    def on_position(self, pose, goal=None, moving=True, now=None):
        """按位姿判断是否在受控路口范围；返回 Decision（限速或停车等待）。"""
        now = self.clock() if now is None else now
        if self.state in (WAITING, RESUMING):
            return self._decide('none', 'already_waiting')
        if self.state == ABORTED:
            return self._decide('none', 'aborted_until_reenable')
        near = self._nearest(pose) if pose else None
        if near is None:
            if self.state == APPROACH:
                self.state = IDLE
                self.active_intersection = None
            return self._decide('none', 'outside_intersection')
        if near.name in self._passed:
            return self._decide('none', 'already_passed')
        if self.state == IDLE:
            self.state = APPROACH
            self.active_intersection = near
            self._record('approach', intersection=near.name)
        return self._decide('limit', 'approach_intersection', speed_limit=near.speed_limit_near,
                            state=APPROACH)

    def on_lamp(self, color, track_id=None, passed_line=False, now=None):
        """桥在红灯/黄灯时调用：进入等待。返回 stop_and_hold 或 none。"""
        now = self.clock() if now is None else now
        if self.state in (WAITING, RESUMING):
            return self._decide('none', 'already_waiting')
        if self.state == ABORTED:
            return self._decide('none', 'aborted_until_reenable')
        intersection = self.active_intersection
        if intersection is None:
            # 尚未进入路口关联（普通路段看到远处灯）→ 只记录，不干预
            self.last_lamp = {'color': color, 'track_id': track_id}
            return self._decide('none', 'no_intersection')
        self.last_lamp = {'color': color, 'track_id': track_id}
        if color == GREEN:
            self._passed.add(intersection.name)
            self.state = PASSED
            return self._decide('none', 'green_proceed', state=PASSED)
        if color == YELLOW and passed_line:
            # 黄灯且已可靠越线：避障有效前提下低速驶离（桥负责避障有效判定）
            self._passed.add(intersection.name)
            self.state = PASSED
            self._record('yellow_passed_line', intersection=intersection.name)
            return self._decide('none', 'yellow_passed_line_slow', speed_limit=self.policy.resume_speed,
                                state=PASSED)
        # 红线前 / 无线数据的黄灯 / 不确定色 → 停车等待
        self.state = WAITING
        self._green_run = 0
        self._record('wait_begin', intersection=intersection.name, color=color)
        return self._decide('stop_and_hold', f'{color}_stop', state=WAITING)

    def arm_snapshot(self, goal, queue, boot_id='', epoch=''):
        """桥在取消前调用：保存任务快照。取消必须发生在此之后。"""
        if not isinstance(goal, dict) or any(k not in goal for k in ('x', 'y', 'yaw')):
            raise ValueError('快照目标必须含 x/y/yaw')
        self.snapshot = Snapshot(goal=dict(goal), queue=[dict(p) for p in (queue or [])],
                                 intersection=(self.active_intersection.name
                                               if self.active_intersection else ''),
                                 created_at=self.clock(), boot_id=boot_id, epoch=epoch)
        self._record('snapshot_armed', intersection=self.snapshot.intersection)
        return self.snapshot

    # ---------- 等待中评估恢复 ----------
    def _evaluate_waiting(self, lights, now):
        if self.snapshot is None:
            # 未持有快照却处于等待（异常）→ 直接中止，避免无依据恢复
            self.state = ABORTED
            self._record('abort_no_snapshot')
            return self._decide('abort', 'no_snapshot', state=ABORTED)
        if self._stream_lost_at is not None and now - self._stream_lost_at > self.policy.stream_lost_grace:
            # 断流超容差：保持等待（丢绿帧也不能恢复），但不清快照
            self._green_run = 0
            return self._decide('none', 'stream_unavailable_hold')
        lamp = self._associated_green(lights)
        if lamp is None:
            self._green_run = 0
            self.last_lamp = None
            return self._decide('none', 'not_stable_green')
        # 同一盏灯（按 track_id 或同框判定）
        if self.last_lamp and not self._same_lamp(lamp):
            self._green_run = 0
        self.last_lamp = lamp
        if now - self._last_green_at < self.policy.min_green_interval:
            return self._decide('none', 'green_frame_too_soon')
        self._last_green_at = now
        self._green_run += 1
        if self._green_run < self.policy.green_confirm_frames:
            return self._decide('none', f'green_run_{self._green_run}')
        # 稳定绿达成 → 进入 RESUMING，等桥确认旧终态与授权
        self.state = RESUMING
        self._record('stable_green', frames=self._green_run, lamp=lamp)
        return self._decide('none', 'stable_green_await_quiet', state=RESUMING)

    def _associated_green(self, lights):
        """只认「本车道」的绿：要求 pass_eligible 且有有效判色。"""
        candidates = []
        for light in lights:
            if not isinstance(light, dict):
                continue
            if light.get('color') != GREEN or not light.get('color_pass_eligible'):
                continue
            candidates.append(light)
        if not candidates:
            return None
        return {'track_id': candidates[0].get('track_id'),
                'xyxy': tuple(candidates[0].get('xyxy') or ())}

    def _same_lamp(self, lamp):
        if self.last_lamp is None:
            return True
        if lamp.get('track_id') is not None and self.last_lamp.get('track_id') is not None:
            return lamp['track_id'] == self.last_lamp['track_id']
        return lamp.get('xyxy') == self.last_lamp.get('xyxy')

    # ---------- 桥侧条件与恢复动作 ----------
    def can_resume(self, *, actions_quiet, authorized, ready, identity_ok, now=None):
        """桥查询完外部条件后调用；全真才允许一次性恢复。"""
        now = self.clock() if now is None else now
        if self.state != RESUMING or self.snapshot is None:
            return self._decide('none', f'not_resuming:{self.state}')
        missing = []
        if not actions_quiet:
            missing.append('old_action_terminal_unknown')
        if not authorized:
            missing.append('not_authorized')
        if not ready:
            missing.append('not_ready')
        if not identity_ok:
            missing.append('identity_mismatch')
        if missing:
            return self._decide('none', 'resume_blocked:' + ','.join(missing), state=RESUMING)
        snapshot, self.snapshot = self.snapshot, None
        self.state = PASSED
        if self.active_intersection is not None:
            self._passed.add(self.active_intersection.name)
            # 本轮路口已放行：清空活动路口，否则 _traffic_limited 会把恢复后的
            # 全部自动速度一直削到路口限速，车辆爬向 A 时被 Nav2 判无进展。
            self.active_intersection = None
        self._green_run = 0
        self._record('resume_dispatch', intersection=snapshot.intersection)
        return self._decide('resume', 'resume_same_goal_new_uuid',
                            resume_goal=snapshot.goal, resume_queue=snapshot.queue,
                            speed_limit=self.policy.resume_speed, state=PASSED)

    def abort(self, reason):
        """安全事件（取消/接管/急停/租约失效/断流超限）→ 丢弃快照，绿灯不自启。"""
        had_snapshot = self.snapshot is not None
        self.snapshot = None
        self._green_run = 0
        self.state = ABORTED
        self._record('abort', reason=reason, had_snapshot=had_snapshot)
        return self._decide('abort', reason, state=ABORTED)

    def reenable(self):
        """人工显式重新启用后才允许再次参与路口逻辑。"""
        if self.state == ABORTED:
            self.state = IDLE
            self._record('reenable')
        return self._decide('none', 'reenabled', state=self.state)

    # ---------- 输出 ----------
    def _decide(self, action, reason, speed_limit=None, resume_goal=None,
                resume_queue=None, state=None):
        return Decision(action=action, reason=reason, speed_limit=speed_limit,
                        resume_goal=resume_goal, resume_queue=resume_queue,
                        state=state or self.state)

    def snapshot_status(self):
        """页面三问所需：是否在等灯/哪盏灯/为什么禁动/现在能做什么。"""
        return {'state': self.state, 'intersection': (self.active_intersection.name
                                                      if self.active_intersection else None),
                'lamp': self.last_lamp, 'greenRun': self._green_run,
                'hasSnapshot': self.snapshot is not None,
                'snapshotGoal': dict(self.snapshot.goal) if self.snapshot else None,
                'snapshotQueueLength': len(self.snapshot.queue) if self.snapshot else 0,
                'streamLost': self._stream_lost_at is not None,
                'canUserDo': self._user_options()}

    def _user_options(self):
        if self.state == WAITING:
            return ['等待绿灯', '人工接管（放弃等待）', '取消任务']
        if self.state == RESUMING:
            return ['等待旧目标终态确认', '取消任务']
        if self.state == ABORTED:
            return ['重新启用导航']
        return []