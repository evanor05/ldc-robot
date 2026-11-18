"""无位移失败的停机策略（纯逻辑，零 ROS 依赖）。

现场证据（2026-11-11）：1612 行 `No valid trajectories` = 806 控制周期 × 2 logger，
799/806 是 100% BaseObstacle；机器人末位姿距最近 occupied 仅 0.07m —— 车已经贴墙、
在膨胀区里，控制器每一帧都被 BaseObstacle 否决。这类失败的共同特征是**根本没动**，
与"路径太远/超时/被拒"完全不同，必须区别对待：

- 单次"无位移失败" → 取消剩余队列、零速、**只清一次** costmap、给重定位/挪车提示；
- 连续 max_consecutive 次同样失败 → 置 stuck 锁存，拒绝新目标，等人工介入
  （不再让 UI 把 20 个目标一个个喂进去）；
- 有位移的失败（例如绕障失败）只按普通失败处理，不触发本策略。

只做判定与决策，不发任何 ROS 指令 —— 便于离线测试，也避免策略里藏副作用。
"""
from __future__ import annotations

import math

DEFAULTS = {
    # 小于这个直线/累计位移视为"没动过"（膨胀区内贴墙时通常 < 0.05m）
    'no_progress_distance_m': 0.15,
    # 太短的失败（目标被立即拒绝/取消）不参与判定
    'no_progress_min_seconds': 3.0,
    # 连续几次同样失败后彻底锁存
    'max_consecutive': 2,
    # 清 costmap 的最小间隔，避免失败瞬间反复调用服务
    'clear_costmap_cooldown_s': 10.0,
}


def _finite(value, fallback):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    return number if math.isfinite(number) else fallback


class NoProgressPolicy:
    """记录每个目标的位移，并在失败时给出停机决策。"""

    def __init__(self, **options):
        unknown = set(options) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'未知策略参数：{", ".join(sorted(unknown))}')
        self.options = {key: _finite(options.get(key, value), value) for key, value in DEFAULTS.items()}
        self.reset()

    def reset(self):
        self.consecutive = 0
        self.latched = False
        self.latch_reason = ''
        self.last_clear_at = 0.0
        self.goal_started_at = None
        self.current = None
        self.origin = None
        self.last_pose = None
        self.travelled = 0.0
        self.goals_seen = 0
        self.no_progress_failures = 0

    # ---- 目标生命周期 ----
    def begin_goal(self, at, pose=None, point=None):
        self.goal_started_at = at
        self.current = point
        self.origin = pose
        self.last_pose = pose
        self.travelled = 0.0
        self.goals_seen += 1

    def observe_odom(self, pose):
        """累计里程：贴墙时机器人可能微小抖动，累计路径比直线距离更能说明"动没动"。"""
        if pose is None or not all(math.isfinite(value) for value in pose[:2]):
            return
        if self.last_pose is not None:
            self.travelled += math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
        self.last_pose = pose

    def displacement(self, pose):
        """直线位移与累计里程取大者，避免原地打转被当成"动了"。"""
        straight = 0.0
        if self.origin is not None and pose is not None:
            straight = math.hypot(pose[0] - self.origin[0], pose[1] - self.origin[1])
        return max(straight, self.travelled)

    # ---- 失败判定 ----
    def classify(self, at, pose=None, error_code=0, message=''):
        duration = (at - self.goal_started_at) if self.goal_started_at is not None else 0.0
        moved = self.displacement(pose)
        no_progress = (moved < self.options['no_progress_distance_m'] and
                       duration >= self.options['no_progress_min_seconds'])
        return {
            'noProgress': bool(no_progress),
            'displacement': round(moved, 3),
            'duration': round(duration, 2),
            'errorCode': error_code,
            'message': message,
            'thresholds': {
                'distance': self.options['no_progress_distance_m'],
                'minSeconds': self.options['no_progress_min_seconds'],
            },
        }

    def decide(self, at, classification):
        """根据分类结果给出动作：fail / halt_queue / lock。"""
        if not classification.get('noProgress'):
            # 有位移的失败：普通失败，但连续计数归零（不是同一个卡死问题）
            self.consecutive = 0
            return {'action': 'fail', 'clearCostmap': False, 'latched': self.latched,
                    'consecutive': self.consecutive, 'reason': '', 'classification': classification}
        self.consecutive += 1
        self.no_progress_failures += 1
        clear = (at - self.last_clear_at) >= self.options['clear_costmap_cooldown_s']
        if clear:
            self.last_clear_at = at
        if self.consecutive >= int(self.options['max_consecutive']):
            self.latched = True
            self.latch_reason = (f'连续 {self.consecutive} 个目标无位移失败（最近一次位移 '
                                 f"{classification['displacement']:.2f}m / "
                                 f"{classification['duration']:.1f}s）：车辆可能贴墙或在膨胀区内")
            return {'action': 'lock', 'clearCostmap': clear, 'latched': True,
                    'consecutive': self.consecutive, 'reason': self.latch_reason,
                    'classification': classification}
        reason = (f"目标无位移失败（位移 {classification['displacement']:.2f}m / "
                  f"{classification['duration']:.1f}s，错误：{classification['message'] or '未提供'}）："
                  '车辆可能贴墙或在膨胀区内，请手动挪开或重新设置初始位置后再试')
        return {'action': 'halt_queue', 'clearCostmap': clear, 'latched': False,
                'consecutive': self.consecutive, 'reason': reason, 'classification': classification}

    # ---- 人工介入 ----
    def on_success(self):
        self.consecutive = 0

    def manual_release(self):
        """人工显式接管/复位后解除锁存（stuck 只能由人解除）。"""
        self.latched = False
        self.latch_reason = ''
        self.consecutive = 0

    def snapshot(self):
        return {
            'consecutive': self.consecutive,
            'latched': self.latched,
            'latchReason': self.latch_reason,
            'noProgressFailures': self.no_progress_failures,
            'goalsSeen': self.goals_seen,
            'options': dict(self.options),
        }
