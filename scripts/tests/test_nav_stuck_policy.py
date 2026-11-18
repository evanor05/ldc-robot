"""D：无位移失败策略的纯逻辑测试（不依赖 ROS/实车）。

现场事实：1612 行 No valid trajectories = 806 控制周期 × 2 logger，799/806 是
100% BaseObstacle，末位姿距最近 occupied 仅 0.07m。这类失败的特征是"根本没动"。
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav_stuck_policy import DEFAULTS, NoProgressPolicy  # noqa: E402


def policy(**options):
    return NoProgressPolicy(**options)


def test_unknown_option_is_rejected():
    with pytest.raises(ValueError):
        NoProgressPolicy(max_consecutive_typo=3)


def test_no_movement_failure_halts_queue_and_clears_costmap_once():
    stuck = policy()
    stuck.begin_goal(100.0, pose=(0.0, 0.0))
    stuck.observe_odom((0.02, 0.0))          # 贴墙抖动 2cm
    decision = stuck.decide(105.0, stuck.classify(105.0, (0.02, 0.0), 0, 'No valid trajectories'))
    assert decision['action'] == 'halt_queue'
    assert decision['clearCostmap'] is True
    assert decision['latched'] is False
    assert '无位移' in decision['reason'] and '贴墙' in decision['reason']
    # 冷却期内不重复清 costmap
    stuck.begin_goal(106.0, pose=(0.02, 0.0))
    again = stuck.decide(111.0, stuck.classify(111.0, (0.02, 0.0), 0, 'No valid trajectories'))
    assert again['clearCostmap'] is False
    assert again['action'] == 'lock'


def test_second_consecutive_failure_latches_stuck():
    stuck = policy()
    for start in (100.0, 200.0):
        stuck.begin_goal(start, pose=(0.0, 0.0))
        decision = stuck.decide(start + 5, stuck.classify(start + 5, (0.0, 0.0), 0, 'BaseObstacle'))
    assert decision['action'] == 'lock'
    assert decision['latched'] is True
    assert decision['consecutive'] == 2
    assert '连续 2 个目标无位移失败' in decision['reason']
    assert stuck.snapshot()['latched'] is True


def test_failure_with_movement_is_not_treated_as_stuck():
    """绕障失败/超时失败会有位移，不能触发停机策略。"""
    stuck = policy()
    stuck.begin_goal(100.0, pose=(0.0, 0.0))
    for step in range(1, 11):
        stuck.observe_odom((step * 0.1, 0.0))   # 移动 1m 后失败
    decision = stuck.decide(110.0, stuck.classify(110.0, (1.0, 0.0), 0, 'patience exceeded'))
    assert decision['action'] == 'fail'
    assert decision['clearCostmap'] is False
    assert stuck.snapshot()['latched'] is False


def test_short_failure_does_not_count_as_no_progress():
    """目标被立即拒绝（<3s）不算"卡死"，避免误锁。"""
    stuck = policy()
    stuck.begin_goal(100.0, pose=(0.0, 0.0))
    classification = stuck.classify(101.0, (0.0, 0.0), 0, 'Nav2 拒绝目标')
    assert classification['noProgress'] is False
    assert stuck.decide(101.0, classification)['action'] == 'fail'


def test_turning_in_place_counts_as_no_progress():
    """原地打转的累计里程很小，仍应判定为无位移（贴墙时常见）。"""
    stuck = policy()
    stuck.begin_goal(100.0, pose=(0.0, 0.0))
    for _ in range(30):
        stuck.observe_odom((0.001, 0.0))
    classification = stuck.classify(105.0, (0.001, 0.0), 0, 'No valid trajectories')
    assert classification['displacement'] < DEFAULTS['no_progress_distance_m']
    assert classification['noProgress'] is True


def test_success_resets_streak_and_manual_release_clears_latch():
    stuck = policy()
    stuck.begin_goal(100.0, pose=(0.0, 0.0))
    stuck.decide(105.0, stuck.classify(105.0, (0.0, 0.0), 0, 'x'))
    assert stuck.consecutive == 1
    stuck.on_success()
    assert stuck.consecutive == 0
    stuck.begin_goal(200.0, pose=(0.0, 0.0))
    stuck.decide(205.0, stuck.classify(205.0, (0.0, 0.0), 0, 'x'))
    stuck.begin_goal(300.0, pose=(0.0, 0.0))
    decision = stuck.decide(305.0, stuck.classify(305.0, (0.0, 0.0), 0, 'x'))
    assert decision['latched'] is True
    stuck.manual_release()
    assert stuck.latched is False and stuck.consecutive == 0


def test_classification_carries_thresholds_and_metrics():
    stuck = policy(no_progress_distance_m=0.05)
    stuck.begin_goal(100.0, pose=(1.0, 2.0))
    classification = stuck.classify(104.0, (1.02, 2.0), 0, 'BaseObstacle 100%')
    assert classification['thresholds']['distance'] == 0.05
    assert classification['displacement'] == pytest.approx(0.02)
    assert classification['duration'] == pytest.approx(4.0)
    assert classification['errorCode'] == 0
