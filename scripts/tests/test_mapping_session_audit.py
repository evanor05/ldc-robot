"""建图会话对账脚本的测试：全部用合成 fixture，不读真实日志。"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import audit_mapping_sessions as audit  # noqa: E402

# 与现场一致的行形态（两个 logger 各写一行，同一控制周期）
TRAJECTORY_TEMPLATE = (
    "[controller_server-2] [{level}] [{ts:.9f}] [{source}]: "
    "No valid trajectories out of {count}!\n"
)
OBSTACLE_TEMPLATE = "[controller_server-2] [ERROR] [{ts:.9f}] [DWBLocalPlanner]: 1.00: BaseObstacle/Trajectory Hits Obstacle.\n"


def session(root: Path, name: str, lines: str = '', product: bool = False) -> Path:
    directory = root / name
    directory.mkdir(parents=True)
    (directory / 'console.log').write_text(lines, 'utf-8')
    if product:
        (directory / 'map_20261111180000_000.yaml').write_text('image: map_20261111180000_000.pgm\n', 'utf-8')
        (directory / 'map_20261111180000_000.pgm').write_text('P5\n1 1\n255\n\x00', 'utf-8')
    return directory


def trajectory_block(cycles: int, count: int = 1229) -> str:
    text = ''
    for index in range(cycles):
        stamp = 1794390000.0 + index
        text += TRAJECTORY_TEMPLATE.format(level='ERROR', ts=stamp, source='DWBLocalPlanner', count=count)
        text += TRAJECTORY_TEMPLATE.format(level='WARN', ts=stamp + 0.001, source='controller_server', count=count)
        text += OBSTACLE_TEMPLATE.format(ts=stamp)
    return text


def test_eight_sessions_seven_without_product(tmp_path):
    root = tmp_path / 'runs'
    for index in range(7):
        session(root, f'20261111-1736{index:02d}-real', 'mapping_started 建图\n')
    session(root, '20261111-180000-real', 'mapping_started 建图\nmap_saved\n', product=True)
    report = audit.audit([root])
    assert report['summary']['total'] == 8
    assert report['summary']['withProduct'] == 1
    assert report['summary']['withoutProduct'] == 7
    assert report['summary']['byKind']['mapping'] == 8


def test_duplicate_logs_do_not_double_count(tmp_path):
    root = tmp_path / 'runs'
    directory = session(root, '20261111-173615-real', trajectory_block(2))
    copy = directory / 'console-copy.log'
    copy.write_bytes((directory / 'console.log').read_bytes())     # 内容完全相同
    report = audit.audit([root])
    item = report['sessions'][0]
    assert item['duplicateLogsDropped'] == 1
    # 4 行（2 周期 × 2 logger），去重后仍是 4 行而不是 8 行
    assert item['noValidTrajectory']['lines'] == 4
    assert item['noValidTrajectory']['estimatedControlCycles'] == 2


def test_no_valid_trajectory_lines_are_not_failures(tmp_path):
    root = tmp_path / 'runs'
    session(root, '20261111-173615-real', trajectory_block(3))
    metrics = audit.audit([root])['sessions'][0]['noValidTrajectory']
    assert metrics['lines'] == 6, '3 个周期 × 2 个 logger = 6 行'
    assert metrics['loggerSources'] == ['DWBLocalPlanner', 'controller_server']
    assert metrics['estimatedControlCycles'] == 3, '周期数 = 行数 / logger 种类数'
    assert metrics['baseObstacleHits'] == 3
    assert metrics['baseObstacleHundredPercent'] == 3
    assert metrics['cycleCountCrossChecked'] is True
    assert 'failures' not in metrics, '不得把行数暴露成失败次数'
    assert '不是失败次数' in metrics['note']


def test_unknown_terminal_is_not_guessed(tmp_path):
    root = tmp_path / 'runs'
    session(root, 'a-session', 'mapping_started 建图\n')
    session(root, 'b-session', 'mapping_started 建图\nmap_saved name=map_x\n')
    report = audit.audit([root])
    terminals = {item['session']: item['terminal'] for item in report['sessions']}
    assert terminals['a-session'] == 'unknown'
    assert terminals['b-session'] == 'saved'


def test_old_session_without_manifest_is_not_backfilled(tmp_path):
    root = tmp_path / 'runs'
    directory = session(root, 'old-session', 'mapping_started\n')
    audit.audit([root])
    assert not (directory / 'manifest.json').exists(), '脚本不得补造 manifest'
    assert audit.audit([root])['sessions'][0]['hasManifest'] is False


def test_input_order_stable_and_sources_untouched(tmp_path):
    root = tmp_path / 'runs'
    first = session(root, 'b-session', trajectory_block(1))
    second = session(root, 'a-session', 'mapping_started\n', product=True)
    other = tmp_path / 'other'
    session(other, 'c-session', '导航启动 startup_begin\n')

    before = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in list(root.rglob('*')) + list(other.rglob('*')) if path.is_file()}
    report_a = audit.audit([root, other])
    report_b = audit.audit([other, root])
    after = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in before}

    assert json.dumps(report_a, sort_keys=True) == json.dumps(report_b, sort_keys=True), \
        '输入顺序不得影响输出'
    assert before == after, '审计必须只读，不得改动任何日志'
    assert [item['session'] for item in report_a['sessions']] == \
        ['a-session', 'b-session', 'c-session']
    kinds = {item['session']: item['kind'] for item in report_a['sessions']}
    assert kinds['c-session'] == 'navigation'
    assert kinds['a-session'] == 'mapping'


def test_missing_roots_are_ignored_not_fatal(tmp_path):
    report = audit.audit([tmp_path / 'does-not-exist'])
    assert report['sessions'] == []
    assert report['summary']['total'] == 0
