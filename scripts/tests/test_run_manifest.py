"""run_manifest 的离线测试：不依赖 ROS、不依赖实车。"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_manifest as rm  # noqa: E402


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_RUN_ACTIVE_MARKER', str(tmp_path / '.run-active'))
    return tmp_path


def test_start_writes_manifest_with_baseline_sources_and_params(tmp_path, isolated):
    runs = tmp_path / 'runs'
    manifest = rm.start_run('navigation', {'map': '/maps/map_20261111165629_640.yaml',
                                          'cmd_vel_output': 'cmd_vel_auto'},
                            run_id='run_test_001', runs_dir=runs)
    path = runs / 'run_test_001' / rm.MANIFEST_NAME
    assert path.is_file()
    stored = json.loads(path.read_text('utf-8'))
    assert stored['runId'] == 'run_test_001'
    assert stored['kind'] == 'navigation'
    assert stored['effectiveParams']['cmd_vel_output'] == 'cmd_vel_auto'
    # 基线不能只写一个 HEAD：必须有 dirty paths 与逐文件 SHA256
    assert 'head' in stored['git'] and 'dirtyPaths' in stored['git']
    assert stored['sources'], '必须记录参与运行的源码'
    server = next(item for item in stored['sources'] if item['path'] == 'web_console/server.js')
    assert server['exists'] is True
    assert len(server['sha256']) == 64
    assert server['sha256'] == hashlib.sha256(
        (rm.REPO_ROOT / 'web_console/server.js').read_bytes()).hexdigest()
    # 地图门槛必须与统一入口同源
    assert stored['mapThresholds']['minOccupiedCells'] == 2500
    assert stored['mapThresholds']['maxUnknownRatio'] == 0.85
    assert (isolated / '.run-active').read_text('utf-8') == 'run_test_001'


def test_finish_writes_summary_pair_and_clears_marker(tmp_path):
    runs = tmp_path / 'runs'
    rm.start_run('mapping', {'mode': 'auto'}, run_id='run_test_002', runs_dir=runs)
    record = rm.finish_run('run_test_002', 'failed', '地图内容不足（1717 栅格 / 86.1% 未知）',
                           {'goalsAttempted': 1, 'baseObstacle': 806}, runs_dir=runs)
    run_dir = runs / 'run_test_002'
    assert (run_dir / rm.SUMMARY_JSON).is_file()
    assert (run_dir / rm.SUMMARY_MD).is_file()
    summary = json.loads((run_dir / rm.SUMMARY_JSON).read_text('utf-8'))
    assert summary['runId'] == 'run_test_002'
    assert summary['status'] == 'failed'
    assert summary['summary']['baseObstacle'] == 806
    assert record['durationSeconds'] is not None
    markdown = (run_dir / rm.SUMMARY_MD).read_text('utf-8')
    assert 'run_test_002' in markdown
    assert '1717 栅格' in markdown
    assert '## 生效参数' in markdown
    # 结束后必须清掉运行标记，避免"运行中禁止改前端"的禁令永久生效
    assert not (tmp_path / '.run-active').exists()


def test_failed_run_keeps_snapshot_for_localization(tmp_path):
    runs = tmp_path / 'runs'
    rm.start_run('navigation', {}, run_id='run_test_003', runs_dir=runs)
    snapshot = {'localization': {'ok': False, 'detail': 'AMCL 报 future TF'},
                'requestedStamp': '2026-11-11T18:04:29+08:00', 'lastPoseStamp': None}
    rm.finish_run('run_test_003', 'timeout', 'localization_confirmed 未置位',
                  snapshots={'localization-at-failure': snapshot}, runs_dir=runs)
    path = runs / 'run_test_003' / 'checks' / 'localization-at-failure.json'
    assert path.is_file()
    assert json.loads(path.read_text('utf-8'))['localization']['ok'] is False
    summary = json.loads((runs / 'run_test_003' / rm.SUMMARY_JSON).read_text('utf-8'))
    assert summary['snapshots'] == [str(path)]


def test_cli_start_finish_and_show(tmp_path, capsys):
    runs = tmp_path / 'runs'
    assert rm.main(['start', '--kind', 'bench', '--param', 'max_vel_x=0.12',
                    '--run-id', 'run_cli_001', '--runs-dir', str(runs)]) == 0
    assert 'run_cli_001' in capsys.readouterr().out
    assert rm.main(['finish', 'run_cli_001', '--status', 'aborted',
                    '--reason', '人工中止', '--runs-dir', str(runs)]) == 0
    capsys.readouterr()
    assert rm.main(['show', 'run_cli_001', '--runs-dir', str(runs)]) == 0
    out = capsys.readouterr().out
    assert 'run_cli_001' in out and 'aborted' in out


def test_latest_run_filters_by_kind(tmp_path):
    runs = tmp_path / 'runs'
    rm.start_run('navigation', {}, run_id='run_a_navigation', runs_dir=runs)
    rm.start_run('mapping', {}, run_id='run_b_mapping', runs_dir=runs)
    assert rm.latest_run('mapping', runs_dir=runs) == 'run_b_mapping'
    assert rm.latest_run(runs_dir=runs) == 'run_b_mapping'


def test_watched_sources_cover_frontend_and_policy(tmp_path, isolated):
    """manifest 必须覆盖真正决定运行行为的前端组件与车端策略模块。

    历史缺口：WATCHED_SOURCES 只有 13 个后端/脚本条目，App.vue、CameraView.vue、
    NavigationControl.vue、MappingControl.vue、SlamMap.vue、nav_stuck_policy.py
    全都不在，导致"运行中换过前端或策略"无法被证据发现。
    """
    required = [
        'web_console/src/App.vue',
        'web_console/src/components/NavigationControl.vue',
        'web_console/src/components/MappingControl.vue',
        'web_console/src/components/CameraView.vue',
        'web_console/src/components/SlamMap.vue',
        'web_console/src/composables/useFoxglove.js',
        'web_console/visionRoutes.js',
        'scripts/nav_stuck_policy.py',
        'scripts/sensor_health.py',
        'scripts/navigation_profile.py',
        'scripts/run_manifest.py',
    ]
    for path in required:
        assert path in rm.WATCHED_SOURCES, f'{path} 未纳入 manifest 监控'
        assert (rm.REPO_ROOT / path).is_file(), f'{path} 声明了但文件不存在'
    # 不能反过来把不存在的文件塞进监控清单（会让每次运行都带 error 条目）
    assert all((rm.REPO_ROOT / path).is_file() for path in rm.WATCHED_SOURCES)
