"""上车自检脚本的离线测试（--skip-ros 路径，不碰 ROS）。"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / 'scripts' / 'preflight_field.py'
MAPS = REPO / 'maps'
GOOD = MAPS / 'map_20261111165629_640'
BAD = MAPS / 'map_20261111180316_294'


def run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args],
                          capture_output=True, text=True, timeout=120, cwd=REPO)


def test_good_map_passes_and_reports_thresholds():
    result = run('--skip-ros', '--map', str(GOOD), '--json')
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['ok'] is True
    assert report['checks']['map']['ok'] is True
    # 门槛必须与 ros_config.json 的 save_map 同源
    assert report['checks']['map']['thresholds']['minOccupiedCells'] == 2500
    assert report['checks']['map']['thresholds']['maxUnknownRatio'] == 0.85
    assert report['checks']['map']['quality']['occupied'] == 3348


def test_bad_map_fails_with_reason():
    result = run('--skip-ros', '--map', str(BAD))
    assert result.returncode == 1
    assert '1717' in result.stdout
    assert '未通过' in result.stdout


def test_missing_map_fails():
    result = run('--skip-ros', '--map', str(MAPS / 'map_does_not_exist_000000'), '--json')
    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['checks']['map']['ok'] is False
    assert '不存在' in report['checks']['map']['detail']


def test_no_checks_is_a_usage_error_not_a_pass():
    result = run('--skip-ros')
    assert result.returncode == 2, '什么都不检查时必须报用法错误，不能显示通过'


def test_failed_checks_are_listed():
    result = run('--skip-ros', '--map', str(BAD), '--json')
    report = json.loads(result.stdout)
    assert report['failed'] == ['map']
