"""对抗性证据审查：上车自检会不会"误放行"？

决策负责人点名的 5 种误放行风险，这里逐个固定成测试：
话题存在但 TF 断链、Nav2 未就绪、清 costmap 服务名不匹配、命令异常退出却含旧 rate、
地图合格但机器人数据链路坏了。核心断言是：**能跑 ≠ 该跑**，任一项不成立就必须 FAIL。
"""
from __future__ import annotations

import json
import os
import stat
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import preflight_field as preflight  # noqa: E402

MAPS = Path(__file__).resolve().parents[2] / 'maps'
GOOD_MAP = MAPS / 'map_20261111165629_640'

FAKE_ROS2 = r'''#!/usr/bin/env bash
key="$1 $2"
case "$key" in
  "topic list")
    printf "%b" "${FAKE_TOPICS:-/scan\n/odom\n/tf\n}"; exit 0 ;;
  "topic hz")
    [ -n "${FAKE_HZ:-}" ] && printf "%b" "$FAKE_HZ"
    [ "${FAKE_HZ_HANG:-1}" = "1" ] && while true; do sleep 0.05; done
    exit "${FAKE_HZ_EXIT:-0}" ;;
  "run tf2_ros")
    [ -n "${FAKE_TF:-}" ] && printf "%b" "$FAKE_TF"
    [ "${FAKE_TF_HANG:-1}" = "1" ] && sleep 5
    exit "${FAKE_TF_EXIT:-0}" ;;
  "lifecycle get")
    echo "${FAKE_LIFECYCLE:-active [3]}"; exit "${FAKE_LIFECYCLE_EXIT:-0}" ;;
  "service list")
    printf "%b" "${FAKE_SERVICES:-/local_costmap/clear_entirely_local_costmap\n/global_costmap/clear_entirely_global_costmap\n}"
    exit 0 ;;
esac
exit 0
'''

TF_OK = 'At time 1.0\n- Translation: [0.0, 0.0, 0.0]\n'
TF_BROKEN = 'Exception while looking up transform: "base_footprint" passed to lookupTransform argument source_frame does not exist.\n'
HZ_OK = 'average rate: 12.5\n'


@pytest.fixture
def fake_ros(tmp_path, monkeypatch):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    script = bin_dir / 'ros2'
    script.write_text(FAKE_ROS2, 'utf-8')
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv('PATH', f'{bin_dir}:{os.environ["PATH"]}')
    monkeypatch.setenv('FAKE_TF', TF_OK)
    monkeypatch.setenv('FAKE_HZ', HZ_OK)
    # 默认让假命令立即返回（带有效输出），避免每条用例都等 8s/6s 超时；
    # "超时结束采样"这条路径已在 test_preflight_rate.py 单独覆盖。
    monkeypatch.setenv('FAKE_TF_HANG', '0')
    monkeypatch.setenv('FAKE_HZ_HANG', '0')
    monkeypatch.delenv('FAKE_TOPICS', raising=False)
    monkeypatch.delenv('FAKE_LIFECYCLE', raising=False)
    return monkeypatch


def run_json(capsys, *args):
    code = preflight.main(list(args))
    output = capsys.readouterr().out
    return code, json.loads(output)


def test_topics_and_rates_ok_but_tf_broken_must_fail(fake_ros, capsys, tmp_path):
    """最典型的误放行：/scan、/odom 频率正常、地图也合格，但 TF 链断了。"""
    fake_ros.setenv('FAKE_TF', TF_BROKEN)
    fake_ros.setenv('FAKE_TF_HANG', '0')
    fake_ros.setenv('FAKE_TF_EXIT', '1')
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--json')
    assert code == 1, 'TF 断链时绝不能判通过'
    assert report['ok'] is False
    assert any(name.startswith('tf ') for name in report['failed']), report['failed']
    # 话题与地图本身是好的，说明失败来自 TF 而不是别的原因
    assert report['checks']['scan']['ok'] is True
    assert report['checks']['map']['ok'] is True


def test_tf_ok_passes_with_good_map(fake_ros, capsys):
    fake_ros.setenv('FAKE_TF_HANG', '0')
    fake_ros.setenv('FAKE_TF_EXIT', '0')
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--json')
    assert code == 0, report['failed']
    assert report['ok'] is True
    assert report['checks']['tf odom->base_footprint']['ok'] is True


def test_nav2_inactive_is_warn_without_require_nav(fake_ros, capsys):
    fake_ros.setenv('FAKE_LIFECYCLE', 'inactive [2]')
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--json')
    assert code == 0, '只做健康检查时 Nav2 未起不应算失败'
    assert report['warned'], '但必须显式告警'
    assert any('nav2' in name for name in report['warned'])


def test_nav2_inactive_fails_when_require_nav(fake_ros, capsys):
    fake_ros.setenv('FAKE_LIFECYCLE', 'inactive [2]')
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--require-nav', '--json')
    assert code == 1, '要跑 C/D 验收时 Nav2 未 active 必须判失败'
    assert any('nav2' in name for name in report['failed'])


def test_missing_clear_services_fail_when_require_nav(fake_ros, capsys):
    fake_ros.setenv('FAKE_SERVICES', '/amcl/get_parameters\n')
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--require-nav', '--json')
    assert code == 1
    assert 'clear services' in report['failed']
    assert len(report['checks']['clear services']['missing']) == 2


def test_clear_services_present_pass(fake_ros, capsys):
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--require-nav', '--json')
    assert code == 0, report['failed']
    assert report['checks']['clear services']['ok'] is True


def test_missing_scan_topic_blocks_even_with_good_map(fake_ros, capsys):
    fake_ros.setenv('FAKE_TOPICS', '/odom\n/tf\n')
    code, report = run_json(capsys, '--map', str(GOOD_MAP), '--json')
    assert code == 1
    assert 'ros' in report['failed']
    assert '/scan' in report['checks']['ros']['detail']
