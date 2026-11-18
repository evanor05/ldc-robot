"""上车自检的频率检查（check_topic_rate）离线测试。

背景（真实缺陷）：`ros2 topic hz` 是持续采样命令，只能靠超时结束。
此前的实现超时后把已采集 stdout 丢掉、并把「超时」当成失败，
导致 /scan、/odom 在任何情况下都会被判「没有采样到数据」——
自检工具对 ROS 路径永远报 FAIL。这里用假 ros2 覆盖四种真实情形。
"""
from __future__ import annotations

import os
import stat
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import preflight_field as preflight  # noqa: E402


def fake_ros2(tmp_path: Path, body: str) -> Path:
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir(exist_ok=True)
    script = bin_dir / 'ros2'
    script.write_text('#!/usr/bin/env bash\n' + body + '\n', 'utf-8')
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    return bin_dir


@pytest.fixture
def use_fake(tmp_path, monkeypatch):
    def install(body: str):
        bin_dir = fake_ros2(tmp_path, body)
        monkeypatch.setenv('PATH', f'{bin_dir}:{os.environ["PATH"]}')
        return bin_dir
    return install


def test_持续采样被超时结束时要保留已采集输出(use_fake):
    """正常情况：hz 不会自己退出，超时是预期结果，必须用已采集输出判频率。"""
    use_fake('while true; do echo "average rate: 12.34"; sleep 0.05; done')
    result = preflight.check_topic_rate('/scan', 1.0, timeout=0.6)
    assert result['ok'] is True, result['detail']
    assert result['rate'] == pytest.approx(12.34)
    assert result['exitCode'] == 124, '超时结束应记录为 124'


def test_频率低于要求时判失败(use_fake):
    use_fake('while true; do echo "average rate: 0.20"; sleep 0.05; done')
    result = preflight.check_topic_rate('/odom', 1.0, timeout=0.6)
    assert result['ok'] is False
    assert result['rate'] == pytest.approx(0.2)
    assert '0.2 Hz' in result['detail']


def test_完全没有输出时说明原因而不是空话(use_fake):
    use_fake('sleep 5')     # 采样窗口内一个字都没打印
    result = preflight.check_topic_rate('/scan', 1.0, timeout=0.4)
    assert result['ok'] is False
    assert result['rate'] is None
    assert '没有读到频率' in result['detail']
    assert result['exitCode'] == 124


def test_命令非零退出时带上错误输出(use_fake):
    use_fake('echo "boom: no such topic" >&2; exit 2')
    result = preflight.check_topic_rate('/scan', 1.0, timeout=2)
    assert result['ok'] is False
    assert result['exitCode'] == 2
    assert 'boom' in result['detail']


def test_话题列表缺失时报出缺哪个(use_fake):
    use_fake('printf "/scan\\n/tf\\n/tf_static\\n"; exit 0')
    result = preflight.check_ros_environment()
    assert result['ok'] is False
    assert '/odom' in result['detail']


def test_话题齐全时通过(use_fake):
    use_fake('printf "/scan\\n/odom\\n/tf\\n"; exit 0')
    result = preflight.check_ros_environment()
    assert result['ok'] is True
    assert result['topics'] == ['/odom', '/scan', '/tf']


def test_多次采样要取最后一次而不是冷启动读数(use_fake):
    """hz 每个窗口打一条；最早的可能是刚启动时的偏低读数。"""
    use_fake('echo "average rate: 0.10"; sleep 0.1; echo "average rate: 20.00"; while true; do sleep 0.05; done')
    result = preflight.check_topic_rate('/scan', 1.0, timeout=0.6)
    assert result['ok'] is True, result['detail']
    assert result['rate'] == pytest.approx(20.0), '应采用最新一条采样'


def test_命令异常退出时不得凭残留频率判通过(use_fake):
    """打印过一条好看的 rate 之后异常退出：输出不可信，必须判失败。"""
    use_fake('echo "average rate: 50.00"; echo "rcl error: topic vanished" >&2; exit 3')
    result = preflight.check_topic_rate('/scan', 1.0, timeout=2)
    assert result['ok'] is False
    assert result['exitCode'] == 3
    assert '异常退出' in result['detail'] and 'rcl error' in result['detail']
