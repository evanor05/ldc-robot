#!/usr/bin/env python3
"""上车前自检：一条命令回答"现在能不能跑导航验证"。

为什么需要它：C（初始位姿/定位确认）与 D（无位移保护）都只能在实车上验收，
而现场最容易浪费时间的不是代码，是"车端到底有没有 /scan、/odom、TF、Nav2"。
本脚本把这些前置条件检查固化成可重复、可留证的一步：

  python3 scripts/preflight_field.py --map maps/map_20261111165629_640
  python3 scripts/preflight_field.py --skip-ros --map ...   # 只验地图质量（离线可用）

检查项：
  1. ROS 环境可用（ros2 命令 + /scan、/odom、/tf 话题存在）
  2. /scan 与 /odom 有实际数据（ros2 topic hz 采样，低于阈值记 FAIL）
  3. 地图质量：直接复用 web_console/mapStorage.js 的 inspectMapQuality，
     门槛读 ros_config.json 的 save_map —— 与后端/前端同一真源，不重写判断逻辑
  4. 汇总输出 JSON（可给 run_manifest 当证据），任一 FAIL 则退出码非 0

只用标准库；不依赖实车也能跑 --skip-ros。
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MAP_STORAGE = REPO_ROOT / 'web_console' / 'mapStorage.js'


def run(command, timeout=10, cwd=None, env=None):
    """执行命令，返回 (returncode, stdout, stderr)。超时视为失败。

    stdout 与 stderr 必须分开：node 的告警写在 stderr，混进 stdout 会让
    JSON 解析取到半行（实测踩过，别合并）。
    """
    environment = dict(os.environ)
    if env:
        environment.update(env)
    try:
        result = subprocess.run(command, cwd=cwd or REPO_ROOT, capture_output=True, text=True,
                                timeout=timeout, check=False, env=environment)
    except FileNotFoundError:
        return 127, '', f'命令不存在: {command[0]}'
    except subprocess.TimeoutExpired as exc:
        # 关键：ros2 topic hz 是"持续采样"型命令，只能靠超时结束。
        # 超时那一刻已经采到的输出必须保留，否则永远读不到 average rate。
        partial = exc.stdout or ''
        if isinstance(partial, bytes):
            partial = partial.decode('utf-8', 'replace')
        return 124, partial, f'命令超时({timeout}s，已保留已采集输出): {" ".join(command)}'
    return result.returncode, result.stdout or '', result.stderr or ''


def check_ros_environment():
    if not shutil.which('ros2'):
        return {'ok': False, 'detail': '未找到 ros2 命令（需要 source /opt/ros/<distro>/setup.bash）'}
    code, output, errors = run(['ros2', 'topic', 'list'], timeout=15)
    if code != 0:
        return {'ok': False, 'detail': f'ros2 topic list 失败: {(errors or output).strip()[-200:]}'}
    topics = {line.strip() for line in output.splitlines() if line.strip().startswith('/')}
    required = ('/scan', '/odom', '/tf')
    missing = [name for name in required if name not in topics]
    return {'ok': not missing, 'detail': '' if not missing else f'缺少话题: {"、".join(missing)}',
            'topics': sorted(t for t in topics if t in required)}


def check_topic_rate(topic, minimum, timeout=8):
    """ros2 topic hz 采样一次；没有数据就是 FAIL（比"话题存在"更重要）。

    注意：hz 不会自己退出，正常情况下我们**用超时来结束采样**，
    所以 code==124 是预期结果而不是失败；判定只看有没有解出 average rate。
    """
    code, output, errors = run(['ros2', 'topic', 'hz', topic, '--window', '10'],
                               timeout=timeout, env={'PYTHONUNBUFFERED': '1'})
    # 多次采样取最后一次：hz 每个窗口打一条，最早的可能是刚启动时的冷启动读数。
    rates = re.findall(r'average rate:\s*([0-9.]+)', output)
    match = rates[-1] if rates else None
    if match is None:
        tail = (output or errors or '').strip().splitlines()[-2:]
        return {'ok': False, 'rate': None,
                'detail': (f'{topic} 在 {timeout}s 内没有读到频率（退出码 {code}）'
                           + (f'；输出尾部：{" / ".join(tail)}' if tail else '；无任何输出')),
                'exitCode': code}
    if code not in (0, 124):
        # 命令异常退出时，输出里可能残留更早一次采样的 rate；不能凭它判通过。
        tail = (errors or output).strip().splitlines()[-2:]
        return {'ok': False, 'rate': None, 'exitCode': code,
                'detail': (f'ros2 topic hz 异常退出（码 {code}），输出不可信'
                           + (f'；尾部：{" / ".join(tail)}' if tail else ''))}
    rate = float(match)
    return {'ok': rate >= minimum, 'rate': rate, 'exitCode': code,
            'detail': f'{topic} {rate:.1f} Hz（要求 ≥{minimum}）'}


def check_tf_chain(pairs=(('odom', 'base_footprint'),), timeout=6):
    """TF 链是否真的可用。

    这是最容易"误放行"的一项：/scan、/odom、/tf 话题都在、频率也正常，
    但 odom→base_footprint 断了，导航照样起不来。话题存在 ≠ TF 可用。
    """
    results = {}
    for parent, child in pairs:
        code, output, errors = run(['ros2', 'run', 'tf2_ros', 'tf2_echo', parent, child],
                                   timeout=timeout, env={'PYTHONUNBUFFERED': '1'})
        text = f'{output}\n{errors}'
        key = f'{parent}->{child}'
        if 'Invalid frame' in text or 'does not exist' in text:
            first = next((line for line in text.splitlines() if 'frame' in line.lower()), '')
            results[key] = {'ok': False, 'detail': f'{key} 不存在：{first.strip()[:120]}',
                            'exitCode': code}
        elif 'At time' in text or 'Translation' in text:
            results[key] = {'ok': True, 'detail': f'{key} 可用', 'exitCode': code}
        else:
            tail = ' / '.join(text.strip().splitlines()[-2:])
            results[key] = {'ok': False, 'exitCode': code,
                            'detail': f'{key} 在 {timeout}s 内没有输出变换'
                                      + (f'（{tail[:140]}）' if tail else '')}
    return results


def check_nav2_state(nodes=('/controller_server', '/planner_server', '/amcl'), timeout=6):
    """Nav2 生命周期是否 active。导航没起来时不算失败，但"要跑 C/D 验收"时必须 active。"""
    results = {}
    for node in nodes:
        code, output, errors = run(['ros2', 'lifecycle', 'get', node], timeout=timeout)
        text = (output or errors or '').strip()
        state = text.splitlines()[-1] if text else ''
        # 必须按前缀判定：'inactive [2]' 里也含 active，子串匹配会把未激活当成就绪
        # （本条由 test_preflight_false_pass 抓到，属典型误放行）。
        is_active = state.strip().lower().startswith('active')
        results[node] = {
            'ok': is_active,
            'state': state[:80],
            'detail': (f'{node} 状态：{state[:60]}' if state else f'{node} 无响应（退出码 {code}）'),
            'exitCode': code,
        }
    return results


def check_clear_services(timeout=15):
    """D 的清 costmap 依赖两个固定服务名；名字对不上时只会静默跳过。"""
    required = ('/local_costmap/clear_entirely_local_costmap',
                '/global_costmap/clear_entirely_global_costmap')
    code, output, errors = run(['ros2', 'service', 'list'], timeout=timeout)
    names = {line.strip() for line in (output or '').splitlines() if line.strip()}
    missing = [name for name in required if name not in names]
    return {'ok': not missing,
            'found': [name for name in required if name in names],
            'missing': missing,
            'detail': ('两个清 costmap 服务均存在' if not missing
                       else f'缺少 {len(missing)} 个清 costmap 服务：{"、".join(missing)}'),
            'exitCode': code}


def check_map_quality(prefix: Path):
    """通过 node 调用 mapStorage，保持与 save_map/导航门禁同一套门槛。"""
    if not MAP_STORAGE.is_file():
        return {'ok': False, 'detail': f'缺少 {MAP_STORAGE}'}
    if not shutil.which('node'):
        return {'ok': False, 'detail': '未找到 node（地图质量校验需要它复用后端逻辑）'}
    script = (
        "import(" + json.dumps(str(MAP_STORAGE)) + ").then(m=>{"
        "const cfg=JSON.parse(require('fs').readFileSync(" +
        json.dumps(str(REPO_ROOT / 'web_console' / 'ros_config.json')) + ",'utf8'));"
        "const gate=m.qualityOptions(cfg.save_map||{});"
        "const r=m.inspectMapQuality(" + json.dumps(str(prefix)) + ",gate);"
        "console.log(JSON.stringify({navigable:r.navigable,gate,quality:r.quality,error:r.error}));"
        "}).catch(e=>{console.error(e.message);process.exit(1)})"
    )
    code, output, errors = run(['node', '--input-type=commonjs', '-e', script], timeout=20,
                               env={'NODE_NO_WARNINGS': '1'})
    if code != 0:
        return {'ok': False, 'detail': f'地图质量校验失败: {(errors or output).strip()[-300:]}'}
    try:
        payload = json.loads(output.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {'ok': False, 'detail': f'无法解析质量结果: {(output or errors).strip()[-200:]}'}
    gate = payload['gate']
    return {
        'ok': bool(payload['navigable']),
        'detail': (f"可导航（占用下限 {gate['minOccupiedCells']}，未知上限 "
                   f"{gate['maxUnknownRatio'] * 100:.0f}%）" if payload['navigable']
                   else payload['error'] or '不可导航'),
        'thresholds': gate,
        'quality': payload['quality'],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description='LDC 上车前自检')
    parser.add_argument('--map', default=None, help='要验证的地图前缀（maps/xxx）')
    parser.add_argument('--skip-ros', action='store_true', help='只验地图质量，不碰 ROS')
    parser.add_argument('--min-rate', type=float, default=1.0, help='/scan 与 /odom 的最低频率')
    parser.add_argument('--require-nav', action='store_true',
                        help='要跑 C/D 验收：Nav2 未 active、清 costmap 服务缺失都按 FAIL 计')
    parser.add_argument('--tf-pair', action='append', default=None, metavar='PARENT:CHILD',
                        help='要检查的 TF 链，可多次（默认 odom:base_footprint）')
    parser.add_argument('--json', action='store_true', help='只输出 JSON')
    args = parser.parse_args(argv)

    checks = {}
    if not args.skip_ros:
        checks['ros'] = check_ros_environment()
        if checks['ros']['ok']:
            checks['scan'] = check_topic_rate('/scan', args.min_rate)
            checks['odom'] = check_topic_rate('/odom', args.min_rate)
        # TF 链永远必查：话题在、频率正常，但 TF 断链时导航同样起不来。
        pairs = [tuple(item.split(':', 1)) for item in (args.tf_pair or ['odom:base_footprint'])]
        for key, result in check_tf_chain(pairs).items():
            result['severity'] = 'fail'
            checks[f'tf {key}'] = result
        # Nav2 / clear 服务在"只做健康检查"时只告警；要跑 C/D 验收（--require-nav）时算失败。
        nav_severity = 'fail' if args.require_nav else 'warn'
        for node, result in check_nav2_state().items():
            result['severity'] = nav_severity
            checks[f'nav2 {node}'] = result
        cleared = check_clear_services()
        cleared['severity'] = nav_severity
        checks['clear services'] = cleared
    if args.map:
        prefix = Path(args.map)
        if not prefix.is_absolute():
            prefix = (REPO_ROOT / prefix)
        checks['map'] = check_map_quality(prefix)

    if not checks:
        print('没有任何检查项：请给 --map，或去掉 --skip-ros', file=sys.stderr)
        return 2
    failed = [name for name, item in checks.items()
              if not item['ok'] and item.get('severity', 'fail') == 'fail']
    warned = [name for name, item in checks.items()
              if not item['ok'] and item.get('severity') == 'warn']
    report = {'ok': not failed, 'failed': failed, 'warned': warned,
              'map': str(args.map) if args.map else None, 'checks': checks}

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    else:
        print(f"LDC 上车自检：{'通过' if report['ok'] else '未通过'}")
        for name, item in checks.items():
            if item['ok']:
                mark = 'OK  '
            else:
                mark = 'WARN' if item.get('severity') == 'warn' else 'FAIL'
            print(f'  [{mark}] {name}: {item["detail"]}')
        if warned:
            print(f"\n提示：{len(warned)} 项未就绪但当前不算失败；要跑 C/D 验收请加 --require-nav。")
        if report['ok']:
            print('\n下一步：启动导航 → 在 165629 图上做 0.3–0.5m 单点，'
                  '观察 manifest 里 localization_confirmed 与 requestedStamp/lastPoseStamp。')
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
