#!/usr/bin/env python3
"""离线分析一次运行留下的日志，生成 summary.md / summary.json。

用法:
    python3 scripts/diagnose_run.py logs/runs/20261111-161800-real
    python3 scripts/diagnose_run.py --latest        # 分析最近一次运行
"""
import argparse
import collections
import glob
import json
import os
import re
import statistics
import sys


def read_jsonl(path):
    rows = []
    try:
        with open(path, encoding='utf-8', errors='ignore') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    continue
    except OSError:
        pass
    return rows


def read_text(path):
    try:
        with open(path, encoding='utf-8', errors='ignore') as handle:
            return handle.read()
    except OSError:
        return ''


def fmt(seconds):
    if seconds is None:
        return '—'
    seconds = float(seconds)
    if seconds < 0:
        return '—'
    if seconds < 90:
        return f'{seconds:.1f}s'
    minutes, rest = divmod(seconds, 60)
    if minutes < 90:
        return f'{int(minutes)}m{int(rest):02d}s'
    hours, minutes = divmod(int(minutes), 60)
    return f'{hours}h{minutes:02d}m'


def analyse(run_dir):
    events = read_jsonl(os.path.join(run_dir, 'bridge-events.jsonl'))
    samples = read_jsonl(os.path.join(run_dir, 'bridge-samples.jsonl'))
    net = read_jsonl(os.path.join(run_dir, 'net.jsonl'))
    client = read_jsonl(os.path.join(run_dir, 'frontend-events.jsonl'))
    console = read_text(os.path.join(run_dir, 'console.log'))

    ticks = [row for row in samples if row.get('kind') == 'tick']
    stamps = [row['at'] for row in ticks if row.get('at')] or \
             [row['at'] for row in net if row.get('at')] or \
             [row['at'] for row in events if row.get('at')]
    started = min(stamps) if stamps else None
    ended = max(stamps) if stamps else None
    duration = (ended - started) if started and ended else None

    def count(kind):
        return sum(1 for row in events if row.get('type') == kind)

    locks = [row for row in events if row.get('type') in ('estop', 'cancel_confirmed')]
    hardware = [row for row in events if row.get('type') == 'estop']
    protective = [row for row in locks if (row.get('kind') or '') == 'protective']
    session_idle = {'estop', 'cancel_confirmed', 'cancel_requested', 'navigation_stop'}

    # 传感器质量
    def quality(key):
        values = [row[key] for row in ticks if isinstance(row.get(key), dict)]
        ok = sum(1 for item in values if item.get('ok'))
        gaps = [item.get('maxGap5s') for item in values
                if isinstance(item.get('maxGap5s'), (int, float))]
        ages = [item.get('ageSeconds') for item in values
                if isinstance(item.get('ageSeconds'), (int, float))]
        return {
            'samples': len(values),
            'okRatio': round(ok / len(values), 3) if values else None,
            'worstGapSeconds': round(max(gaps), 2) if gaps else None,
            'gapsOver1s': sum(1 for gap in gaps if gap > 1.0),
            'gapsOver2s': sum(1 for gap in gaps if gap > 2.0),
            'worstAgeSeconds': round(max(ages), 2) if ages else None,
            'medianAgeSeconds': round(statistics.median(ages), 2) if ages else None,
        }

    # micro-ROS 会话：从 console.log 里取 establish/close 时间
    stamps = []
    for match in re.finditer(r'\[(\d+\.\d+)\].*?(session established|session closed)', console):
        stamps.append((float(match.group(1)), match.group(2)))
    rebuilds = []
    pending_close = None
    for stamp, kind in stamps:
        if kind == 'session closed':
            pending_close = stamp
        elif pending_close is not None:
            rebuilds.append(stamp - pending_close)
            pending_close = None

    # 网络
    pings = [row['ping'] for row in net if isinstance(row.get('ping'), dict) and row['ping'].get('ip')]
    rtts = [item['rttMs'] for item in pings if isinstance(item.get('rttMs'), (int, float))]
    signals = [row['wifi'].get('signalDbm') for row in net
               if isinstance(row.get('wifi'), dict) and isinstance(row['wifi'].get('signalDbm'), (int, float))]
    backend_down = sum(1 for row in net
                       if isinstance(row.get('backend'), dict) and not row['backend'].get('reachable'))

    summary = {
        'runDir': os.path.abspath(run_dir),
        'startedAt': started,
        'durationSeconds': round(duration, 1) if duration else None,
        'events': len(events),
        'locks': len(locks),
        'hardwareEstops': len(hardware),
        'protectiveStops': len(protective),
        'sensorDropouts': count('sensor_dropout'),
        'sensorRecoveries': count('sensor_recovered'),
        'leaseExpired': count('lease_expired'),
        'navFaults': count('navigation_fault'),
        'microRosRebuilds': len(rebuilds),
        'microRosWorstRebuildSeconds': round(max(rebuilds), 2) if rebuilds else None,
        'lidar': quality('lidar'),
        'odometry': quality('odometry'),
        'network': {
            'samples': len(net),
            'pingFailures': sum(1 for item in pings if not item.get('ok')),
            'pingWorstMs': round(max(rtts), 1) if rtts else None,
            'pingMedianMs': round(statistics.median(rtts), 1) if rtts else None,
            'signalMinDbm': min(signals) if signals else None,
            'signalMaxDbm': max(signals) if signals else None,
            'backendUnreachableSamples': backend_down,
            'carIp': pings[0]['ip'] if pings else None,
        },
        'frontendEvents': len(client),
        'frontendErrors': sum(1 for row in client if row.get('type') in ('window-error', 'unhandled-rejection')),
    }
    return summary, {'events': events, 'ticks': ticks, 'net': net, 'client': client,
                     'started': started}


def render(summary, raw):
    started = raw['started'] or 0
    def rel(stamp):
        return '—' if not stamp else f'+{fmt(stamp - started)}'
    lines = ['# 运行分析报告', '', f"日志目录：`{summary['runDir']}`", '']
    lines += ['## 概览', '',
              '| 指标 | 值 |', '| --- | --- |',
              f"| 时长 | {fmt(summary['durationSeconds'])} |",
              f"| 事件数 | {summary['events']} |",
              f"| 锁定次数 | {summary['locks']} |",
              f"| 其中硬件急停 | {summary['hardwareEstops']} |",
              f"| 其中保护停车 | {summary['protectiveStops']} |",
              f"| 数据断流 | {summary['sensorDropouts']}（恢复 {summary['sensorRecoveries']}） |",
              f"| 租约过期 | {summary['leaseExpired']} |",
              f"| 导航失效 | {summary['navFaults']} |",
              f"| micro-ROS 会话重建 | {summary['microRosRebuilds']} 次（最长 {fmt(summary['microRosWorstRebuildSeconds'])}） |",
              '']

    lines += ['## 传感器质量', '', '| 传感器 | 有效率 | 最大到达间隔 | >1s 次数 | >2s 次数 | 最差样本年龄 |',
              '| --- | --- | --- | --- | --- | --- |']
    for name, label in (('lidar', '雷达 /scan'), ('odometry', '里程计 /odom')):
        item = summary[name]
        lines.append(f"| {label} | {item['okRatio'] if item['okRatio'] is not None else '—'} | "
                     f"{fmt(item['worstGapSeconds'])} | {item['gapsOver1s']} | {item['gapsOver2s']} | "
                     f"{fmt(item['worstAgeSeconds'])} |")
    lines.append('')

    network = summary['network']
    lines += ['## 网络', '',
              f"| 车端 IP | {network['carIp'] or '未识别'} |", '| --- | --- |',
              f"| 采样次数 | {network['samples']} |",
              f"| ping 失败 | {network['pingFailures']} |",
              f"| RTT 中位/最差 | {network['pingMedianMs']} / {network['pingWorstMs']} ms |",
              f"| 信号范围 | {network['signalMinDbm']} ~ {network['signalMaxDbm']} dBm |",
              f"| 后端不可达采样 | {network['backendUnreachableSamples']} |",
              '']

    lines += ['## 关键事件时间线', '', '| 相对时间 | 类型 | 说明 |', '| --- | --- | --- |']
    for event in raw['events'][-60:]:
        lines.append(f"| {rel(event.get('at'))} | {event.get('type')} | "
                     f"{event.get('reason') or ''} |")
    lines.append('')

    # 锁定归因：锁定前后 10 秒的传感器状态
    locks = [event for event in raw['events']
             if event.get('type') in ('estop', 'cancel_confirmed')]
    if locks:
        lines += ['## 锁定归因', '']
        for lock in locks[-15:]:
            stamp = lock.get('at')
            near = [tick for tick in raw['ticks'] if abs(tick.get('at', 0) - stamp) <= 10]
            lidar = [tick['lidar'].get('ageSeconds') for tick in near
                     if isinstance(tick.get('lidar'), dict) and isinstance(tick['lidar'].get('ageSeconds'), (int, float))]
            odom = [tick['odometry'].get('ageSeconds') for tick in near
                    if isinstance(tick.get('odometry'), dict) and isinstance(tick['odometry'].get('ageSeconds'), (int, float))]
            lines.append(f"- {rel(stamp)} **{lock.get('type')}**（{lock.get('kind', '—')}）："
                         f"{lock.get('reason') or ''}；雷达年龄最大 "
                         f"{fmt(max(lidar)) if lidar else '—'}，里程计年龄最大 {fmt(max(odom)) if odom else '—'}")
        lines.append('')

    # 建议
    advice = []
    lidar = summary['lidar']
    if lidar['gapsOver2s']:
        advice.append(f"雷达出现 {lidar['gapsOver2s']} 次超过 2 秒的到达空洞，优先检查雷达串口/供电与固件 lidar 任务调度。")
    if summary['microRosRebuilds'] > 2:
        advice.append(f"micro-ROS 会话重建 {summary['microRosRebuilds']} 次，说明车端 XRCE 传输反复中断；"
                      "建议抓 ESP32 串口日志确认 'micro-ROS transport stopped' 的触发原因。")
    if summary['leaseExpired']:
        advice.append(f"租约过期 {summary['leaseExpired']} 次：检查浏览器是否在后台被节流，或页面长时间失焦。")
    if network['pingFailures']:
        advice.append(f"ping 车端失败 {network['pingFailures']} 次，检查 2.4G 干扰与主机省电设置。")
    if summary['hardwareEstops'] > 3:
        advice.append(f"硬件急停 {summary['hardwareEstops']} 次；若非手动操作，检查是否仍有自动触发路径。")
    if not advice:
        advice.append('本次运行未发现明显异常。')
    lines += ['## 建议', ''] + [f'- {item}' for item in advice] + ['']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', nargs='?')
    parser.add_argument('--latest', action='store_true', help='分析最近一次运行')
    parser.add_argument('--root', default=None, help='日志根目录（默认仓库 logs/runs）')
    parser.add_argument('--json', action='store_true', help='只输出 JSON 摘要')
    args = parser.parse_args()

    root = args.root or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs', 'runs')
    run_dir = args.run_dir
    if args.latest or not run_dir:
        candidates = sorted(glob.glob(os.path.join(root, '*')))
        if not candidates:
            sys.exit(f'没有找到运行日志：{root}')
        run_dir = candidates[-1]
    if not os.path.isdir(run_dir):
        sys.exit(f'目录不存在：{run_dir}')

    summary, raw = analyse(run_dir)
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return
    report = render(summary, raw)
    with open(os.path.join(run_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(report)
    with open(os.path.join(run_dir, 'summary.json'), 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    print(report)


if __name__ == '__main__':
    main()
