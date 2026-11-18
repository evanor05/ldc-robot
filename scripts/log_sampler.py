#!/usr/bin/env python3
"""运行期采样器：把网络、无线链路和后端状态按固定间隔写入 JSONL。

独立于 ROS，不依赖 rclpy；由 scripts/run_real_logged.sh 启动。
目的：事后能回答"那一刻无线链路/后端/车端是什么状态"。
"""
import argparse
import json
import os
import re
import subprocess
import time
import urllib.error
import urllib.request


def sh(command, timeout=2):
    try:
        return subprocess.run(command, shell=True, capture_output=True, text=True,
                              timeout=timeout).stdout.strip()
    except (subprocess.SubprocessError, OSError):
        return ''


def wifi_line(iface=''):
    """解析 `iw dev <if> link`，返回信号与速率。"""
    text = sh(f'iw dev {iface} link' if iface else 'iw dev 2>/dev/null | head -20')
    if not text:
        return {'connected': False}
    info = {'connected': True}
    for pattern, key in ((r'SSID:\s*(.+)', 'ssid'), (r'signal:\s*(-?\d+)', 'signalDbm'),
                         (r'rx bitrate:\s*([\d.]+)', 'rxMbit'), (r'tx bitrate:\s*([\d.]+)', 'txMbit'),
                         (r'freq:\s*([\d.]+)', 'freqMhz')):
        match = re.search(pattern, text)
        if match:
            value = match.group(1).strip()
            info[key] = float(value) if key in ('signalDbm', 'rxMbit', 'txMbit', 'freqMhz') else value
    return info


def power_save(iface=''):
    if not iface:
        return None
    text = sh(f'iw dev {iface} get power_save')
    return 'on' in text


def interface():
    text = sh('iw dev | awk \'$1=="Interface"{print $2; exit}\'')
    return text.strip()


def counters(iface):
    """从 sysfs 读取网卡累计计数，比解析 ip -s 稳定。"""
    keys = {'rxPackets': 'rx_packets', 'rxErrors': 'rx_errors', 'rxDropped': 'rx_dropped',
            'txPackets': 'tx_packets', 'txErrors': 'tx_errors', 'txDropped': 'tx_dropped'}
    result = {}
    for name, filename in keys.items():
        try:
            with open(f'/sys/class/net/{iface}/statistics/{filename}', encoding='utf-8') as handle:
                result[name] = int(handle.read().strip())
        except (OSError, ValueError):
            pass
    return result


def ping(ip):
    if not ip:
        return {}
    started = time.monotonic()
    text = sh(f'ping -c 1 -W 1 {ip}', timeout=3)
    match = re.search(r'time=([\d.]+)', text)
    return {'ip': ip, 'ok': bool(match),
            'rttMs': float(match.group(1)) if match else None,
            'elapsedMs': round((time.monotonic() - started) * 1000)}


def car_ip_from_agent_log(log_dir):
    """从 micro-ROS agent 日志里取车端 IP：日志里会记录 session 建立时的 address。"""
    path = os.path.join(log_dir, 'console.log')
    try:
        with open(path, 'r', encoding='utf-8', errors='ignore') as handle:
            handle.seek(max(0, os.path.getsize(path) - 200000))
            matches = re.findall(r'address:\s*(\d+\.\d+\.\d+\.\d+):', handle.read())
            if matches:
                return matches[-1]
    except OSError:
        pass
    return ''


def backend_status(api):
    try:
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(
                api + '/navigation_status', timeout=2) as reply:
            data = json.load(reply)
    except (urllib.error.URLError, OSError, ValueError):
        return {'reachable': False}
    checks = data.get('checks') or {}
    compact = {name: {k: value.get(k) for k in ('ok', 'ageSeconds', 'sampleAgeSeconds', 'detail')}
               for name, value in checks.items() if isinstance(value, dict)}
    return {'reachable': True, 'mode': data.get('mode'), 'lockKind': data.get('lockKind'),
            'ready': data.get('ready'), 'error': data.get('error'),
            'controlOwner': bool(data.get('controlOwner')),
            'leaseAgeSeconds': data.get('leaseAgeSeconds'), 'checks': compact,
            'lastStop': data.get('lastStop')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dir', required=True, help='运行日志目录')
    parser.add_argument('--api', default='http://127.0.0.1:3000/api')
    parser.add_argument('--interval', type=float, default=5.0)
    parser.add_argument('--car-ip', default=os.environ.get('ROBOT_CAR_IP', ''))
    args = parser.parse_args()

    os.makedirs(args.dir, exist_ok=True)
    path = os.path.join(args.dir, 'net.jsonl')
    last = {'rxPackets': None, 'txPackets': None}
    with open(path, 'a', buffering=1, encoding='utf-8') as handle:
        while True:
            started = time.time()
            iface = interface()
            ip = args.car_ip or car_ip_from_agent_log(args.dir)
            current = counters(iface) if iface else {}
            delta = {}
            for key in ('rxPackets', 'txPackets', 'rxDropped', 'txDropped', 'rxErrors', 'txErrors'):
                if key in current:
                    delta['d' + key[0].upper() + key[1:]] = (
                        current[key] - last[key] if last.get(key) is not None else 0)
            last.update({k: current.get(k) for k in last})
            handle.write(json.dumps({
                'at': started, 'interface': iface, 'wifi': wifi_line(iface),
                'powerSave': power_save(iface),
                'link': current, 'delta': delta, 'ping': ping(ip),
                'backend': backend_status(args.api)}, ensure_ascii=False) + '\n')
            time.sleep(max(0.5, args.interval - (time.time() - started)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
