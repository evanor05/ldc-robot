#!/usr/bin/env python3
"""LDC bounded passive recording and isolated replay."""
import argparse
import json
import os
import sqlite3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    record = sub.add_parser('record')
    record.add_argument('--output', required=True)
    record.add_argument('--seconds', type=float, default=60)
    record.add_argument('--max-mb', type=int, default=128)
    record.add_argument('--camera-every', type=float, default=0)
    record.add_argument('--api-port', type=int, default=3000)
    group = record.add_mutually_exclusive_group()
    group.add_argument('--sim', action='store_true')
    group.add_argument('--real', action='store_true')
    for name in ('report', 'summarize', 'replay'):
        command = sub.add_parser(name)
        command.add_argument('folder')
        if name == 'replay':
            command.add_argument('--rate', type=float, default=1.)
    args = parser.parse_args()
    try:
        if args.command == 'record':
            if not 0<args.seconds<=600 or not 1<=args.max_mb<=512 or not 0<=args.camera_every<=60 or not 1<=args.api_port<=65535:
                raise ValueError('时长 0–600s，容量 1–512MiB，拍照间隔 0–60s，端口 1–65535')
            if args.sim:
                os.environ.update(ROS_DOMAIN_ID='197', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOCALHOST_ONLY='1', ROBOT_SIMULATION='1')
            elif args.real:
                os.environ.update(ROS_DOMAIN_ID=os.environ.get('ROBOT_REAL_DOMAIN_ID', '0'), ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET', ROBOT_SIMULATION='0')
                os.environ.pop('ROS_LOCALHOST_ONLY', None)
            from scripts.diagnostics.capture import capture
            result = capture(args.output, args.seconds, args.max_mb*1024*1024, args.camera_every, args.api_port)
        elif args.command == 'report':
            from scripts.diagnostics.storage import report
            result = report(args.folder)
        elif args.command == 'summarize':
            from scripts.diagnostics.replay import summarize_capture
            result = summarize_capture(args.folder)
        else:
            if not .1<=args.rate<=4:
                raise ValueError('回放倍率必须为 0.1–4')
            from scripts.diagnostics.replay import replay
            result = replay(args.folder, args.rate)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        if result.get('state') == 'failed':
            parser.exit(1, '采集失败：'+result.get('error', '请查看 capture.json')+'\n')
    except (ValueError, OSError, sqlite3.Error) as error:
        parser.exit(1, f'采集/回放未完成：{error}\n')


if __name__ == '__main__':
    main()
