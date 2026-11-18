#!/usr/bin/env python3
"""只读核验一次运行的证据是否还站得住。

回答一个具体问题：manifest 里记录的源码指纹，现在还在不在、还是不是同一份？
现场复盘最常见的事故是"事后改了代码再回看日志"，导致证据无法证明当时跑的是什么。

纪律：
- **只读**：不修改归档、不补造字段、不重写旧 manifest、不把"当时的期望"改写成"现在的实际"。
- 不判断"运行是否成功"，只判断"证据是否自洽"。
- 旧归档缺少后来新增的监控文件时，只报 coverage 警告，不算漂移（不能拿新标准判旧证据）。

退出码：
  0 全部一致且证据完整
  1 有漂移（文件被改/被删）
  2 找不到 manifest
  3 manifest 损坏
  4 证据不完整（缺 summary / 失败运行缺现场快照）
  5 用法错误（没给 runId 且没有任何历史 run）
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_manifest as rm  # noqa: E402

EXIT_OK = 0
EXIT_DRIFT = 1
EXIT_NO_MANIFEST = 2
EXIT_CORRUPT = 3
EXIT_INCOMPLETE = 4
EXIT_USAGE = 5


def _runs_dir(value=None) -> Path:
    return Path(value or os.environ.get('ROBOT_RUNS_DIR') or rm.DEFAULT_RUNS_DIR)


def verify(run_id: str, runs_dir=None) -> tuple[int, dict]:
    runs = _runs_dir(runs_dir)
    run_dir = runs / run_id
    manifest_path = run_dir / rm.MANIFEST_NAME
    report = {'runId': run_id, 'runDir': str(run_dir), 'drift': [], 'files': [],
              'warnings': [], 'incomplete': [], 'coverage': {}}
    if not manifest_path.is_file():
        report['error'] = f'找不到 {manifest_path}'
        return EXIT_NO_MANIFEST, report
    try:
        manifest = json.loads(manifest_path.read_text('utf-8'))
    except (OSError, ValueError) as error:
        report['error'] = f'manifest 损坏：{error}'
        return EXIT_CORRUPT, report

    for entry in manifest.get('sources') or []:
        relative = entry.get('path')
        if not isinstance(relative, str) or not relative:
            report['warnings'].append(f'忽略无路径条目：{entry}')
            continue
        recorded = entry.get('sha256')
        target = rm.REPO_ROOT / relative
        current = rm.sha256_file(target) if target.is_file() else None
        if recorded is None:
            state = 'unverified'      # 运行当时就不存在，不是漂移
        elif current is None:
            state = 'missing'
        elif current != recorded:
            state = 'changed'
        else:
            state = 'identical'
        item = {'path': relative, 'state': state, 'recorded': recorded, 'current': current}
        report['files'].append(item)
        if state in ('missing', 'changed'):
            report['drift'].append(item)

    recorded_paths = {item['path'] for item in report['files']}
    not_covered = [path for path in rm.WATCHED_SOURCES if path not in recorded_paths]
    report['coverage'] = {'recorded': len(recorded_paths),
                          'expected': len(rm.WATCHED_SOURCES),
                          'missingFromManifest': not_covered}
    if not_covered:
        report['warnings'].append(f'本次 manifest 未记录 {len(not_covered)} 个当前受监控文件'
                                  f'（旧归档用旧口径，不算漂移）')

    summary_path = run_dir / rm.SUMMARY_JSON
    if not summary_path.is_file():
        report['incomplete'].append('缺少 summary.json（运行未收尾或未正常结束）')
    else:
        try:
            summary = json.loads(summary_path.read_text('utf-8'))
        except (OSError, ValueError) as error:
            report['incomplete'].append(f'summary.json 损坏：{error}')
            summary = {}
        absent = [item for item in (summary.get('snapshots') or []) if not Path(item).is_file()]
        if absent:
            report['incomplete'].append(f'缺少 {len(absent)} 个已登记的现场快照')
        status = summary.get('status')
        if status and status != 'success' and not summary.get('snapshots'):
            report['incomplete'].append(f'运行结论为 {status}，但没有保存任何现场快照')
        report['status'] = status

    if report['drift']:
        return EXIT_DRIFT, report
    if report['incomplete']:
        return EXIT_INCOMPLETE, report
    return EXIT_OK, report


def render(report: dict) -> str:
    lines = [f"runId: {report['runId']}", f"目录: {report['runDir']}"]
    if report.get('error'):
        return '\n'.join(lines + [f"错误: {report['error']}"])
    total = len(report['files'])
    identical = sum(1 for item in report['files'] if item['state'] == 'identical')
    lines.append(f"源码指纹: {identical}/{total} 一致")
    for item in report['drift']:
        if item['state'] == 'changed':
            lines.append(f"  [CHANGED] {item['path']}")
            lines.append(f"            记录 {item['recorded'][:16]}… ≠ 当前 {item['current'][:16]}…")
        else:
            lines.append(f"  [MISSING] {item['path']}（运行时有记录，现在找不到文件）")
    coverage = report['coverage']
    if coverage.get('missingFromManifest'):
        lines.append(f"覆盖警告: 本 manifest 只记录 {coverage['recorded']}/{coverage['expected']} 个受监控文件")
        for path in coverage['missingFromManifest'][:8]:
            lines.append(f"  - {path}")
        if len(coverage['missingFromManifest']) > 8:
            lines.append(f"  … 另有 {len(coverage['missingFromManifest']) - 8} 个")
    for item in report['incomplete']:
        lines.append(f"证据不完整: {item}")
    verdict = ('一致' if not report['drift'] and not report['incomplete'] else
               '有漂移' if report['drift'] else '证据不完整')
    lines.append(f"结论: {verdict}")
    return '\n'.join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='只读核验一次运行的证据是否自洽')
    parser.add_argument('run_id', nargs='?', default=None, help='runId；缺省取最近一次')
    parser.add_argument('--runs-dir', default=None)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)

    run_id = args.run_id or rm.latest_run(runs_dir=_runs_dir(args.runs_dir))
    if not run_id:
        print('没有找到任何 run：请先 python3 scripts/run_manifest.py start ...', file=sys.stderr)
        return EXIT_USAGE
    code, report = verify(run_id, args.runs_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2, default=str) if args.json else render(report))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
