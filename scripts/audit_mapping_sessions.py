#!/usr/bin/env python3
"""碎片化建图会话的只读对账。

现场问题：17:37–18:00 之间出现了 8 段建图会话，其中 7 段没有任何地图产物，
但事后从日志里很难一眼看出"哪段结束了、为什么结束、有没有产物"。
本脚本把这件事变成可重复的一条命令：

  python3 scripts/audit_mapping_sessions.py                     # 默认扫 logs/runs
  python3 scripts/audit_mapping_sessions.py --root logs/runs --root log
  python3 scripts/audit_mapping_sessions.py --json

纪律（对应决策约束）：
- **只读**：不写任何日志、不补造缺失的 manifest、不合并/重启任何会话。
- **不猜终态**：没有可识别结束事件的会话标 unknown，不按"没有产物"倒推成功或失败。
- **行数不是失败次数**：`No valid trajectories` 同一条消息会被多个 logger 各写一行
  （现场是 2 个），所以同时给出"行数""去重后的控制周期估计"，并显式声明不能当失败计数。
- 重复的日志文件（软链/同一 inode/内容相同）只算一次，避免把故障放大。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOTS = ('logs/runs', 'log')

# 结束类事件（任何一条都说明该会话有过明确的收尾/中断）
TERMINAL_PATTERNS = (
    ('saved', re.compile(r'map_saved|地图已保存|savedMap.*name', re.I)),
    ('aborted', re.compile(r'mapping_aborted|建图已中止|abort_mapping', re.I)),
    ('stopped_no_save', re.compile(r'saveMap.*failed|保存失败|保存地图超时', re.I)),
    ('failed', re.compile(r'建图.*失败|mapping.*fail|进程退出|Traceback', re.I)),
)
NO_VALID_TRAJECTORY = re.compile(r'No valid trajectories', re.I)
# 真实行形态（logs/runs/20261111-173615-real/console.log）：
#   [controller_server-2] [ERROR] [1794390218.093707904] [DWBLocalPlanner]: No valid trajectories out of 1229!
#   [controller_server-2] [WARN]  [1794390218.094773790] [controller_server]: Could not find ... No valid trajectories out of 1229!
# 同一控制周期被两个 logger 各写一行 → 周期数 = 行数 / logger 种类数。
TRAJECTORY_LINE = re.compile(
    r'\[(ERROR|WARN)\]\s*\[(\d+\.\d+)\]\s*\[([^\]]+)\][^\n]*?No valid trajectories out of (\d+)')
BASE_OBSTACLE_LINE = re.compile(r'BaseObstacle/Trajectory Hits Obstacle')
BASE_OBSTACLE_FULL = re.compile(r'1\.00:\s*BaseObstacle/Trajectory Hits Obstacle')
MAPPING_MARKERS = ('mapping_started', '建图', 'gmapping', 'start_mapping', 'start_mapping_task')
NAVIGATION_MARKERS = ('startup_begin', '导航启动', 'navigation_started', 'amcl', 'navigate_to_pose')
PRODUCT_SUFFIXES = ('.yaml', '.pgm')


def file_hash(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open('rb') as handle:
            for chunk in iter(lambda: handle.read(1024 * 256), b''):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def read_text(path: Path, limit=4 * 1024 * 1024) -> str:
    try:
        with path.open('r', encoding='utf-8', errors='replace') as handle:
            return handle.read(limit)
    except OSError:
        return ''


def collect_log_files(root: Path) -> list[Path]:
    if root.is_file():
        return [root]
    if not root.is_dir():
        return []
    files = []
    for item in sorted(root.rglob('*')):
        if item.is_file() and item.suffix in ('.log', '.jsonl', '.json', '.txt'):
            files.append(item)
    return files


def dedupe_by_content(files: list[Path]) -> tuple[list[Path], int]:
    """内容相同或指向同一 inode 的日志只保留一份（重复日志不得把故障放大）。"""
    seen_inodes, seen_hashes, kept, dropped = set(), {}, [], 0
    for path in files:
        try:
            stat = path.stat()
        except OSError:
            continue
        inode = (stat.st_dev, stat.st_ino)
        if inode in seen_inodes:
            dropped += 1
            continue
        seen_inodes.add(inode)
        digest = file_hash(path)
        if digest and digest in seen_hashes:
            dropped += 1
            continue
        if digest:
            seen_hashes[digest] = path
        kept.append(path)
    return kept, dropped


def load_manifest(session_dir: Path) -> dict | None:
    path = session_dir / 'manifest.json'
    if not path.is_file():
        return None            # 旧会话没有 manifest 就不补造
    try:
        return json.loads(path.read_text('utf-8'))
    except (OSError, ValueError):
        return {'error': 'manifest.json 损坏'}


def map_products(session_dir: Path, text: str) -> list[str]:
    """产物判定：目录里真实存在的 yaml+pgm 配对，或日志里明确记录了保存成功的地图名。"""
    products = set()
    try:
        for item in session_dir.iterdir():
            if item.suffix == '.yaml' and item.with_suffix('.pgm').is_file():
                products.add(item.stem)
    except OSError:
        pass
    for match in re.finditer(r'map_saved[^\n]*?name["\']?\s*[:=]\s*["\']?([A-Za-z0-9_\-]+)', text):
        products.add(match.group(1))
    return sorted(products)


def trajectory_metrics(text: str) -> dict:
    """把 No valid trajectories 行折算成"控制周期"量级，并交叉验证 BaseObstacle。

    现场实测（logs/runs/20261111-173615-real/console.log）：
      1612 行 / 2 个 logger（DWBLocalPlanner + controller_server）= 806 个控制周期，
      而 BaseObstacle/Trajectory Hits Obstacle 恰好 806 行，其中 799 行是 1.00（100%）。
    所以"1612"绝不能当 1612 次失败；周期数与 BaseObstacle 命中数互相印证。
    """
    hits = [(float(match.group(2)), match.group(3), int(match.group(4)))
            for match in TRAJECTORY_LINE.finditer(text)]
    plain_lines = len(NO_VALID_TRAJECTORY.findall(text))
    sources = sorted({hit[1] for hit in hits})
    cycles = round(len(hits) / len(sources)) if sources else plain_lines
    obstacle_hits = len(BASE_OBSTACLE_LINE.findall(text))
    obstacle_full = len(BASE_OBSTACLE_FULL.findall(text))
    consistent = bool(cycles) and obstacle_hits == cycles
    return {
        'lines': max(plain_lines, len(hits)),
        'loggerSources': sources,
        'loggerCopies': len(sources) or 1,
        'estimatedControlCycles': cycles,
        'trajectoryCounts': sorted({hit[2] for hit in hits}),
        'baseObstacleHits': obstacle_hits,
        'baseObstacleHundredPercent': obstacle_full,
        'cycleCountCrossChecked': consistent,
        'note': '行数不是失败次数：同一控制周期被每个 logger 各写一行；'
                'estimatedControlCycles = 行数 / logger 种类数，'
                '并与 BaseObstacle 命中行数交叉验证（一致时可信）。禁止把 lines 当独立失败计数。',
    }


def audit_session(session_dir: Path) -> dict:
    files, dropped = dedupe_by_content(collect_log_files(session_dir))
    combined = '\n'.join(read_text(path) for path in files)

    terminals = [(name, pattern.search(combined)) for name, pattern in TERMINAL_PATTERNS]
    hits = [name for name, match in terminals if match]
    # 多类结束事件同时出现时按"最明确"的优先：saved > aborted > stopped_no_save > failed
    terminal = next((name for name in ('saved', 'aborted', 'stopped_no_save', 'failed')
                     if name in hits), 'unknown')
    if terminal == 'unknown' and not combined.strip():
        terminal = 'unknown'

    started = any(marker in combined for marker in (*MAPPING_MARKERS, *NAVIGATION_MARKERS))
    products = map_products(session_dir, combined)
    manifest = load_manifest(session_dir)
    mtimes = []
    for path in files:
        try:
            mtimes.append(path.stat().st_mtime)
        except OSError:
            pass

    lowered = combined.lower()
    mapping_hits = [marker for marker in MAPPING_MARKERS if marker.lower() in lowered]
    navigation_hits = [marker for marker in NAVIGATION_MARKERS if marker.lower() in lowered]
    if mapping_hits and not navigation_hits:
        kind = 'mapping'
    elif navigation_hits and not mapping_hits:
        kind = 'navigation'
    elif mapping_hits and navigation_hits:
        kind = 'mixed'
    else:
        kind = 'unknown'

    return {
        'session': session_dir.name,
        'path': str(session_dir),
        'kind': kind,
        'files': len(files),
        'duplicateLogsDropped': dropped,
        'hasManifest': manifest is not None,
        'manifestError': (manifest or {}).get('error') if isinstance(manifest, dict) else None,
        'startedMarker': started,
        'terminal': terminal,
        'terminalEvidence': hits,
        'mapProducts': products,
        'hasProduct': bool(products),
        'spanSeconds': round(max(mtimes) - min(mtimes), 1) if len(mtimes) >= 2 else None,
        'noValidTrajectory': trajectory_metrics(combined),
    }


def audit(roots: list[Path]) -> dict:
    sessions = []
    for root in roots:
        if not root.exists():
            continue
        if root.is_file():
            sessions.append(audit_session(root.parent))
            continue
        for item in sorted(root.iterdir()):
            if item.is_dir():
                sessions.append(audit_session(item))
    # 输入顺序无关：按会话名排序，保证同样输入得到同样输出
    unique = {}
    for session in sessions:
        unique[(session['path'], session['session'])] = session
    # 先按会话名、再按路径：与 --root 的传入顺序无关，保证同样输入得到同样输出
    ordered = [unique[key] for key in sorted(unique, key=lambda key: (key[1], key[0]))]
    with_product = [item for item in ordered if item['hasProduct']]
    return {
        'sessions': ordered,
        'summary': {
            'total': len(ordered),
            'withProduct': len(with_product),
            'withoutProduct': len(ordered) - len(with_product),
            'unknownTerminal': sum(1 for item in ordered if item['terminal'] == 'unknown'),
            'duplicateLogsDropped': sum(item['duplicateLogsDropped'] for item in ordered),
            'byKind': {kind: sum(1 for item in ordered if item['kind'] == kind)
                       for kind in ('mapping', 'navigation', 'mixed', 'unknown')},
        },
    }


def render(report: dict) -> str:
    lines = ['会话\t类型\t终态\t产物\t时长s\t重复日志\tNo valid trajectories(行/周期)']
    for item in report['sessions']:
        lines.append('\t'.join([
            item['session'], item['kind'], item['terminal'],
            ','.join(item['mapProducts']) or '-',
            '-' if item['spanSeconds'] is None else f"{item['spanSeconds']:.0f}",
            str(item['duplicateLogsDropped']),
            f"{item['noValidTrajectory']['lines']}/{item['noValidTrajectory']['estimatedControlCycles']}",
        ]))
        metrics = item['noValidTrajectory']
        if metrics['lines']:
            lines.append(f"    → {metrics['lines']} 行来自 {metrics['loggerCopies']} 个 logger"
                         f"（{','.join(metrics['loggerSources']) or '-'}）；"
                         f"周期估计 {metrics['estimatedControlCycles']}，"
                         f"BaseObstacle 命中 {metrics['baseObstacleHits']}"
                         f"（其中 100% {metrics['baseObstacleHundredPercent']}）；"
                         f"交叉验证{'一致' if metrics['cycleCountCrossChecked'] else '不一致，需人工核对'}")
    summary = report['summary']
    mapping_sessions = [item for item in report['sessions'] if item['kind'] == 'mapping']
    lines.append('')
    if mapping_sessions:
        lines.append(f"建图会话 {len(mapping_sessions)} 段：有产物 "
                     f"{sum(1 for item in mapping_sessions if item['hasProduct'])} 段；"
                     f"无产物 {sum(1 for item in mapping_sessions if not item['hasProduct'])} 段")
    lines.append('')
    lines.append(f"会话 {summary['total']} 段；有产物 {summary['withProduct']} 段；"
                 f"无产物 {summary['withoutProduct']} 段；终态未知 {summary['unknownTerminal']} 段")
    if summary['unknownTerminal']:
        lines.append('终态未知的会话不做推断：需要人工看对应日志确认，脚本不会补造 manifest 或合并会话。')
    return '\n'.join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='碎片化建图会话只读对账')
    parser.add_argument('--root', action='append', default=None, help='会话目录根（可多次）')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)

    roots = [Path(item) for item in (args.root or [str(REPO_ROOT / r) for r in DEFAULT_ROOTS])]
    roots = [item if item.is_absolute() else (REPO_ROOT / item) for item in roots]
    report = audit(roots)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    else:
        print(render(report))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
