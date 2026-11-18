#!/usr/bin/env python3
"""Run manifest：为每次实车/离线运行留下可复核的证据。

问题背景：仓库里 web_console/、scripts/、robot_ws/ 长期是 untracked，日志只能写
git HEAD=a494e10，根本无法证明"当时跑的是哪份代码、哪套参数"。本模块为每次运行
建立独立 run 目录，落盘：

  <run_dir>/manifest.json   启动即为真：runId、时间、kind、git 基线（HEAD/branch/dirty
                            paths）、参与运行的源码 SHA256、生效参数、地图与质量门槛
  <run_dir>/summary.json    结束时的结论（status/reason/关键计数/耗时）
  <run_dir>/summary.md      同一结论的人读版本
  <run_dir>/checks/*.json   失败现场快照（checks、localization 等）

写入一律 best-effort：调用方（导航桥/建图）即使落盘失败也不能因此中断控制流程，
所以 main() 的错误只打印并以非零码退出，接口函数本身抛异常由调用方 try/except。

用法：
  python3 scripts/run_manifest.py start --kind navigation --param map=/path/map.yaml
  python3 scripts/run_manifest.py finish <runId> --status failed --reason '定位未确认'
  python3 scripts/run_manifest.py show <runId>
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNS_DIR = REPO_ROOT / 'logs' / 'runs'
# 实际参与运行的代码与配置：manifest 必须能定位到它们，而不只是一个 git HEAD。
WATCHED_SOURCES = [
    # 后端路由与进程管理
    'web_console/server.js',
    'web_console/mapStorage.js',
    'web_console/navigationRoutes.js',
    'web_console/navigationBridge.js',
    'web_console/visionRoutes.js',
    'web_console/managedProcess.js',
    'web_console/ros_config.json',
    # 前端：运行期真正决定行为的面板与状态机（此前完全没进 manifest）
    'web_console/src/App.vue',
    'web_console/src/components/NavigationControl.vue',
    'web_console/src/components/MappingControl.vue',
    'web_console/src/components/CameraView.vue',
    'web_console/src/components/SlamMap.vue',
    'web_console/src/composables/useFoxglove.js',
    # 车端脚本与策略
    'scripts/navigation_bridge.py',
    'scripts/localization_health.py',
    'scripts/sensor_health.py',
    'scripts/navigation_profile.py',
    'scripts/nav_stuck_policy.py',
    'scripts/mapping_preflight.py',
    'scripts/run_manifest.py',
    'scripts/start_mapping_task.sh',
    'scripts/start_navigation.sh',
    # 导航参数与启动
    'robot_ws/src/ldc_navigation/launch/navigation2.launch.py',
    'robot_ws/src/ldc_navigation/param/ldc_nav2.yaml',
]
MANIFEST_NAME = 'manifest.json'
SUMMARY_JSON = 'summary.json'
SUMMARY_MD = 'summary.md'
ACTIVE_MARKER = '.run-active'


def _utc_now() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc).astimezone()


def _iso(moment: dt.datetime | None = None) -> str:
    return (moment or _utc_now()).isoformat(timespec='seconds')


def sha256_file(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open('rb') as handle:
            for chunk in iter(lambda: handle.read(1024 * 256), b''):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def _git(*args: str) -> str | None:
    try:
        result = subprocess.run(['git', *args], cwd=REPO_ROOT, capture_output=True,
                                text=True, timeout=15, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def git_baseline() -> dict:
    """记录真实基线：HEAD 与 dirty paths（untracked 也要列出来）。"""
    head = _git('rev-parse', 'HEAD')
    branch = _git('rev-parse', '--abbrev-ref', 'HEAD')
    porcelain = _git('status', '--porcelain') or ''
    dirty = [line.rstrip() for line in porcelain.splitlines() if line.strip()]
    return {
        'head': head,
        'branch': branch,
        'dirtyCount': len(dirty),
        'dirtyPaths': dirty[:400],
        'dirtyTruncated': max(0, len(dirty) - 400),
        'tags': (_git('tag', '--points-at', 'HEAD') or '').split(),
    }


def collect_sources(paths: list[str] | None = None) -> list[dict]:
    entries = []
    for relative in paths or WATCHED_SOURCES:
        target = (REPO_ROOT / relative).resolve()
        entry = {'path': relative, 'exists': target.is_file()}
        if entry['exists']:
            stat = target.stat()
            entry.update({'sha256': sha256_file(target), 'bytes': stat.st_size,
                          'mtime': _iso(dt.datetime.fromtimestamp(stat.st_mtime, dt.timezone.utc))})
        entries.append(entry)
    return entries


def read_map_thresholds() -> dict:
    """记录本次实际生效的地图质量门槛（与 web_console/mapStorage.js 的 MAP_QUALITY 对齐）。"""
    defaults = {'minOccupiedCells': 2500, 'maxUnknownRatio': 0.85,
                'expectedResolution': 0.05, 'source': 'MAP_QUALITY 默认'}
    try:
        config = json.loads((REPO_ROOT / 'web_console' / 'ros_config.json').read_text('utf-8'))
    except (OSError, json.JSONDecodeError):
        return defaults
    save_map = config.get('save_map') or {}
    resolved = dict(defaults)
    resolved.update({
        'minOccupiedCells': save_map.get('min_occupied_cells', defaults['minOccupiedCells']),
        'maxUnknownRatio': save_map.get('max_unknown_ratio', defaults['maxUnknownRatio']),
        'expectedResolution': save_map.get('expected_resolution', defaults['expectedResolution']),
        'source': 'web_console/ros_config.json save_map',
    })
    return resolved


def _active_marker() -> Path:
    """运行期标记文件；测试可用 ROBOT_RUN_ACTIVE_MARKER 重定向，避免污染仓库。"""
    override = os.environ.get('ROBOT_RUN_ACTIVE_MARKER')
    return Path(override) if override else REPO_ROOT / ACTIVE_MARKER


def new_run_id(kind: str) -> str:
    stamp = _utc_now().strftime('%Y%m%dT%H%M%S')
    return f'run_{stamp}_{kind}'


def start_run(kind: str, params: dict | None = None, extra: dict | None = None,
              run_id: str | None = None, runs_dir: Path | None = None) -> dict:
    """建立 run 目录并写 manifest.json；返回 manifest 字典。"""
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    run_id = run_id or new_run_id(kind)
    run_dir = runs / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        'runId': run_id,
        'kind': kind,
        'startedAt': _iso(),
        'startedAtEpoch': _utc_now().timestamp(),
        'timezone': str(_utc_now().tzinfo),
        'host': os.uname().nodename if hasattr(os, 'uname') else None,
        'repoRoot': str(REPO_ROOT),
        'git': git_baseline(),
        'sources': collect_sources(),
        # 生效参数（不是配置文件里的期望值），调用方必须显式传入。
        'effectiveParams': dict(params or {}),
        'mapThresholds': read_map_thresholds(),
        'extra': dict(extra or {}),
        'schemaVersion': 2,
    }
    (run_dir / MANIFEST_NAME).write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), 'utf-8')
    # 运行期标记：实车/台架运行期间禁止 HMR 改前端源码（防止运行中换代码）。
    try:
        _active_marker().write_text(run_id, 'utf-8')
    except OSError:
        pass
    return manifest


def load_manifest(run_id: str, runs_dir: Path | None = None) -> dict:
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    path = runs / run_id / MANIFEST_NAME
    return json.loads(path.read_text('utf-8'))


def update_manifest(run_id: str, patch: dict, runs_dir: Path | None = None) -> dict:
    """把增量信息合并进 manifest.json（例如启动后实读到的生效参数）。"""
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    path = runs / run_id / MANIFEST_NAME
    manifest = json.loads(path.read_text('utf-8'))
    for key, value in (patch or {}).items():
        if isinstance(value, dict) and isinstance(manifest.get(key), dict):
            merged = dict(manifest[key])
            merged.update(value)
            manifest[key] = merged
        else:
            manifest[key] = value
    manifest['updatedAt'] = _iso()
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, default=str), 'utf-8')
    return manifest


def save_snapshot(run_id: str, name: str, payload, runs_dir: Path | None = None) -> str:
    """失败现场快照：checks / localization 等，文件名即快照名。"""
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    checks_dir = runs / run_id / 'checks'
    checks_dir.mkdir(parents=True, exist_ok=True)
    safe = ''.join(ch if ch.isalnum() or ch in '-_.' else '_' for ch in name)[:80] or 'snapshot'
    target = checks_dir / f'{safe}.json'
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str), 'utf-8')
    return str(target)


def finish_run(run_id: str, status: str, reason: str = '', summary: dict | None = None,
               snapshots: dict | None = None, runs_dir: Path | None = None) -> dict:
    """写 summary.json/summary.md，并在失败时保存快照。"""
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    run_dir = runs / run_id
    manifest = load_manifest(run_id, runs)
    finished = _utc_now()
    started_epoch = manifest.get('startedAtEpoch')
    duration = round(finished.timestamp() - started_epoch, 1) if started_epoch else None
    saved_snapshots = []
    for name, payload in (snapshots or {}).items():
        saved_snapshots.append(save_snapshot(run_id, name, payload, runs))

    record = {
        'runId': run_id,
        'kind': manifest.get('kind'),
        'status': status,
        'reason': reason,
        'startedAt': manifest.get('startedAt'),
        'finishedAt': _iso(finished),
        'durationSeconds': duration,
        'gitHead': (manifest.get('git') or {}).get('head'),
        'dirtyCount': (manifest.get('git') or {}).get('dirtyCount'),
        'effectiveParams': manifest.get('effectiveParams'),
        'mapThresholds': manifest.get('mapThresholds'),
        'summary': dict(summary or {}),
        'snapshots': saved_snapshots,
    }
    (run_dir / SUMMARY_JSON).write_text(
        json.dumps(record, ensure_ascii=False, indent=2, default=str), 'utf-8')
    (run_dir / SUMMARY_MD).write_text(render_summary_markdown(manifest, record), 'utf-8')
    try:
        marker = _active_marker()
        if marker.exists() and marker.read_text('utf-8').strip() == run_id:
            marker.unlink()
    except OSError:
        pass
    return record


def render_summary_markdown(manifest: dict, record: dict) -> str:
    params = record.get('effectiveParams') or {}
    thresholds = record.get('mapThresholds') or {}
    summary = record.get('summary') or {}
    lines = [
        f"# 运行摘要 {record.get('runId')}",
        '',
        f"- 类型：{record.get('kind')}",
        f"- 结论：**{record.get('status')}**{'（' + record['reason'] + '）' if record.get('reason') else ''}",
        f"- 起始：{record.get('startedAt')}　结束：{record.get('finishedAt')}　耗时：{record.get('durationSeconds')}s",
        f"- git：{(manifest.get('git') or {}).get('head')} ({(manifest.get('git') or {}).get('branch')})"
        f"，dirty={(manifest.get('git') or {}).get('dirtyCount')}",
        '',
        '## 生效参数',
        '',
    ]
    lines += [f'- `{key}` = `{value}`' for key, value in params.items()] or ['- （未记录）']
    lines += ['', '## 地图质量门槛', '',
              f"- 占用下限 {thresholds.get('minOccupiedCells')}，未知上限 {thresholds.get('maxUnknownRatio')}"
              f"（来源：{thresholds.get('source')}）", '', '## 关键结论', '']
    lines += [f'- `{key}` = `{value}`' for key, value in summary.items()] or ['- （未记录）']
    if record.get('snapshots'):
        lines += ['', '## 现场快照', ''] + [f'- `{item}`' for item in record['snapshots']]
    dirty = (manifest.get('git') or {}).get('dirtyPaths') or []
    if dirty:
        lines += ['', '## 未提交/未跟踪路径（前 40）', '```']
        lines += dirty[:40]
        lines += ['```']
    sources = [item for item in manifest.get('sources') or [] if item.get('sha256')]
    if sources:
        lines += ['', '## 参与运行的源码指纹', '', '| 文件 | SHA256 |', '| --- | --- |']
        lines += [f"| `{item['path']}` | `{item['sha256'][:16]}…` |" for item in sources]
    return '\n'.join(lines) + '\n'


def latest_run(kind: str | None = None, runs_dir: Path | None = None) -> str | None:
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    if not runs.is_dir():
        return None
    candidates = sorted((path.name for path in runs.iterdir()
                         if path.is_dir() and (path / MANIFEST_NAME).is_file()), reverse=True)
    if kind:
        candidates = [name for name in candidates if name.endswith(kind)]
    return candidates[0] if candidates else None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='运行证据清单（manifest/summary）')
    sub = parser.add_subparsers(dest='action', required=True)

    start = sub.add_parser('start', help='创建 run 并写 manifest.json')
    start.add_argument('--kind', required=True, help='navigation / mapping / bench 等')
    start.add_argument('--param', action='append', default=[], metavar='KEY=VALUE', help='生效参数')
    start.add_argument('--run-id', default=None)
    start.add_argument('--runs-dir', default=None)

    finish = sub.add_parser('finish', help='写 summary.json/summary.md')
    finish.add_argument('run_id')
    finish.add_argument('--status', required=True, choices=['success', 'failed', 'aborted', 'timeout'])
    finish.add_argument('--reason', default='')
    finish.add_argument('--summary', default=None, help='summary JSON 文件路径')
    finish.add_argument('--snapshot', action='append', default=[], metavar='NAME=/path/to.json')
    finish.add_argument('--runs-dir', default=None)

    show = sub.add_parser('show', help='打印 run 的 manifest/summary')
    show.add_argument('run_id', nargs='?', default=None)
    show.add_argument('--runs-dir', default=None)

    args = parser.parse_args(argv)
    runs_dir = Path(args.runs_dir) if args.runs_dir else None

    if args.action == 'start':
        params = {}
        for item in args.param:
            key, _, value = item.partition('=')
            params[key.strip()] = value.strip()
        manifest = start_run(args.kind, params, run_id=args.run_id, runs_dir=runs_dir)
        print(f"runId={manifest['runId']}")
        print(f"runDir={Path(runs_dir or DEFAULT_RUNS_DIR) / manifest['runId']}")
        return 0

    if args.action == 'finish':
        summary = {}
        if args.summary:
            summary = json.loads(Path(args.summary).read_text('utf-8'))
        snapshots = {}
        for item in args.snapshot:
            name, _, path = item.partition('=')
            snapshots[name] = json.loads(Path(path).read_text('utf-8'))
        record = finish_run(args.run_id, args.status, args.reason, summary, snapshots, runs_dir)
        print(json.dumps(record, ensure_ascii=False, indent=2, default=str))
        return 0

    run_id = args.run_id or latest_run(runs_dir=runs_dir)
    if not run_id:
        print('没有找到任何 run', file=sys.stderr)
        return 1
    runs = Path(runs_dir or os.environ.get('ROBOT_RUNS_DIR') or DEFAULT_RUNS_DIR)
    for name in (MANIFEST_NAME, SUMMARY_JSON):
        target = runs / run_id / name
        if target.is_file():
            print(f'--- {target} ---')
            print(target.read_text('utf-8'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
