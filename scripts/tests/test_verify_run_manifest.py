"""只读核验器的测试：证明"证据自洽"与"证据漂移"能被区分，且核验本身不写盘。"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_manifest as rm  # noqa: E402
import verify_run_manifest as vm  # noqa: E402


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_RUN_ACTIVE_MARKER', str(tmp_path / '.run-active'))
    return tmp_path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_run(runs: Path, sources: list[dict], summary: dict | None = None) -> Path:
    run_dir = runs / 'run_synth'
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / rm.MANIFEST_NAME).write_text(json.dumps(
        {'runId': 'run_synth', 'kind': 'bench', 'startedAt': '2026-11-11T00:00:00+08:00',
         'sources': sources}, ensure_ascii=False), 'utf-8')
    if summary is not None:
        (run_dir / rm.SUMMARY_JSON).write_text(json.dumps(summary, ensure_ascii=False), 'utf-8')
    return run_dir


def test_real_run_verifies_as_identical(isolated):
    """用真实的 start_run/finish 产物核验：全部源码指纹一致才给 0。"""
    runs = isolated / 'runs'
    rm.start_run('bench', {'probe': 'verify'}, run_id='run_real', runs_dir=runs)
    rm.finish_run('run_real', 'success', 'ok', runs_dir=runs)
    code, report = vm.verify('run_real', runs)
    assert code == vm.EXIT_OK
    assert report['drift'] == []
    assert report['coverage']['recorded'] == len(rm.WATCHED_SOURCES)
    assert all(item['state'] == 'identical' for item in report['files'])
    # 前端关键文件必须在覆盖范围内（此前 manifest 完全没有它们）
    covered = {item['path'] for item in report['files']}
    for required in ('web_console/src/App.vue', 'web_console/src/components/CameraView.vue',
                     'web_console/src/components/NavigationControl.vue',
                     'web_console/src/components/MappingControl.vue',
                     'web_console/src/components/SlamMap.vue', 'scripts/nav_stuck_policy.py'):
        assert required in covered, f'{required} 未进 manifest'


def test_single_byte_change_is_localized(isolated, monkeypatch):
    """改一个字节 → 必须精确指出是哪个文件，并给出记录值/当前值。"""
    root = isolated / 'repo'
    (root / 'src').mkdir(parents=True)
    target = root / 'src' / 'a.js'
    stable = root / 'src' / 'b.js'
    target.write_text('const a = 1\n', 'utf-8')
    stable.write_text('const b = 2\n', 'utf-8')
    monkeypatch.setattr(rm, 'REPO_ROOT', root)
    runs = isolated / 'runs'
    build_run(runs, [
        {'path': 'src/a.js', 'sha256': sha(target)},
        {'path': 'src/b.js', 'sha256': sha(stable)},
    ], {'runId': 'run_synth', 'status': 'success', 'snapshots': []})
    target.write_text('const a = 2\n', 'utf-8')          # 单字节变化
    code, report = vm.verify('run_synth', runs)
    assert code == vm.EXIT_DRIFT
    assert [item['path'] for item in report['drift']] == ['src/a.js']
    changed = report['drift'][0]
    assert changed['recorded'] != changed['current']
    assert changed['state'] == 'changed'


def test_missing_file_is_reported(isolated, monkeypatch):
    root = isolated / 'repo'
    root.mkdir()
    monkeypatch.setattr(rm, 'REPO_ROOT', root)
    runs = isolated / 'runs'
    build_run(runs, [{'path': 'src/gone.js', 'sha256': 'a' * 64}])
    code, report = vm.verify('run_synth', runs)
    assert code == vm.EXIT_DRIFT
    assert report['drift'][0]['state'] == 'missing'


def test_corrupt_manifest_exits_corrupt(isolated):
    run_dir = isolated / 'runs' / 'run_bad'
    run_dir.mkdir(parents=True)
    (run_dir / rm.MANIFEST_NAME).write_text('{not json', 'utf-8')
    code, report = vm.verify('run_bad', isolated / 'runs')
    assert code == vm.EXIT_CORRUPT
    assert '损坏' in report['error']


def test_missing_manifest_exits_not_found(isolated):
    code, report = vm.verify('run_nope', isolated / 'runs')
    assert code == vm.EXIT_NO_MANIFEST
    assert '找不到' in report['error']


def test_missing_summary_is_incomplete_not_pass(isolated):
    """没写 summary 的运行不能算"核验通过"。"""
    runs = isolated / 'runs'
    build_run(runs, [], summary=None)
    code, report = vm.verify('run_synth', runs)
    assert code == vm.EXIT_INCOMPLETE
    assert any('summary.json' in item for item in report['incomplete'])


def test_failed_run_without_snapshot_is_incomplete(isolated):
    runs = isolated / 'runs'
    build_run(runs, [], {'runId': 'run_synth', 'status': 'failed', 'snapshots': []})
    code, report = vm.verify('run_synth', runs)
    assert code == vm.EXIT_INCOMPLETE
    assert any('现场快照' in item for item in report['incomplete'])


def test_failed_run_with_snapshot_and_clean_sources_passes(isolated, monkeypatch):
    root = isolated / 'repo'
    (root / 'src').mkdir(parents=True)
    src = root / 'src' / 'a.js'
    src.write_text('x\n', 'utf-8')
    monkeypatch.setattr(rm, 'REPO_ROOT', root)
    runs = isolated / 'runs'
    snap = runs / 'run_synth' / 'checks' / 'localization.json'
    run_dir = build_run(runs, [{'path': 'src/a.js', 'sha256': sha(src)}],
                        {'runId': 'run_synth', 'status': 'failed', 'snapshots': [str(snap)]})
    snap.parent.mkdir(parents=True, exist_ok=True)
    snap.write_text('{}', 'utf-8')
    code, _report = vm.verify('run_synth', runs)
    assert code == vm.EXIT_OK, '有现场快照的失败运行，证据本身是自洽的'


def test_verification_never_writes(isolated, monkeypatch):
    """核验必须只读：归档文件的内容与 mtime 都不能变。"""
    root = isolated / 'repo'
    (root / 'src').mkdir(parents=True)
    src = root / 'src' / 'a.js'
    src.write_text('x\n', 'utf-8')
    monkeypatch.setattr(rm, 'REPO_ROOT', root)
    runs = isolated / 'runs'
    run_dir = build_run(runs, [{'path': 'src/a.js', 'sha256': sha(src)}],
                        {'runId': 'run_synth', 'status': 'success', 'snapshots': []})
    before = {item.name: (sha(item), item.stat().st_mtime_ns) for item in run_dir.rglob('*') if item.is_file()}
    vm.verify('run_synth', runs)
    vm.verify('run_synth', runs)
    after = {item.name: (sha(item), item.stat().st_mtime_ns) for item in run_dir.rglob('*') if item.is_file()}
    assert before == after, '核验器不得修改归档'


def test_coverage_warning_for_old_manifest_without_frontend_files(isolated):
    """旧 manifest 只记了少数文件时给覆盖警告，但不算漂移。"""
    runs = isolated / 'runs'
    build_run(runs, [{'path': 'web_console/server.js', 'sha256': sha(rm.REPO_ROOT / 'web_console/server.js')}],
              {'runId': 'run_synth', 'status': 'success', 'snapshots': []})
    code, report = vm.verify('run_synth', runs)
    assert code == vm.EXIT_OK, '旧归档用旧口径，不能因此判失败'
    assert report['coverage']['missingFromManifest']
    assert any('未记录' in item for item in report['warnings'])
