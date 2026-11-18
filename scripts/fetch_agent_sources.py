#!/usr/bin/python3
"""Fetch immutable, hash-checked Agent sources without altering the Humble baseline."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

workspace = Path(__file__).resolve().parents[1] / 'agent_ws'
for source in json.loads((workspace / 'lyrical-sources.json').read_text())['sources']:
    archive = workspace / '.deps' / source['archive']
    archive.parent.mkdir(parents=True, exist_ok=True)
    if not archive.exists():
        url = f"https://codeload.github.com/{source['repo']}/tar.gz/{source['commit']}"
        partial = archive.with_suffix('.part')
        subprocess.run(['curl', '-fL', '--retry', '3', '--max-time', '180', url, '-o', str(partial)], check=True)
        partial.rename(archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != source['sha256']:
        raise RuntimeError(f"Archive checksum mismatch: {archive}")
    dest = workspace / source['destination']
    marker = dest / '.migration-source-sha256'
    if dest.exists():
        if marker.exists() and marker.read_text().strip() == source['sha256']:
            print(f"Verified archive; using existing {source['name']}")
            continue
        raise RuntimeError(f"Existing unmarked source directory: {dest}; preserve it before fetching")
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=dest.parent) as directory:
        with tarfile.open(archive) as tar:
            tar.extractall(directory, filter='data')
        roots = list(Path(directory).iterdir())
        if len(roots) != 1 or not roots[0].is_dir():
            raise RuntimeError(f"Unexpected archive layout: {archive}")
        roots[0].rename(dest)
    marker.write_text(source['sha256'] + '\n')
    print(f"Fetched {source['name']} @ {source['commit']}")
