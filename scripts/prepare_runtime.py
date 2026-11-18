#!/usr/bin/python3
"""Seed editable runtime maps once, preserving existing files and baseline sources."""
import os
from pathlib import Path
import shutil
root=Path(__file__).resolve().parents[1]
dest=Path(os.environ.get('ROBOT_MAPS_DIR', root/'maps')).expanduser().resolve()
dest.mkdir(parents=True,exist_ok=True)
source=root/'robot_ws/src/ldc_navigation/maps'
for src in source.iterdir():
    if src.suffix not in ('.pgm','.yaml'):continue
    target=dest/src.name
    try:
        with target.open('xb') as output, src.open('rb') as input_file:
            shutil.copyfileobj(input_file,output)
    except FileExistsError:
        pass
print(f'Runtime maps: {dest}')
