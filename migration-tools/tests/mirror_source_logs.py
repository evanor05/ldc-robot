"""Regression: source log directories survive mirror creation; runtime logs do not."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile

script = Path(__file__).resolve().parents[1] / 'prepare_mirror.py'
spec = importlib.util.spec_from_file_location('mirror', script)
mirror = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mirror)
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    mirror.ROOT = root / 'output'
    mirror.ROOT.mkdir()
    source = root / 'input'
    source.mkdir()
    mirror.SOURCES = {'xuegeros_ws/src': source / 'host', 'leap_demo': source / 'firmware'}
    mirror.REVIEWED_LITERAL_LINES = {}
    expected = [path + '/fixture.cpp' for path in mirror.SOURCE_LOG_DIRS]
    runtime = ['xuegeros_ws/src/other/log/runtime.txt', 'leap_demo/other/log/runtime.txt']
    for path in expected + runtime:
        prefix = next(k for k in mirror.SOURCES if path.startswith(k + '/'))
        target = mirror.SOURCES[prefix] / path[len(prefix) + 1:]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('source fixture\n')
    with contextlib.redirect_stdout(io.StringIO()): mirror.main()
    for path in expected: assert (mirror.ROOT / path).exists(), path
    for path in runtime: assert not (mirror.ROOT / path).exists(), path
print('PASS: three required upstream log source paths retained; runtime logs excluded')
