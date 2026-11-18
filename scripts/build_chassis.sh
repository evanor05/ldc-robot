#!/usr/bin/env bash
# Isolated source snapshot: SDK/component generation never rewrites the worktree.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$#" != 2 || "$1" != --output ]]; then
  echo '用法: bash scripts/build_chassis.sh --output <不存在的新输出目录>' >&2
  exit 2
fi
BUILD_OUTPUT="$(realpath -m -- "$2")"
case "$BUILD_OUTPUT/" in "$ROOT/"*) echo '输出目录必须在仓库外，避免递归复制或覆盖源码' >&2; exit 2;; esac
[[ ! -e "$BUILD_OUTPUT" ]] || { echo '输出目录已存在，请使用新目录以保留构建证据' >&2; exit 2; }
export IDF_PATH="${LDC_IDF_PATH:-/home/<user>/esp/esp-idf-v5.5.3}"
export IDF_TOOLS_PATH="${LDC_IDF_TOOLS_PATH:-/home/<user>/esp/.espressif}"
export IDF_PYTHON_ENV_PATH="${LDC_IDF_PYTHON_ENV:-/home/<user>/esp/idf-python-env}"
[[ -f "$IDF_PATH/export.sh" && -x "$IDF_PYTHON_ENV_PATH/bin/python" ]] || {
  echo 'ESP-IDF 或 Python 环境不存在；请设置 LDC_IDF_PATH / LDC_IDF_TOOLS_PATH / LDC_IDF_PYTHON_ENV' >&2; exit 1;
}
export PATH="$IDF_PYTHON_ENV_PATH/bin:$PATH"
source "$IDF_PATH/export.sh" >/dev/null
mkdir -p -- "$(dirname -- "$BUILD_OUTPUT")"
mkdir -- "$BUILD_OUTPUT"
python "$IDF_PATH/tools/idf_tools.py" check-python-dependencies > "$BUILD_OUTPUT/environment.log" 2>&1
python - "$ROOT/firmware/chassis" "$BUILD_OUTPUT" <<'PY'
from pathlib import Path
import hashlib,json,shutil,sys
src,out=map(Path,sys.argv[1:])
shutil.copytree(src,out/'source',ignore=shutil.ignore_patterns('.git','build','__pycache__','sdkconfig.old'))
manifest={str(p.relative_to(out/'source')):hashlib.sha256(p.read_bytes()).hexdigest()
          for folder in ('main','components','extra_ros_packages')
          for p in (out/'source'/folder).rglob('*') if p.is_file()}
for name in ('sdkconfig','sdkconfig.defaults','dependencies.lock','app-colcon.meta'):
    p=out/'source'/name
    if p.exists(): manifest[name]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'source-sha256.json').write_text(json.dumps(manifest,indent=2))
PY
cd "$BUILD_OUTPUT/source"
# Limit regular build jobs; the vendor micro-ROS makefile retains its own job policy.
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
python "$IDF_PATH/tools/idf.py" -B "$BUILD_OUTPUT/build" build > "$BUILD_OUTPUT/build.log" 2>&1 || {
  echo "固件构建失败；日志: $BUILD_OUTPUT/build.log" >&2
  tail -35 "$BUILD_OUTPUT/build.log" >&2
  exit 1
}
python - "$BUILD_OUTPUT" <<'PY'
from pathlib import Path
import hashlib,json,sys
out=Path(sys.argv[1]);artifacts={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (out/'build').rglob('*') if p.is_file() and p.suffix in ('.bin','.elf')}
assert artifacts,'构建成功但未找到固件产物'
(out/'artifacts-sha256.json').write_text(json.dumps(artifacts,indent=2))
print('固件构建通过；产物与校验值:',out)
PY
