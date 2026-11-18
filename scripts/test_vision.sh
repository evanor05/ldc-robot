#!/usr/bin/env bash
# Reproducible tests; --soak runs the full real 1800-second GPU acceptance.
set -eo pipefail
VISION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$VISION_ROOT/scripts/env.bash"
cd "$VISION_ROOT"
MODE="${1:---unit}"
REPORT="${LDC_VISION_REPORT_DIR:-/tmp/ldc-vision-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$REPORT"
case "$MODE" in
  --unit)
    python3 scripts/vision/test_core.py 2>&1 | tee "$REPORT/unit.log"
    node --test web_console/tests/vision*.test.js 2>&1 | tee "$REPORT/web.log";;
  --integration)
    python3 scripts/vision/integration.py --image "${2:?需要 JPEG 样例路径}" --output "$REPORT/integration" 2>&1 | tee "$REPORT/integration.log";;
  --soak)
    python3 scripts/vision/run_soak.py --image "${2:?需要 JPEG 样例路径}" --output "$REPORT/soak" 2>&1 | tee "$REPORT/soak.log";;
  *) echo '用法: bash scripts/test_vision.sh --unit|--integration IMAGE.jpg|--soak IMAGE.jpg' >&2;exit 2;;
esac
echo "测试证据: $REPORT"
