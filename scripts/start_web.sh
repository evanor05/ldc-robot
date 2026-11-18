#!/usr/bin/env bash
# Compatibility web stack: backend + frontend + existing Foxglove startup task.
set -eo pipefail
LDC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then echo '用法: start_web.sh [--real]；完整仿真请用 start_all.sh --sim';exit 0;fi
if [[ "${1:-}" == --sim ]]; then echo '完整仿真请用 bash scripts/start_all.sh --sim（5173）' >&2;exit 2;fi
exec bash "$LDC_ROOT/web_console/start_console.sh"
