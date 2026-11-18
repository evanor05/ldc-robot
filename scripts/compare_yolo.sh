#!/usr/bin/env bash
set -eo pipefail
LDC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$LDC_ROOT/scripts/env.bash"
exec python3 "$LDC_ROOT/scripts/calibration/compare_models.py" "$@"
