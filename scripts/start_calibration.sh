#!/usr/bin/env bash
# Real-car calibration wizard; reuse an existing console or own a new web stack.
set -eo pipefail
LDC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3 "$LDC_ROOT/scripts/open_calibration.py" "$@"
