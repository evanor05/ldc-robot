#!/usr/bin/env bash
# Start the mapping stack. Manual (default) keeps the existing GMapping launch;
# --auto hands mapping to the ldc_exploration autonomous frontier explorer.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/runtime/environment.bash"
ldc_environment --real

MODE=manual
for arg in "$@"; do
  case "$arg" in
    --auto) MODE=auto;;
    --manual) MODE=manual;;
    -h|--help)
      echo "用法: start_mapping_task.sh [--auto|--manual]"
      echo "  --auto    ldc_exploration auto_mapping.launch.py rviz:=false"
      echo "  --manual  slam_gmapping.launch.py（默认）"
      exit 0;;
    *)
      echo "未知参数: $arg" >&2
      exit 2;;
  esac
done

if [[ "$MODE" == auto ]]; then
  exec ros2 launch ldc_exploration auto_mapping.launch.py rviz:=false
fi
exec ros2 launch slam_gmapping slam_gmapping.launch.py
