#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$TASK_ROOT/scripts/env.bash"
# Refresh this package's launch files before using the isolated stack.
colcon build --base-paths "$TASK_ROOT/robot_ws/src/ldc_navigation" \
  --build-base "$TASK_ROOT/robot_ws/build" --install-base "$TASK_ROOT/robot_ws/install" \
  --packages-select ldc_navigation > /tmp/ldc-offline-build.log 2>&1 || {
  cat /tmp/ldc-offline-build.log >&2
  exit 1
}
export ROS_DOMAIN_ID=197
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROBOT_SIMULATION=1
export ROBOT_API_PORT=18310
export VITE_ROBOT_API_PORT=18310
export VITE_ROBOT_WS_PORT=8766
exec python3 "$TASK_ROOT/scripts/offline_console.py" "$@"
