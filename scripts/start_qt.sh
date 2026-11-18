#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$TASK_ROOT/scripts/env.bash"
case "${1:-}" in
  --real)
    export ROS_DOMAIN_ID="${ROBOT_REAL_DOMAIN_ID:-0}"
    unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE FASTDDS_BUILTIN_TRANSPORTS
    api="http://127.0.0.1:${ROBOT_API_PORT:-3000}/api"
    ;;
  --sim)
    export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    api="http://127.0.0.1:${ROBOT_API_PORT:-18310}/api"
    ;;
  *) echo '用法: bash scripts/start_qt.sh (--real|--sim) [--enable]；默认只观察，--enable 明确启用遥控'; exit 2 ;;
esac
shift
exec python3 "$TASK_ROOT/scripts/qt_console_control.py" --api "$api" "$@"
