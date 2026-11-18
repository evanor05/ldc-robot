# Source from bash; mode is explicit for reusable module launchers.
ldc_environment() {
  local mode="${1:---real}"
  LDC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
  source "$LDC_ROOT/scripts/env.bash"
  case "$mode" in
    --real)
      export ROS_DOMAIN_ID="${ROBOT_REAL_DOMAIN_ID:-0}" ROBOT_SIMULATION=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
      unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
      export ROBOT_API_PORT="${ROBOT_API_PORT:-3000}" VITE_ROBOT_WS_PORT="${VITE_ROBOT_WS_PORT:-8765}"
      LDC_GATEWAY_PORT=18320;;
    --sim)
      export ROS_DOMAIN_ID=197 ROBOT_SIMULATION=1 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
      export ROBOT_API_PORT="${ROBOT_API_PORT:-18310}" VITE_ROBOT_WS_PORT="${VITE_ROBOT_WS_PORT:-8766}"
      LDC_GATEWAY_PORT=18321;;
    *) echo '模式必须是 --real 或 --sim' >&2; return 2;;
  esac
  export VITE_ROBOT_API_PORT="$ROBOT_API_PORT" ROBOT_WEB_PORT="${ROBOT_WEB_PORT:-5173}"
}
