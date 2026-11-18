#!/usr/bin/env bash
set -eo pipefail
LDC_COMPONENT="${1:?需要组件名}";shift
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo "用法: start_${LDC_COMPONENT}.sh [--real|--sim] [组件参数]；默认 --real。Qt 默认只观察，视觉默认 YOLO26n。"
  exit 0
fi
LDC_MODE=--real
if [[ "${1:-}" == --real || "${1:-}" == --sim ]]; then LDC_MODE="$1";shift;fi
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
ldc_environment "$LDC_MODE"
case "$LDC_COMPONENT" in
  agent)
    [[ "$LDC_MODE" == --real ]] || { echo '仿真不连接实车 Agent，请用 start_all.sh --sim' >&2;exit 2; }
    exec ros2 run micro_ros_agent micro_ros_agent udp4 -p "${ROBOT_AGENT_PORT:-8888}" -v 4 "$@";;
  chassis) exec ros2 launch ldc_bringup ldc_bringup.launch.py "$@";;
  mavlink)
    [[ "$LDC_MODE" == --real ]] || { echo '仿真不连接 MAVLink 实车' >&2;exit 2; }
    exec ros2 launch ldc_mavlink_bridge mavlink_bridge.launch.py "$@";;
  lidar)
    [[ "$LDC_MODE" == --real ]] || { echo '仿真激光由 offline_robot 提供' >&2;exit 2; }
    echo '仅外接主机串口雷达时使用；MCU 已发布 /scan 时不要再启动此驱动。'
    exec ros2 launch camsense_lidar camsense_lidar.launch.py "$@";;
  camera)
    if [[ "$LDC_MODE" == --sim ]]; then
      python3 - "$@" <<'PY'
import sys
from urllib.parse import urlsplit
url=next((v[5:] for v in reversed(sys.argv[1:]) if v.startswith('url:=')),'')
if urlsplit(url).hostname not in ('127.0.0.1','localhost','::1'):raise SystemExit('仿真相机必须显式传入 loopback url:=...')
PY
    fi
    exec ros2 launch ldc_camera http_video_publisher.launch.py publish_raw:=true "$@";;
  yolo) exec ros2 run ldc_vision detector --ros-args -p gateway_port:="$LDC_GATEWAY_PORT" "$@";;
  backend) cd "$LDC_ROOT/web_console";exec node server.js "$@";;
  frontend) cd "$LDC_ROOT/web_console";exec npm run dev -- --host 127.0.0.1 --port "$ROBOT_WEB_PORT" --strictPort "$@";;
  foxglove) exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:="$VITE_ROBOT_WS_PORT" \
    -p 'capabilities:=[connectionGraph]' -p 'min_qos_depth:=1' -p 'max_qos_depth:=10' \
    -p "topic_whitelist:=['^/(scan|odom|map|global_costmap/costmap|plan|tf|tf_static|battery_state|imu|diagnostics)$']" "$@";;
  mapping|navigation) exec python3 "$LDC_ROOT/scripts/runtime/task.py" "$LDC_COMPONENT" "$@";;
  *) echo "未知组件: $LDC_COMPONENT" >&2;exit 2;;
esac
