#!/usr/bin/env bash
# start_rviz.sh —— RViz 观察入口：只启动可视化。
# 绝不启动 Nav2、AMCL、建图或任何控制节点；定位、目标、取消、急停一律走网页。
set -eo pipefail

TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
用法: scripts/start_rviz.sh (--sim | --real) [--diagnostics | --vision] [--config <文件>]

  --sim          隔离仿真观察：Domain 197 + localhost 发现 + UDPv4，真实时间。
                 先运行 scripts/offline_console.sh 并打开 http://localhost:5174/。
  --real         实车观察：项目实际配置（默认 Domain 0，默认发现范围），
                 不继承仿真的 localhost/UDPv4 限制。
  --diagnostics  使用 scripts/rviz/diagnostics.rviz（雷达/里程计/TF，Fixed Frame: odom）。
                 缺省使用 scripts/rviz/navigation_observer.rviz（地图/雷达/TF/路径/代价地图，
                 Fixed Frame: map，俯视视角）。
  --vision       原图 / YOLO 叠框观察；诊断请另用 ros2 topic echo /diagnostics。
  --config PATH  使用指定的 RViz 配置文件（覆盖上面两种缺省）。
  -h, --help     显示本帮助。

说明:
  * 两套配置只包含 rviz_default_plugins 已安装插件，无 nav2_rviz_plugins 依赖。
  * 配置中没有 2D Pose Estimate / Nav2 Goal / Publish Point 等会发布控制话题的工具；
    初始位置、目标点、取消与急停请在网页上操作。
  * navigation_observer 的 Fixed Frame 是 map：请先在网页开始建图或导航，
    再打开该视图，否则会出现 "Fixed Frame [map] does not exist"（属等待数据，非故障）。
  * 纯雷达/TF 体检请用 --diagnostics（Fixed Frame: odom，开机即有数据）。
EOF
}

MODE=''
CONFIG=''
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim) MODE='sim'; shift ;;
    --real) MODE='real'; shift ;;
    --diagnostics) CONFIG="$TASK_ROOT/scripts/rviz/diagnostics.rviz"; shift ;;
    --vision) CONFIG="$TASK_ROOT/scripts/rviz/vision.rviz"; shift ;;
    --config) CONFIG="${2:?--config 需要一个文件路径}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$MODE" ]]; then
  echo '错误: 必须指定 --sim 或 --real（两者环境不可混用）' >&2
  usage
  exit 2
fi

CONFIG="${CONFIG:-$TASK_ROOT/scripts/rviz/navigation_observer.rviz}"
if [[ ! -f "$CONFIG" ]]; then
  echo "错误: RViz 配置不存在: $CONFIG" >&2
  exit 1
fi

source "$TASK_ROOT/scripts/env.bash"
command -v rviz2 >/dev/null 2>&1 || {
  echo '错误: 未找到 rviz2；请确认 ROS 2 Lyrical 安装完整（本入口不安装系统软件）' >&2
  exit 1
}

if [[ "$MODE" == 'sim' ]]; then
  # 与 scripts/offline_console.sh 完全一致的仿真通信设置。
  export ROS_DOMAIN_ID=197
  export ROS_LOCALHOST_ONLY=1
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  echo '[start_rviz] 仿真观察: Domain 197 | localhost 发现 | UDPv4 | 真实时间(use_sim_time=false)'
  echo '[start_rviz] 请先运行 scripts/offline_console.sh；本入口只启动 RViz，不启动 Nav2/AMCL/建图。'
  echo '[start_rviz] 提示: map 坐标系在建图或导航启动后才存在；仅检查雷达/TF 请加 --diagnostics。'
else
  # 实车：清除仿真专用限制，恢复项目默认发现范围；Domain 未显式设置时按项目默认 0。
  unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE FASTDDS_BUILTIN_TRANSPORTS
  export ROS_DOMAIN_ID="${ROBOT_REAL_DOMAIN_ID:-0}"
  echo "[start_rviz] 实车观察: Domain ${ROS_DOMAIN_ID} | 默认发现范围（未继承仿真 localhost 限制）| 真实时间"
  echo '[start_rviz] 实车控制台入口是 web_console/start_console.sh；本入口只启动 RViz 观察。'
fi

# 重复实例只提醒，不杀任何进程。
RVIZ_COUNT="$(pgrep -xc rviz2 2>/dev/null || true)"
if [[ "${RVIZ_COUNT:-0}" -gt 0 ]]; then
  echo "[start_rviz] 警告: 检测到 ${RVIZ_COUNT} 个已在运行的 rviz2 进程；将再开一个独立实例，请自行确认哪个窗口在用。" >&2
fi

echo "[start_rviz] 配置: $CONFIG"
exec rviz2 -d "$CONFIG" --ros-args -p use_sim_time:=false
