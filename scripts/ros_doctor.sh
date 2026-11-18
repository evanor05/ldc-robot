#!/usr/bin/env bash
# ros_doctor.sh —— ROS 2 排障诊断入口。
# 输出中文报告，每项标注 [正常] / [等待数据] / [故障] / [信息]，附原因与下一步。
# 只读诊断：不杀进程、不改系统默认 ROS/RMW、不清理、不重装。
# 注意: ROS 的 setup.bash 不是 nounset 安全的（AMENT_TRACE_SETUP_FILES 未绑定），不能开 set -u。
set -o pipefail

TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
用法: scripts/ros_doctor.sh (--sim | --real) [--log <文件>]

  --sim    诊断隔离仿真（Domain 197 + localhost 发现 + UDPv4，与 offline_console.sh 一致）
  --real   诊断实车链路（项目实际配置，默认 Domain 0，不继承仿真 localhost 限制）
  --log    指定报告日志输出路径（缺省 /tmp/ros_doctor_<模式>_<时间>.log）

检查内容: 环境与通信配置、控制台进程与端口、后端状态 API、ros2 CLI 守护进程一致性、
节点、话题/类型/QoS/重复发布者、消息频率、TF 链与时间源、生命周期与 Action、重复进程。
退出码: 0=正常或等待数据；1=存在故障项；2=用法错误。
EOF
}

MODE=''
LOG=''
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim) MODE='sim'; shift ;;
    --real) MODE='real'; shift ;;
    --log) LOG="${2:?--log 需要文件路径}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
done
[[ -n "$MODE" ]] || { usage; exit 2; }

LOG="${LOG:-/tmp/ros_doctor_${MODE}_$(date +%Y%m%d_%H%M%S).log}"
exec > >(tee "$LOG") 2>&1

OK_N=0; WAIT_N=0; FAIL_N=0
ok()   { OK_N=$((OK_N+1));   echo "[正常]   $*"; }
wait_(){ WAIT_N=$((WAIT_N+1));echo "[等待数据] $*"; }
fail() { FAIL_N=$((FAIL_N+1));echo "[故障]   $*"; }
info() { echo "[信息]   $*"; }
hdr()  { echo; echo "== $* =="; }

# run <超时秒> <命令...>：记录输出与退出码；OUT/RC 供后续解析。
OUT=''; RC=0
run() {
  local t="$1"; shift
  OUT="$(timeout "$t" "$@" 2>&1)"; RC=$?
}

echo "ROS 2 诊断报告（$MODE 模式） $(date '+%F %T %Z')"
echo "日志文件: $LOG"

# ---------- 0. 环境 ----------
hdr "0. 环境与通信配置"
if ! source "$TASK_ROOT/scripts/env.bash"; then
  fail "scripts/env.bash 加载失败：当前 shell 可能混入了其他 ROS 发行版。下一步: 新开一个干净终端重试。"
  echo; echo "总体结论: 故障 (1 项)"; exit 1
fi
info "ROS_DISTRO=${ROS_DISTRO:-<未设置>}（env.bash 已强制校验为 lyrical，混入其他发行版会直接报错退出）"
info "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<未设置，使用发行版默认 rmw_fastrtps_cpp>}（本脚本不修改系统默认）"
info "rviz2: $(command -v rviz2 || echo '<未找到>')；导航包: $(command -v ros2 >/dev/null && timeout 15 ros2 pkg prefix ldc_navigation 2>/dev/null || echo '<未构建，运行一次 scripts/offline_console.sh 可重建>')"

if [[ "$MODE" == 'sim' ]]; then
  export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  ok "仿真通信配置: Domain 197 | ROS_LOCALHOST_ONLY=1 | 发现范围 LOCALHOST | FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
  API_PORT="${ROBOT_API_PORT:-18310}"; WEB_PORT=5174; FOX_PORT=8766
else
  unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE FASTDDS_BUILTIN_TRANSPORTS
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  ok "实车通信配置: Domain ${ROS_DOMAIN_ID} | 默认发现范围 | 未继承仿真 localhost/UDPv4 限制"
  API_PORT="${ROBOT_API_PORT:-3000}"; WEB_PORT=5173; FOX_PORT=8765
fi
API="http://localhost:${API_PORT}/api"

# ---------- 1. 控制台进程与端口 ----------
hdr "1. 控制台进程与端口"
port_state() { # $1=端口 → 输出 busy/free
  python3 - "$1" <<'PY'
import socket, sys
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(('127.0.0.1', int(sys.argv[1])))
    print('free')
except OSError:
    print('busy')
finally:
    s.close()
PY
}
count_proc() { pgrep -fc "$1" 2>/dev/null || true; }

if [[ "$MODE" == 'sim' ]]; then
  CONSOLE_N="$(count_proc 'scripts/offline_console.py')"; ROBOT_N="$(count_proc 'scripts/offline_robot.py')"
  info "offline_console.py 进程数: ${CONSOLE_N:-0}；offline_robot.py 进程数: ${ROBOT_N:-0}"
  if [[ "${CONSOLE_N:-0}" -gt 1 ]]; then
    fail "检测到多个仿真控制台进程。原因: 重复启动会产生双份 /tf 与雷达发布。下一步: 逐个终端 Ctrl+C 停止，只保留一个；本脚本不代杀。"
  fi
fi
BACKEND_N="$(count_proc 'web_console.*server\.js|node server\.js')"; BACKEND_N="${BACKEND_N:-0}"
[[ "$BACKEND_N" -gt 1 ]] && BACKEND_N="$(pgrep -f 'node server.js' | wc -l | tr -d ' ')"
info "后端 server.js 进程数: $BACKEND_N"

API_PORT_STATE="$(port_state "$API_PORT")"; WEB_PORT_STATE="$(port_state "$WEB_PORT")"
if [[ "$API_PORT_STATE" == busy ]]; then ok "API 端口 ${API_PORT} 已被控制台占用（预期行为）"; else
  if [[ "$MODE" == 'sim' ]]; then
    wait_ "API 端口 ${API_PORT} 空闲：仿真控制台未运行。下一步: 终端一执行 scripts/offline_console.sh，再重跑本诊断。"
  else
    wait_ "API 端口 ${API_PORT} 空闲：实车控制台未运行。下一步: 执行 bash web_console/start_console.sh，再重跑本诊断。"
  fi
fi
CONSOLE_UP=$([[ "$API_PORT_STATE" == busy ]] && echo 1 || echo 0)

# ---------- 2. 后端状态 API ----------
hdr "2. 后端状态 API"
PHASE=''; READY=''; MISSING_JSON=''; NAV_JSON=''; MAP_JSON=''; MAPPING_RUN=''
if [[ "$CONSOLE_UP" == 1 ]]; then
  run 8 curl -fsS "$API/navigation_status"
  if [[ "$RC" == 0 ]]; then
    NAV_JSON="$OUT"
    eval "$(python3 - "$OUT" <<'PY'
import json, shlex, sys
d = json.loads(sys.argv[1])
print(f"PHASE={shlex.quote(str(d.get('phase','')))}; READY={shlex.quote(str(d.get('ready','')))}; MISSING_JSON={shlex.quote(json.dumps(d.get('missing',[]),ensure_ascii=False))}; ERR={shlex.quote(str(d.get('error','')))}; LOCALIZED={shlex.quote(str(d.get('localized','')))}; MODE_={shlex.quote(str(d.get('mode','')))}; GOAL={shlex.quote(str(d.get('goalState','')))}")
PY
    )"
    info "导航: phase=${PHASE} ready=${READY} localized=${LOCALIZED:-?} mode=${MODE_:-?} goalState=${GOAL:-?}"
    if [[ "$READY" == True || "$READY" == true ]]; then
      ok "导航就绪（节点激活、Action 可用、定位有效、传感器新鲜）"
    else
      if [[ -n "${MISSING_JSON:-}" && "$MISSING_JSON" != '[]' ]]; then
        echo "         未满足条件:"
        python3 -c "import json,sys; [print('         - '+m) for m in json.loads(sys.argv[1])]" "$MISSING_JSON"
      fi
      case "$PHASE" in
        stopped)   wait_ "导航未启动（phase=stopped）。下一步: 网页选择地图开始导航；建图请先点“开始建图”。" ;;
        waiting_localization) wait_ "Nav2 已启动、等待初始定位（phase=waiting_localization）。下一步: 网页“设置初始位置和朝向”，等 AMCL 收敛出 map→odom TF。上面列出的未激活节点会在定位后由后端顺序激活。" ;;
        starting)  wait_ "导航正在启动（phase=starting）。下一步: 等待节点激活（真正启动超时 60 秒），完成后重跑诊断。" ;;
        canceled|estop|error|fault) fail "导航处于 ${PHASE} 状态，错误: ${ERR:-<无>}。下一步: 阅读网页提示；急停后需明确重新启用；传感器断流恢复后不会自动续跑，需重新下发目标。" ;;
        failed)      fail "导航 phase=failed，错误: ${ERR:-<无>}。下一步: 查看后端日志与生命周期节点退出记录。" ;;
        running|succeeded) wait_ "phase=${PHASE} 但 ready=False。下一步: 对照上面“未满足条件”逐项处理（常见为传感器断流、定位失效或节点退出后未恢复）。" ;;
        *)         wait_ "phase=${PHASE}，尚未就绪。下一步: 对照上面“未满足条件”逐项处理。" ;;
      esac
    fi
    [[ -n "${ERR:-}" ]] && info "后端 error 字段: ${ERR}"
  else
    fail "GET $API/navigation_status 失败 (exit=$RC): $OUT。下一步: 查看控制台终端与日志目录中的 backend.log。"
  fi
  run 8 curl -fsSL "$API/task_status/mapping"
  if [[ "$RC" == 0 ]]; then
    MAP_JSON="$OUT"
    MAPPING_RUN="$(python3 -c "import json,sys; print(json.loads(sys.argv[1]).get('isRunning'))" "$OUT" 2>/dev/null || echo '?')"
    LAST_MAP="$(python3 -c "import json,sys; d=json.loads(sys.argv[1]).get('lastSavedMap') or {}; print(d.get('name','<无>'))" "$OUT" 2>/dev/null || echo '?')"
    if [[ "$MAPPING_RUN" == True ]]; then ok "建图运行中（slam_gmapping）；最近保存地图: ${LAST_MAP}"; else info "建图未运行；最近保存地图: ${LAST_MAP}"; fi
  else
    info "GET task_status/mapping 失败 (exit=$RC)（控制台版本较旧时属可接受）"
  fi
else
  wait_ "控制台未运行，跳过 API 检查（当前无数据）。"
fi

# ---------- 3. ros2 CLI 守护进程一致性 ----------
hdr "3. ros2 CLI 守护进程一致性"
run 25 ros2 node list
DAEMON_LIST="$OUT"; DAEMON_RC=$RC
run 35 ros2 node list --no-daemon --spin-time 5
DIRECT_LIST="$OUT"
if [[ -z "${DAEMON_LIST// /}" && -n "${DIRECT_LIST// /}" ]]; then
  info "守护进程列表为空但直连发现正常：守护进程是在其他 Domain/环境启动的（陈旧缓存），已在本脚本环境中重启守护进程。这不修改系统默认配置。"
  timeout 10 ros2 daemon stop >/dev/null 2>&1
  run 25 ros2 node list
  DAEMON_LIST="$OUT"
  if [[ -n "${DAEMON_LIST// /}" ]]; then ok "守护进程重启后可见节点 $(echo "$DAEMON_LIST" | wc -l) 个"; else wait_ "守护进程重启后仍无节点（当前无数据）"; fi
elif [[ -z "${DAEMON_LIST// /}" && -z "${DIRECT_LIST// /}" ]]; then
  wait_ "守护进程与直连发现都看不到任何节点（当前无数据）。下一步: 确认控制台/车端已启动，且本脚本模式(--sim/--real)与实际运行的 Domain 一致。"
else
  ok "ros2 CLI 守护进程与当前环境一致"
fi
# 直连发现的快照窗口较短可能漏节点；取守护进程与直连两路结果的并集，避免误报“节点缺失”。
NODES="$(printf '%s\n%s\n' "$DAEMON_LIST" "$DIRECT_LIST" | sed '/^[[:space:]]*$/d' | sort -u)"

# ---------- 4. 节点 ----------
hdr "4. 节点"
if [[ -n "${NODES// /}" ]]; then
  echo "$NODES" | sort | sed 's/^/         /'
  ok "发现 $(echo "$NODES" | wc -l) 个节点"
  for dup in $(echo "$NODES" | sort | uniq -d); do
    fail "重复节点名: $dup。原因: 同一 Domain 内跑了双份栈，/tf 与话题会互相覆盖。下一步: 找到并停止多余进程（ps -eo pid,cmd | grep <节点相关进程>）；本脚本不代杀。"
  done
else
  if [[ "$MODE" == 'sim' ]]; then
    if [[ "$CONSOLE_UP" == 1 ]]; then fail "控制台在运行但 ROS 图内无节点。原因: 可能 Domain/发现范围不一致或 DDS 异常。下一步: 核对控制台终端 env 与本脚本 --sim 一致；查看运行目录 robot.log/backend.log。"
    else wait_ "仿真控制台未运行，无节点可查（当前无数据）。"; fi
  else
    wait_ "未发现任何节点：车端未上电/驱动未启动，或 Domain 与车端不一致（当前无数据）。下一步: 确认车辆电源、雷达与底盘驱动、车上电脑与本机的 ROS_DOMAIN_ID 一致。"
  fi
fi
# 关键节点在位性（仿真确定性栈）
if [[ "$MODE" == 'sim' && "$CONSOLE_UP" == 1 ]]; then
  for n in /offline_console_robot /console_navigation_bridge /foxglove_bridge; do
    if echo "$NODES" | grep -qx "$n"; then ok "关键节点在位: $n"; else fail "关键节点缺失: $n。下一步: 查看运行目录对应日志（robot.log/backend.log/foxglove.log），重启控制台。"; fi
  done
  if [[ "$PHASE" == waiting_localization || "$READY" == True || "$READY" == true ]]; then
    for n in /map_server /amcl /controller_server /planner_server /bt_navigator; do
      echo "$NODES" | grep -qx "$n" || fail "导航节点缺失: $n（导航阶段应在位）。下一步: 查看后端日志中 launch 输出。"
    done
  fi
  if [[ "$MAPPING_RUN" == True ]] && ! echo "$NODES" | grep -qx /slam_gmapping; then
    fail "建图运行中但 /slam_gmapping 节点缺失。下一步: 查看 backend.log 中 slam_gmapping 是否崩溃（退出码/回溯）。"
  fi
fi

# ---------- 5. 话题 / 类型 / QoS / 重复发布者 ----------
hdr "5. 话题、类型与 QoS"
run 30 ros2 topic list -t
TOPICS="$OUT"
if [[ -z "${TOPICS// /}" ]]; then
  wait_ "话题列表为空（当前无数据）。"
else
  echo "$TOPICS" | grep -v '_action/' | sed 's/^/         /' | head -40
fi
topic_exists() { echo "$TOPICS" | grep -q "^$1 \["; }

# publisher_qos <话题> → 输出每个发布者的 "节点 RELIABILITY DURABILITY"
publisher_qos() {
  timeout 20 ros2 topic info "$1" --verbose 2>/dev/null | python3 -c '
import sys, re
text = sys.stdin.read()
blocks = re.split(r"\nNode name: ", text)[1:]
for b in blocks:
    name = b.split("\n", 1)[0].strip()
    if "Endpoint type: PUBLISHER" not in b:
        continue
    rel = re.search(r"Reliability: (\w+)", b)
    dur = re.search(r"Durability: (\w+)", b)
    print(name, rel.group(1) if rel else "?", dur.group(1) if dur else "?")
'
}
check_sensor_topic() { # $1 话题 $2 期望类型 $3 期望可靠性(BEST_EFFORT|RELIABLE|ANY) $4 期望持久性
  local t="$1" type="$2" rel_want="$3" dur_want="$4"
  if ! topic_exists "$t"; then
    if [[ "$MODE" == sim && "$CONSOLE_UP" == 1 ]]; then
      fail "$t 不存在（仿真控制台在运行时应有数据）。下一步: 查看 robot.log；确认雷达/里程计驱动或仿真节点是否退出。"
    else
      wait_ "$t 不存在（当前无数据）。下一步: 仿真模式先启动 offline_console.sh；实车模式确认车端驱动已发布该话题。"
    fi
    return
  fi
  local line; line="$(echo "$TOPICS" | grep "^$t \[")"
  info "$line"
  if ! echo "$line" | grep -q "$type"; then
    fail "$t 类型与期望 $type 不符。下一步: 核对发布端消息定义。"
  fi
  local pubs; pubs="$(publisher_qos "$t")"
  local n; n="$(echo "$pubs" | grep -c . || true)"
  if [[ "$n" == 0 ]]; then
    wait_ "$t 无发布者（当前无数据）。"
    return
  fi
  if [[ "$n" -gt 1 ]]; then
    case "$t" in
      /scan|/odom)
        fail "$t 有 $n 个发布者: $(echo "$pubs" | awk '{print $1}' | paste -sd, -)。原因: 传感器数据双发布造成抖动与时间戳混乱。下一步: 停止多余的发布节点（常见于同时跑了仿真与实车栈、或双开控制台）。" ;;
      /tf|/tf_static)
        info "$t 有 $n 个发布者（$(echo "$pubs" | awk '{print $1}' | paste -sd, -)）: /tf 多发布者本身合法（如 amcl 发 map→odom、底盘发 odom→base）；真正的冲突（map→odom 双发布）在第 7 节单独判定。" ;;
      *)
        info "$t 有 $n 个发布者，请人工确认是否预期。" ;;
    esac
  fi
  echo "$pubs" | sed 's/^/         发布者: /'
  local rel dur
  rel="$(echo "$pubs" | head -1 | awk '{print $2}')"; dur="$(echo "$pubs" | head -1 | awk '{print $3}')"
  if [[ "$rel_want" != ANY && "$rel" != "$rel_want" ]]; then
    fail "$t 发布端 Reliability=$rel，期望 $rel_want。影响: RViz/订阅端 QoS 不兼容会收不到数据。下一步: 修改发布端 QoS 或按实际值调整 RViz 显示项的 Reliability。"
  elif [[ "$dur_want" != ANY && "$dur" != "$dur_want" ]]; then
    fail "$t 发布端 Durability=$dur，期望 $dur_want。下一步: 同上，保持发布端与订阅端兼容。"
  else
    ok "$t 发布端 QoS 符合预期（$rel / $dur，发布者 $n 个）"
  fi
}
check_sensor_topic /scan  'sensor_msgs/msg/LaserScan' BEST_EFFORT VOLATILE
check_sensor_topic /odom  'nav_msgs/msg/Odometry'     BEST_EFFORT ANY
check_sensor_topic /tf    'tf2_msgs/msg/TFMessage'    ANY ANY
check_sensor_topic /tf_static 'tf2_msgs/msg/TFMessage' ANY TRANSIENT_LOCAL
# /map 与导航话题：建图或导航阶段才存在
MAP_EXPECTED=0
[[ "$MAPPING_RUN" == True || "$PHASE" != stopped && -n "$PHASE" ]] && MAP_EXPECTED=1
if topic_exists '/map'; then
  check_sensor_topic /map 'nav_msgs/msg/OccupancyGrid' RELIABLE TRANSIENT_LOCAL
  info "RViz“地图 /map”显示项已按 Reliable + Transient Local 配置，与发布端一致。"
else
  if [[ "$MAP_EXPECTED" == 1 ]]; then
    fail "建图/导航阶段却没有 /map。原因: gmapping 或 map_server 未正常发布。下一步: 查看后端日志中 gmapping/map_server 输出与退出码。"
  else
    wait_ "/map 不存在：尚未开始建图或导航（当前无数据，属正常）。"
  fi
fi
if [[ "$READY" == True || "$READY" == true ]]; then
  check_sensor_topic /global_costmap/costmap 'nav_msgs/msg/OccupancyGrid' RELIABLE TRANSIENT_LOCAL
  check_sensor_topic /local_costmap/costmap  'nav_msgs/msg/OccupancyGrid' RELIABLE TRANSIENT_LOCAL
  check_sensor_topic /particle_cloud 'nav2_msgs/msg/ParticleCloud' BEST_EFFORT VOLATILE
  info "说明: /particle_cloud 是 nav2_msgs/ParticleCloud 类型，rviz_default_plugins 没有对应显示项（粒子群显示需要未安装的 nav2_rviz_plugins），观察配置用 /amcl_pose 展示定位位姿与协方差。"
  check_sensor_topic /amcl_pose 'geometry_msgs/msg/PoseWithCovarianceStamped' RELIABLE TRANSIENT_LOCAL
  info "RViz“AMCL 定位位姿”显示项已按 Reliable + Transient Local 配置，与发布端一致（新订阅立即收到最近定位）。"
  topic_exists /plan && info "/plan 在位（仅在规划/重规划瞬间发布，属 Volatile：导航执行中打开 RViz 才能看到路径）"
else
  info "导航未就绪，跳过代价地图/粒子/路径检查（当前无数据）。"
fi

# ---------- 6. 消息频率 ----------
hdr "6. 消息频率"
rate_of() { # $1 话题 $2 采样秒 → OUT 为 average rate 或空
  run "$2" ros2 topic hz "$1" --window 20
  echo "$OUT" | grep 'average rate' | tail -1 | awk '{print $3}'
}
for t in /scan /odom; do
  if topic_exists "$t"; then
    R="$(rate_of "$t" 12)"
    if [[ -n "$R" ]]; then ok "$t 频率 ${R} Hz（仿真设计值 20 Hz）"; else
      if [[ "$MODE" == sim && "$CONSOLE_UP" == 1 ]]; then
        fail "$t 12 秒内无消息。原因: 发布者存在但不发数据（如传感器断流测试中）。下一步: 查看 robot.log；确认没有其他程序向仿真节点发送了断流信号。"
      else wait_ "$t 无消息（当前无数据）。"; fi
    fi
  fi
done
if topic_exists '/map'; then
  if [[ "$MAPPING_RUN" == True ]]; then
    R="$(rate_of /map 12)"
    if [[ -n "$R" ]]; then ok "/map 更新频率 ${R} Hz（建图阶段按地图更新间隔发布，低频属正常）"
    else info "/map 12 秒内无新消息：GMapping 在静止场景更新慢，结合 RViz 地图项 Status 判断。"; fi
  else
    run 8 ros2 topic echo /map --once --no-arr --field info.resolution
    if [[ "$RC" == 0 && -n "${OUT// /}" ]]; then
      ok "新订阅立即收到 /map 缓存帧（Transient Local 生效），地图分辨率 ${OUT} m/格"
    else
      fail "/map 存在但 8 秒内收不到缓存帧 (exit=$RC)。下一步: 查看 map_server/gmapping 日志，停止并重新启动导航以重载地图。"
    fi
  fi
fi

# ---------- 7. TF 链与时间源 ----------
hdr "7. TF 链与时间源"
LASER_FRAME=''
if topic_exists /scan; then
  run 8 ros2 topic echo /scan --once --field header.frame_id
  LASER_FRAME="$(echo "$OUT" | tr -d "'\" " | head -1)"
  [[ -n "$LASER_FRAME" ]] && info "/scan 的 header.frame_id = ${LASER_FRAME}"
fi
tf_echo() { # $1 源 $2 目标 $3 标签
  local attempt rc out
  for attempt in 1 2; do
    run 10 ros2 run tf2_ros tf2_echo "$1" "$2"
    out="$OUT"; rc=$RC
    if echo "$out" | grep -q '^At time'; then
      if [[ "$attempt" == 1 ]]; then ok "TF $1 → $2 存在（$3）"
      else ok "TF $1 → $2 第 2 次尝试成功：短暂未就绪后自行恢复，属启动暖机，非持续异常（$3）"; fi
      echo "$out" | grep -m1 'Translation' | sed 's/^/         /'
      return 0
    fi
    echo "$out" | grep -qi 'extrapolat' && { fail "TF $1 → $2 出现外推(extrapolation)错误。原因: 时间戳/发布频率/时间源问题，不能只增大超时。下一步: 检查两坐标系发布者的时间戳与系统时钟差、是否有重复发布者、use_sim_time 是否一致。"; return 1; }
    sleep 1
  done
  if echo "$out" | grep -q 'does not exist'; then
    return 2
  fi
  fail "TF $1 → $2 持续不可用（两次尝试均失败, exit=$rc）。下一步: ros2 run tf2_tools view_frames 生成 TF 树核对断链位置。"
  return 1
}
if [[ -n "$LASER_FRAME" ]]; then
  tf_echo odom "$LASER_FRAME" "里程计→雷达安装位"
  case $? in
    2) if [[ "$MODE" == sim && "$CONSOLE_UP" == 1 ]]; then
         fail "odom 或 ${LASER_FRAME} 坐标系不存在但仿真在运行。下一步: 检查 offline_robot.py 的 TF 发布是否退出。"
       else wait_ "odom/${LASER_FRAME} 坐标系尚不存在（当前无数据）。"; fi ;;
  esac
else
  wait_ "无法读取 /scan 的 frame_id，跳过雷达 TF 检查（当前无数据）。"
fi
if echo "$NODES" | grep -qE '^/(slam_gmapping|amcl)$'; then
  tf_echo map odom "地图→里程计（建图/定位输出）"
  case $? in
    2) fail "建图/定位节点在运行但 map 坐标系不存在。原因: gmapping/AMCL 尚未发布 map→odom（AMCL 需要先收到初始位姿）。下一步: 网页设置初始位置；查看后端日志中 amcl 输出。" ;;
  esac
else
  wait_ "map→odom TF 未检查：建图/定位未运行。说明: 此时 RViz 出现 “Fixed Frame [map] does not exist” 属等待数据，不是故障；雷达体检请用 diagnostics.rviz（Fixed Frame: odom）。"
fi
# map→odom 双发布者冲突：建图与定位不能同时在运行。
if [[ -n "${NODES// /}" ]]; then
  if echo "$NODES" | grep -q '^/slam_gmapping$' && echo "$NODES" | grep -qE '^/amcl$'; then
    fail "slam_gmapping 与 amcl 同时在运行：map→odom 会被双发布，TF 来回跳动、定位与地图互相拉扯。原因: 建图未干净退出就进入了导航。下一步: 网页停止导航，保存并停止建图确认 gmapping 退出后，再重新开始导航。"
  else
    ok "无 map→odom 双发布冲突（slam_gmapping 与 amcl 未同时运行）"
  fi
fi
# 时间源
if topic_exists /clock; then
  fail "发现 /clock 话题但项目约定使用真实时间（use_sim_time=false）。原因: 某节点在发布仿真时钟，将导致 TF 时间混乱。下一步: 找到发布 /clock 的节点并停止，或统一 use_sim_time。"
else
  ok "无 /clock 话题：与“二维仿真也用真实时间(use_sim_time=false)”的项目约定一致"
fi
if [[ -n "$LASER_FRAME" ]] && topic_exists /scan; then
  run 8 ros2 topic echo /scan --once --field header.stamp
  STAMP_SEC="$(echo "$OUT" | awk '/sec:/{print $2; exit}')"
  NOW_SEC="$(date +%s)"
  if [[ -n "${STAMP_SEC:-}" ]]; then
    DIFF=$(( STAMP_SEC - NOW_SEC )); ADIFF=${DIFF#-}
    if (( ADIFF <= 2 )); then ok "雷达时间戳与系统时钟差 ${DIFF}s（≤2s，TF 不会外推失败）"
    else fail "雷达时间戳与系统时钟差 ${DIFF}s。原因: 发布端时间源错误将导致 TF extrapolation。下一步: 检查发布端 get_clock()/系统 NTP；不要靠加大 TF 超时掩盖。"; fi
  else
    wait_ "无法读取雷达时间戳（当前无数据）。"
  fi
fi

# ---------- 8. 生命周期与 Action ----------
hdr "8. 生命周期与 Action"
run 20 ros2 lifecycle nodes
# bt_navigator 的内部子节点（*_rclcpp_node）不提供独立生命周期服务，排除以免误报。
LC_NODES="$(echo "$OUT" | grep -v '_rclcpp_node$')"
echo "$OUT" | grep -q '_rclcpp_node$' && info "已从生命周期检查中排除 bt_navigator 内部子节点（*_rclcpp_node）"
if [[ -n "${LC_NODES// /}" ]]; then
  INACTIVE=''
  for n in $LC_NODES; do
    run 8 ros2 lifecycle get "$n"
    state="$(echo "$OUT" | awk '{print $1}')"
    printf '         %-45s %s\n' "$n" "${state:-查询失败(exit=$RC)}"
    case "$state" in
      active) ;;
      '') INACTIVE="$INACTIVE $n(查询失败)" ;;
      *) INACTIVE="$INACTIVE $n(${state})" ;;
    esac
  done
  if [[ -n "$INACTIVE" ]]; then
    if [[ "$PHASE" == waiting_localization ]]; then
      wait_ "未激活节点:${INACTIVE}。说明: 控制台模式在“等待初始定位”阶段有意只激活 map_server 与 amcl，其余节点在网页设置初始位置后才激活，属正常流程。"
    else
      fail "存在未激活节点:${INACTIVE}。下一步: 对照后端 navigation_status 的 missing 列表；若长期停在 starting，查看后端日志中生命周期转换重试记录。"
    fi
  else
    ok "全部生命周期节点 active"
  fi
else
  wait_ "未发现生命周期节点（导航未启动或当前无数据）。"
fi
run 20 ros2 action list
if [[ "$RC" == 0 && -n "${OUT// /}" ]]; then
  if echo "$OUT" | grep -q '^/navigate_to_pose$'; then
    ok "/navigate_to_pose Action 在位"
    run 15 ros2 action info /navigate_to_pose
    echo "$OUT" | sed 's/^/         /'
  else
    if [[ "$READY" == True || "$READY" == true ]]; then
      fail "导航就绪判定通过但 /navigate_to_pose 不在 Action 列表。下一步: 检查 bt_navigator 是否退出过。"
    else wait_ "/navigate_to_pose 尚未出现（导航未启动，当前无数据）。"; fi
  fi
else
  wait_ "Action 列表为空或查询失败 (exit=$RC)（当前无数据）。"
fi

# ---------- 9. 进程卫生 ----------
hdr "9. 进程卫生（只报告，不杀进程）"
RVIZ_N="$(pgrep -xc rviz2 2>/dev/null)"; RVIZ_N="${RVIZ_N:-0}"
info "rviz2 实例数: ${RVIZ_N}$( (( RVIZ_N > 1 )) && echo '（多个 RViz 窗口同时打开，注意区分观察对象）' )"
GM_N="$(pgrep -fc slam_gmapping 2>/dev/null)"; GM_N="${GM_N:-0}"
if (( GM_N > 1 )); then
  fail "slam_gmapping 相关进程 ${GM_N} 个：重复建图会互相覆盖 /tf 与 /map。下一步: 停止多余的建图入口（网页只点一次“开始建图”；勿再手动 ros2 launch）。"
else
  info "slam_gmapping 相关进程数: ${GM_N}"
fi
NAV_PROC_N="$(pgrep -fc 'bt_navigator|controller_server|planner_server' 2>/dev/null)"; NAV_PROC_N="${NAV_PROC_N:-0}"
if [[ "$PHASE" == stopped && "${NAV_PROC_N:-0}" -gt 0 ]]; then
  fail "后端报告导航已停止，但仍有 ${NAV_PROC_N} 个 Nav2 进程残留。原因: 上一次导航未清理干净，可能与下次启动冲突。下一步: 记录 ps -eo pid,cmd | grep -E 'bt_navigator|controller_server' 输出后手动停止残留进程。"
else
  info "Nav2 相关进程数: ${NAV_PROC_N:-0}（phase=${PHASE:-未知}）"
fi

# ---------- 总结 ----------
hdr "总体结论"
if (( FAIL_N > 0 )); then
  echo "总体结论: 故障（$FAIL_N 项故障 / $WAIT_N 项等待数据 / $OK_N 项正常）"
  echo "下一步: 逐项处理上面标 [故障] 的条目；每条已附原因与建议。修复后重跑本脚本确认。"
  EXIT=1
elif (( WAIT_N > 0 )); then
  echo "总体结论: 等待数据（$WAIT_N 项等待 / $OK_N 项正常，无故障项）"
  echo "下一步: 按各 [等待数据] 条目提示启动对应服务或车端后重跑。"
  EXIT=0
else
  echo "总体结论: 正常（$OK_N 项检查全部通过）"
  EXIT=0
fi
echo "报告已保存: $LOG"
exit "$EXIT"
