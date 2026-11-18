#!/usr/bin/env bash
# 实车控制台「带日志」入口：每次运行自动建立独立日志目录，
# 会话结束后所有原始数据都留在 logs/runs/<时间>-real/ 下，供离线分析。
#
# 用法：
#   bash scripts/run_real_logged.sh              # 等同 start_console.sh
#   ROBOT_CAR_IP=10.147.73.166 bash scripts/run_real_logged.sh   # 显式指定车端 IP
#   ROBOT_LOG_ROOT=/path bash scripts/run_real_logged.sh         # 改日志根目录
#
# 产物：
#   console.log              所有子进程（agent/launch/foxglove/后端/前端）的标准输出
#   bridge-events.jsonl      后端事件（锁定/急停/断流/租约…）
#   bridge-samples.jsonl     后端每秒状态采样（雷达/里程计年龄、到达间隔、模式）
#   net.jsonl                每 5 秒：无线信号、收发计数、ping 车端、后端状态
#   frontend-events.jsonl    浏览器侧关键状态变化（需前端上报）
#   manifest.json            运行环境与启动参数
#   summary.md               diagnose_run.py 生成的分析报告（退出时自动生成）
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_ROOT="${ROBOT_LOG_ROOT:-$ROOT/logs/runs}"
RUN_DIR="$LOG_ROOT/$(date +%Y%m%d-%H%M%S)-real"
mkdir -p "$RUN_DIR"
export ROBOT_CONSOLE_LOG_DIR="$RUN_DIR"
export ROBOT_RUN_DIR="$RUN_DIR"

cat > "$RUN_DIR/manifest.json" <<JSON
{
  "startedAt": "$(date -Iseconds)",
  "kind": "real",
  "repo": "$ROOT",
  "gitBranch": "$(git -C "$ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)",
  "gitCommit": "$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)",
  "rosDistro": "${ROS_DISTRO:-lyrical}",
  "rosDomainId": "${ROBOT_REAL_DOMAIN_ID:-0}",
  "carIp": "${ROBOT_CAR_IP:-auto}",
  "host": "$(hostname)",
  "kernel": "$(uname -r)"
}
JSON

python3 "$ROOT/scripts/log_sampler.py" --dir "$RUN_DIR" \
  --api "http://127.0.0.1:${ROBOT_API_PORT:-3000}/api" \
  --interval "${ROBOT_LOG_SAMPLE_SECONDS:-5}" \
  > "$RUN_DIR/sampler.log" 2>&1 &
SAMPLER_PID=$!

finish() {
  trap - EXIT INT TERM
  kill "$SAMPLER_PID" 2>/dev/null || true
  echo "日志目录: $RUN_DIR" >&2
  if command -v python3 >/dev/null; then
    python3 "$ROOT/scripts/diagnose_run.py" "$RUN_DIR" >/dev/null 2>&1 || true
    [ -f "$RUN_DIR/summary.md" ] && echo "分析报告: $RUN_DIR/summary.md" >&2
  fi
}
trap finish EXIT INT TERM

# 控制台自身输出全部落盘（同时保留终端实时显示）。
bash "$ROOT/web_console/start_console.sh" "$@" 2>&1 | tee -a "$RUN_DIR/console.log"
