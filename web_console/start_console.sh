#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../scripts/env.bash"
# This is the real-car entry; do not inherit an offline simulation domain.
export ROS_DOMAIN_ID="${ROBOT_REAL_DOMAIN_ID:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROBOT_SIMULATION=0
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
/usr/bin/python3 "$SCRIPT_DIR/../scripts/prepare_runtime.py"
command -v node >/dev/null
command -v npm >/dev/null
ros2 pkg prefix foxglove_bridge >/dev/null
cd "$SCRIPT_DIR"
if [[ ! -d node_modules ]]; then npm ci --ignore-scripts; fi
export ROBOT_API_PORT="${ROBOT_API_PORT:-3000}"
export VITE_ROBOT_API_PORT="$ROBOT_API_PORT"
export VITE_ROBOT_WS_PORT=8765
# 浏览器侧关键状态上报（写入 ROBOT_CONSOLE_LOG_DIR；未设置时后端直接忽略）。
export VITE_CLIENT_LOG=1
python3 - "$ROBOT_API_PORT" <<'PY'
import socket,sys
for port in [int(sys.argv[1]),5173,8765]:
    try:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('0.0.0.0',port))
    except OSError as error:
        raise SystemExit(f'Console port {port} is occupied; stop the existing console first: {error}')
PY
BACKEND_PID=''
FRONTEND_PID=''
cleanup() {
  trap - EXIT INT TERM
  node --input-type=module - "$SCRIPT_DIR/managedProcess.js" "$BACKEND_PID" "$FRONTEND_PID" <<'JS'
import { pathToFileURL } from 'node:url';
const { terminateProcess } = await import(pathToFileURL(process.argv[2]));
await Promise.all(process.argv.slice(3).filter(Boolean).map(pid => terminateProcess({ pid: Number(pid) })));
JS
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
node server.js &
BACKEND_PID=$!
ready=false
for attempt in {1..40}; do
  kill -0 "$BACKEND_PID" 2>/dev/null || { echo 'Backend failed to start' >&2; exit 1; }
  if curl -fsS "http://localhost:$ROBOT_API_PORT/api/runtime_config" >/dev/null; then ready=true; break; fi
  sleep 0.2
done
[[ "$ready" == true ]] || { echo 'Backend readiness timeout' >&2; exit 1; }
kill -0 "$BACKEND_PID" 2>/dev/null || { echo 'Backend exited during startup' >&2; exit 1; }
curl -fsS -X POST "http://localhost:$ROBOT_API_PORT/api/run_task/startup"
printf '\nConsole: http://localhost:5173/\n'
npm run dev -- --port 5173 --strictPort &
FRONTEND_PID=$!
wait -n "$BACKEND_PID" "$FRONTEND_PID"
