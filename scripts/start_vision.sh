#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:?用法: start_vision.sh --real|--sim [ROS launch 参数]}";shift
source "$ROOT/scripts/env.bash"
case "$MODE" in
  --sim) export ROS_DOMAIN_ID=197 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    export ROS_LOCALHOST_ONLY=1
    # An offline launch must never auto-discover a real camera.
    python3 - "$@" <<'PY'
import sys
from urllib.parse import urlsplit
url=next((a[5:] for a in reversed(sys.argv[1:]) if a.startswith('url:=')),'')
assert urlsplit(url).hostname in ('127.0.0.1','localhost','::1'),'仿真必须显式指定 loopback 摄像头 URL'
PY
    port=18321;;
  --real) export ROS_DOMAIN_ID="${ROBOT_REAL_DOMAIN_ID:-0}" ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS;port=18320;;
  *) echo '必须选择 --real 或 --sim' >&2;exit 2;;
esac
# flock is held by the launch process and prevents concurrent camera readers.
mkdir -p "${XDG_RUNTIME_DIR:-/tmp}/ldc-vision-$UID"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/ldc-vision-$UID/$port.lock"
if ! flock -n 9; then
  python3 - "$port" <<'PYTHON'
import json,sys,urllib.request
try:
    with urllib.request.urlopen('http://127.0.0.1:'+sys.argv[1]+'/status',timeout=2) as r:s=json.load(r)
    assert s.get('service')=='ldc_vision'
    print(json.dumps(s,ensure_ascii=False))
except Exception:
    print('视觉已在启动或端口不可用，请稍后查询 /api/vision/status',file=sys.stderr)
    sys.exit(1)
PYTHON
  exit $?
fi
for arg in "$@"; do
  if [[ "$arg" == gateway_port:=* ]]; then echo '网关端口由模式决定' >&2; exit 2; fi
done
exec ros2 launch ldc_vision vision.launch.py gateway_port:="$port" "$@"
