#!/usr/bin/env bash
# Closed-loop navigation verification in a synthetic world. No vehicle, no network.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/env.bash"
export ROS_DOMAIN_ID=197 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd "$root"
exec /usr/bin/python3 migration-tools/tests/nav2_closed_loop.py "$@"
