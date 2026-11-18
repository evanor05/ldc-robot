#!/usr/bin/env bash
# Local-only software checks. All ROS participants use an isolated test domain.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/env.bash"
export ROS_DOMAIN_ID=197 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd "$root"
/usr/bin/python3 migration-tools/tests/mirror_source_logs.py
/usr/bin/python3 migration-tools/tests/tf_smoke.py
/usr/bin/python3 migration-tools/tests/serial_smoke.py
/usr/bin/python3 migration-tools/tests/qt_smoke.py
/usr/bin/python3 migration-tools/tests/xrce_ros_smoke.py
/usr/bin/python3 migration-tools/tests/gmapping_smoke.py
/usr/bin/python3 migration-tools/tests/nav2_smoke.py
node --test web_console/tests/battery.test.js
node web_console/tests/backend.integration.js
node web_console/tests/foxglove.integration.js
