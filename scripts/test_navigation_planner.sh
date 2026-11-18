#!/usr/bin/env bash
# Offline tests only: no robot drivers, controller, or velocity publisher.
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$TASK_ROOT/scripts/build_host.sh" --packages-select ldc_navigation --cmake-args -DBUILD_TESTING=ON
source "$TASK_ROOT/scripts/env.bash"
export ROS_DOMAIN_ID=194 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS2_EASY_MODE
ctest --test-dir "$TASK_ROOT/robot_ws/build/ldc_navigation" --output-on-failure

# Read-only map quality gate; does not start ROS nodes or the planner.
/usr/bin/python3 "$TASK_ROOT/robot_ws/src/ldc_navigation/test/planning_offline_check.py" --quality-only
