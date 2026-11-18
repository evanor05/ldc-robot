#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != lyrical ]]; then
  echo "Use a fresh shell; expected ROS_DISTRO=lyrical" >&2
  exit 1
fi
source /opt/ros/lyrical/setup.bash
cd "$root/robot_ws"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
# CMake otherwise finds the user's uv Python 3.12, which cannot import ROS 3.14 modules.
exec /usr/bin/colcon build --base-paths src --parallel-workers 2 \
  --event-handlers console_cohesion+ --cmake-args \
  -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
