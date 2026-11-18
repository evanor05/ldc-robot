#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != lyrical ]]; then
  echo "Use a fresh shell; expected ROS_DISTRO=lyrical" >&2
  exit 1
fi
source /opt/ros/lyrical/setup.bash
/usr/bin/python3 "$root/scripts/fetch_agent_sources.py"
ws="$root/agent_ws"
vendor="$ws/.deps/install"
cmake -S "$ws/.deps/src/spdlog" -B "$ws/.deps/build/spdlog" \
  -DCMAKE_INSTALL_PREFIX="$vendor" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DSPDLOG_BUILD_SHARED=OFF \
  -DSPDLOG_FMT_EXTERNAL=OFF -DSPDLOG_BUILD_EXAMPLE=OFF
cmake --build "$ws/.deps/build/spdlog" --parallel 2
cmake --install "$ws/.deps/build/spdlog"
cmake -S "$ws/.deps/src/xrce" -B "$ws/.deps/build/xrce" \
  -DCMAKE_INSTALL_PREFIX="$vendor" -DCMAKE_PREFIX_PATH="$vendor;/opt/ros/lyrical" \
  -DCMAKE_BUILD_TYPE=Release -DUAGENT_SUPERBUILD=OFF \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_USE_SYSTEM_LOGGER=ON -DUAGENT_CED_PROFILE=OFF \
  -DUAGENT_P2P_PROFILE=OFF -DUAGENT_BUILD_EXECUTABLE=OFF \
  -DUAGENT_ISOLATED_INSTALL=OFF
cmake --build "$ws/.deps/build/xrce" --parallel 2
cmake --install "$ws/.deps/build/xrce"
export CMAKE_PREFIX_PATH="$vendor${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$vendor/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
cd "$ws"
exec /usr/bin/colcon build --base-paths lyrical-src --parallel-workers 2 \
  --event-handlers console_cohesion+ --cmake-args -DBUILD_TESTING=OFF \
  -DMICROROSAGENT_SUPERBUILD=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_RPATH="$vendor/lib" -DCMAKE_INSTALL_RPATH="$vendor/lib"
