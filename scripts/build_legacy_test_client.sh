#!/usr/bin/env bash
# Native test libraries only; this neither builds nor flashes ESP32 firmware.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$root/firmware/chassis/managed_components/micro_ros_espidf_component/micro_ros_src/src"
prefix="$root/agent_ws/.deps/legacy-install"
cmake -S "$source_dir/micro-CDR" -B "$root/agent_ws/.deps/build/microcdr" \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$root/agent_ws/.deps/build/microcdr" --parallel 2
cmake --install "$root/agent_ws/.deps/build/microcdr"
cmake -S "$source_dir/Micro-XRCE-DDS-Client" -B "$root/agent_ws/.deps/build/legacy-client" \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_PREFIX_PATH="$prefix" \
  -DUCLIENT_SUPERBUILD=OFF -DUCLIENT_BUILD_EXAMPLES=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$root/agent_ws/.deps/build/legacy-client" --parallel 2
cmake --install "$root/agent_ws/.deps/build/legacy-client"
