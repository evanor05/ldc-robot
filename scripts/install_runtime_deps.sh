#!/usr/bin/env bash
# Requires the official ROS 2 apt source for Ubuntu resolute to be configured.
set -eo pipefail
source /etc/os-release
[[ "$VERSION_CODENAME" == resolute && "$(dpkg --print-architecture)" == amd64 ]] || {
  echo 'This dependency set was verified on Ubuntu 26.04 amd64.' >&2; exit 1;
}
packages=(
  build-essential cmake ninja-build qtbase5-dev qtbase5-dev-tools curl
  libasio-dev libtinyxml2-dev python3-dev python3-opencv python3-numpy
  python3-colcon-common-extensions python3-rosdep python3-vcstool
  ros-lyrical-ros-base ros-lyrical-ament-cmake ros-lyrical-tf2-ros
  ros-lyrical-tf2-geometry-msgs ros-lyrical-message-filters
  ros-lyrical-rosidl-default-generators ros-lyrical-visualization-msgs
  ros-lyrical-vision-msgs ros-lyrical-diagnostic-msgs python3-yaml
  ros-lyrical-ament-lint-auto ros-lyrical-ament-lint-common
  ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher
  ros-lyrical-nav2-amcl ros-lyrical-nav2-bt-navigator ros-lyrical-nav2-controller
  ros-lyrical-nav2-dwb-controller ros-lyrical-nav2-navfn-planner
  ros-lyrical-nav2-smoother ros-lyrical-nav2-behaviors
  ros-lyrical-nav2-waypoint-follower ros-lyrical-nav2-lifecycle-manager
  ros-lyrical-nav2-map-server ros-lyrical-nav2-planner
  ros-lyrical-nav2-velocity-smoother ros-lyrical-nav2-common
  ros-lyrical-foxglove-bridge
)
if [[ "${1:-}" == --simulate ]]; then
  exec apt-get -s install --no-install-recommends "${packages[@]}"
fi
exec sudo apt-get install -y --no-install-recommends "${packages[@]}"
