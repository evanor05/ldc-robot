# Source this file from bash. Does not modify the login shell configuration.
_ROBOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != lyrical ]]; then
  echo "Use a fresh shell: ROS_DISTRO is $ROS_DISTRO, expected lyrical" >&2
  return 1
fi
source /opt/ros/lyrical/setup.bash || return
# The XRCE library is deliberately installed inside the repository.
if [[ -d "$_ROBOT_ROOT/agent_ws/.deps/install/lib" ]]; then
  export LD_LIBRARY_PATH="$_ROBOT_ROOT/agent_ws/.deps/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [[ -f "$_ROBOT_ROOT/agent_ws/install/local_setup.bash" ]]; then
  source "$_ROBOT_ROOT/agent_ws/install/local_setup.bash" || return
fi
export ROBOT_WORKSPACE="$_ROBOT_ROOT/robot_ws"
export ROBOT_WORKSPACE_SETUP="$ROBOT_WORKSPACE/install/local_setup.bash"
if [[ -f "$ROBOT_WORKSPACE_SETUP" ]]; then
  source "$ROBOT_WORKSPACE_SETUP" || return
fi
unset _ROBOT_ROOT
