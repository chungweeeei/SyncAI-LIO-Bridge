# Source in order: ROS 2 itself -> Rust underlay -> user workspace
# This file is sourced by both the entrypoint and interactive shells, so do not use `set -e`.

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash" --
fi

if [ -f "${ROS2_RUST_UNDERLAY}/install/setup.bash" ]; then
  source "${ROS2_RUST_UNDERLAY}/install/setup.bash" --
fi

# The user workspace does not exist until it has been built once; that is expected
if [ -f "/workspace/install/setup.bash" ]; then
  source "/workspace/install/setup.bash" --
fi
