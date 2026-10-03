#!/usr/bin/env bash
# The in-container half of .github/workflows/ci.yml: the three commands from CLAUDE.md, run against
# a colcon workspace at /workspace with this repo at /workspace/src/syncai_lio_bridge.
#
# It runs under the image's entrypoint, which has already sourced ROS 2 and the ros2_rust underlay.
# It can also be run by hand inside the Dev Container to reproduce CI exactly:
#
#   bash /workspace/src/syncai_lio_bridge/.github/scripts/ci.sh
set -euo pipefail

cd /workspace

# --base-paths src so colcon only ever crawls the source tree: colcon-cargo treats every
# Cargo.toml as a package, so anything else that lands under /workspace (a cargo registry, a
# target directory without COLCON_IGNORE) would otherwise be picked up too. console_cohesion
# prints each package's full output once it ends, so a compiler error is readable in the log
# instead of interleaved.
colcon build --base-paths src --symlink-install --event-handlers console_cohesion+

# The overlay is what `ros-env` reads when cargo runs outside colcon. The ROS setup scripts
# reference unset variables, so -u is lifted around them.
set +u
# shellcheck disable=SC1091
source /workspace/install/setup.bash --
set -u

cd src/syncai_lio_bridge

# -D warnings reaches only this crate: cargo passes the trailing arguments to the primary package
# alone, so the warnings rclrs emits from /opt/ros2_rust_underlay (not ours) do not fail the run.
cargo clippy --target-dir /workspace/build/.clippy --all-targets -- -D warnings

cargo test --target-dir /workspace/build/.clippy
