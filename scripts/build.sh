#!/bin/bash
# build.sh — Colcon build wrapper for Meister project
#
# Usage:
#   ./scripts/build.sh [colcon-options...]
#
# Description:
#   Sources the ROS 2 Jazzy environment, sets ROS_DOMAIN_ID to prevent
#   topic cross-talk (default 42), runs colcon build with symlink-install,
#   then sources the resulting install setup. Passes any extra arguments
#   through to colcon (e.g. --packages-select).

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

set -euo pipefail

colcon build --symlink-install "$@"

# Source workspace setup (nounset disabled: colcon scripts reference unset $COLCON_TRACE)
set +u
source install/setup.bash
set -u

echo "Build complete. Install space sourced."
