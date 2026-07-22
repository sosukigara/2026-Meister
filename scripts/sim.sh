#!/bin/bash
# sim.sh — One-command launch of Meister SLAM + navigation simulation
#
# Usage:
#   ./scripts/sim.sh [world_name=maze]
#
# Description:
#   Launches the full simulation stack via meister_slam_nav.launch.py.
#   Optional first argument sets the Gazebo world (default: maze).
#   Exploration mode is enabled.

set -euo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
source install/setup.bash

ros2 launch meister_robot meister_slam_nav.launch.py \
  world_name:=${1:-maze} explore:=true
