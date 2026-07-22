#!/bin/bash
# setup.sh — Install all ROS 2 Jazzy dependencies for Meister project
#
# Usage:
#   ./scripts/setup.sh
#
# Description:
#   Installs required ros-jazzy packages via apt and resolves source
#   dependencies via rosdep. Run once after cloning the workspace.

source /opt/ros/jazzy/setup.bash

set -euo pipefail

sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-smac-planner

# Automatic dependency resolution for workspace packages
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y

echo "Dependencies installed successfully."
