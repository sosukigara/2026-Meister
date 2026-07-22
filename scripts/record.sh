#!/bin/bash
# record.sh — Record ROS 2 bag with all key topics
#
# Usage:
#   ./scripts/record.sh
#
# Description:
#   Records a ros2 bag from all Meister project topics into a
#   timestamped directory under the current working directory.
#
# Topics recorded:
#   /scan /tf /tf_static /odom /map /cmd_vel /goal_pose /clicked_point

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

set -euo pipefail

BAG_DIR="bag_$(date +%Y%m%d_%H%M%S)"

ros2 bag record -o "$BAG_DIR" \
  /scan /tf /tf_static /odom /map /cmd_vel /goal_pose /clicked_point

echo "Bag recorded to: $BAG_DIR"
