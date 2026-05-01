#!/bin/bash
# Send a navigation goal via the command line
# Usage: bash navigate.sh [x] [y] [yaw]
#   e.g. bash navigate.sh 3.5 3.5 1.57
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash

X=${1:-3.5}
Y=${2:-3.5}
YAW=${3:-0.0}

# Convert yaw to quaternion z,w
QZ=$(python3 -c "import math; print(math.sin($YAW/2))")
QW=$(python3 -c "import math; print(math.cos($YAW/2))")

echo "Sending navigation goal: x=$X, y=$Y, yaw=$YAW"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}}}"
