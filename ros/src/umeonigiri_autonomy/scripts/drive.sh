#!/bin/bash
# Drive the robot manually with keyboard (useful during SLAM mapping)
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.3 -p turn:=0.8
