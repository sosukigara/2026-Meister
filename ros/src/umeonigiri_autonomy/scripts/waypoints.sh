#!/bin/bash
# Run the waypoint follower node — sends the robot on a maze tour
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash
ros2 run umeonigiri_autonomy waypoint_follower
