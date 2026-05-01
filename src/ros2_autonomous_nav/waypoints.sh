#!/bin/bash
# Run the waypoint follower node — sends the robot on a maze tour
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 $(dirname "$0")/scripts/waypoint_follower.py
