#!/bin/bash
# Open RViz with the Nav2 config (map + costmaps + paths + AMCL particles)
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source $(dirname "$0")/../../install/setup.bash
rviz2 -d $(dirname "$0")/config/nav2_rviz.rviz --ros-args -p use_sim_time:=true
