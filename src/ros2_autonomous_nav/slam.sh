#!/bin/bash
# Launch robot + Gazebo + SLAM (to generate a map before Nav2)
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash

echo "============================================"
echo "  SLAM MODE — drive around to build a map"
echo "  Then run: bash save_map.sh nav_world"
if nvidia-smi &>/dev/null; then
    echo "  GPU DETECTED — running in GPU mode"
else
    echo "  NO GPU — running in CPU mode"
fi
echo "============================================"

ros2 launch $(dirname "$0")/launch/slam_map.launch.py
