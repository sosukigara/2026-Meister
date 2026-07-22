#!/bin/bash
# Launch robot + Gazebo + Nav2 (auto-detects GPU/CPU in launch file)
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash

echo "============================================"
if nvidia-smi &>/dev/null; then
    echo "  GPU DETECTED — running in GPU mode"
    echo "  (360 lidar samples, 10 Hz, ogre2)"
else
    echo "  NO GPU — running in CPU mode"
    echo "  (180 lidar samples, 5 Hz, software rendering)"
fi
echo "============================================"

ros2 launch $(dirname "$0")/launch/robot_nav.launch.py
