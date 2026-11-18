#!/bin/bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$(dirname "$0")/../maps"
ros2 run nav2_map_server map_saver_cli -f "$(dirname "$0")/../maps/my_new_map"
echo "地図を $(dirname "$0")/../maps/my_new_map に保存しました。"
