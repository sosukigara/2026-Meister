#!/bin/bash
source /opt/ros/jazzy/setup.bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/my_new_map
echo "地図を maps/my_new_map に保存しました。"
