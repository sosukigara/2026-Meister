#!/bin/bash

# ワークスペースのビルドとソース
colcon build --symlink-install --packages-select ros2_autonomous_nav
source install/setup.bash

echo "1. SLAMモードで起動します..."
echo "2. RVizが立ち上がったら、別のターミナルでキーボード操作を行ってください。"
echo "============================================"

# SLAMを起動
ros2 launch ros2_autonomous_nav slam_map.launch.py
