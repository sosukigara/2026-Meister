#!/bin/bash
# ==========================================
# Meister ROS 2 起動スクリプト
# 使い方: ./start_meister.sh
# ==========================================

WORKSPACE=/home/so/Meister

cd "$WORKSPACE"
echo "=== ビルド中… ==="
colcon build --symlink-install --packages-select ros2_autonomous_nav meistar_description

source /opt/ros/jazzy/setup.bash
source install/setup.bash

cleanup() {
    echo ""
    echo "システムを終了しています..."
    pkill -P $$
    pkill -f "mapping_nav.launch.py"
    pkill -f "rviz2"
    pkill -f "gz sim"
    echo "完了。"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "=== Gazebo + SLAM + Nav2 を起動します ==="
ros2 launch ros2_autonomous_nav mapping_nav.launch.py &
LAUNCH_PID=$!

echo "=== RViz を起動します ==="
sleep 15
"$WORKSPACE/src/ros2_autonomous_nav/rviz.sh" &

echo "------------------------------------------"
echo "起動完了！"
echo "- 手動操作: 別ターミナルで teleop を実行。"
echo "- 自動移動: RVizで '2D Nav Goal' を指定。"
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $LAUNCH_PID
