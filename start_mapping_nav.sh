#!/bin/bash

# ==========================================
# 地図生成・自動移動一括起動スクリプト
# ==========================================

# 1. ビルドと環境の読み込み
echo "ビルドを確認中..."
colcon build --symlink-install --packages-select ros2_autonomous_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# ------------------------------------------
# クリーンアップ処理
# ------------------------------------------
cleanup() {
    echo ""
    echo "システムを終了しています..."
    pkill -f "mapping_nav.launch.py"
    pkill -f "rviz2"
    pkill -f "gz sim"
    echo "完了。"
    exit 0
}

trap cleanup SIGINT

# ------------------------------------------
# 実行
# ------------------------------------------

echo "1. Gazebo + SLAM + Nav2 を一括起動します..."
ros2 launch ros2_autonomous_nav mapping_nav.launch.py &
LAUNCH_PID=$!

echo "2. RVizを起動するまで少し待ちます..."
sleep 15

echo "3. RVizを起動します..."
./ros2-autonomous-nav/rviz.sh &

echo "------------------------------------------"
echo "起動完了！"
echo "- 手動操作: 別ターミナルで teleop を実行してください。"
echo "- 自動移動: RVizで '2D Nav Goal' を指定すると、現在の地図を元に移動します。"
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $LAUNCH_PID
