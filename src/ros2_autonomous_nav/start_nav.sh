#!/bin/bash

# ==========================================
# ros2-autonomous-nav 一括起動スクリプト
# ==========================================

# 1. 環境の読み込み
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source /opt/ros/jazzy/setup.bash

# Meistarワークスペースのルートへ移動
cd "$SCRIPT_DIR/.."
WS_ROOT=$(pwd)

# ビルドが必要な場合は実行
echo "ビルドを確認中..."
colcon build --packages-select ros2_autonomous_nav --symlink-install
source install/setup.bash

cd "$SCRIPT_DIR"

# ------------------------------------------
# クリーンアップ処理
# ------------------------------------------
cleanup() {
    echo ""
    echo "システムを終了しています..."
    pkill -f "robot_nav.launch.py"
    pkill -f "rviz2"
    pkill -f "gz sim"
    echo "完了。"
    exit 0
}

trap cleanup SIGINT

# ------------------------------------------
# 実行
# ------------------------------------------

echo "1. ロボットとGazebo、Nav2スタックを起動します..."
bash run.sh &
RUN_PID=$!

echo "2. RVizを起動するまで少し待ちます..."
sleep 10

echo "3. RVizを起動します..."
bash rviz.sh &
RVIZ_PID=$!

echo "------------------------------------------"
echo "起動完了！RVizで 2D Nav Goal を指定してください。"
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $RUN_PID
