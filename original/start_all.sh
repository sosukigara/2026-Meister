#!/bin/bash

# ==========================================
# ROS 2 Jazzy Simulation All-in-One Starter
# ==========================================

# 1. 環境の読み込み
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source /opt/ros/jazzy/setup.bash
cd "$SCRIPT_DIR"

# ビルドが必要な場合は実行（初回や変更時）
echo "Building workspace at $SCRIPT_DIR..."
colcon build --symlink-install
source install/setup.bash

# Gazeboリソースパスの設定
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/install/meistar_description/share

# ------------------------------------------
# クリーンアップ処理の定義
# ------------------------------------------
cleanup() {
    echo ""
    echo "Stopping all ROS 2 systems..."
    # バックグラウンドで動いているLaunchプロセスを終了
    if [ ! -z "$LAUNCH_PID" ]; then
        kill -INT $LAUNCH_PID
    fi
    # 別窓で開いたテレオペ用プロセスを終了
    pkill -f teleop_twist_keyboard
    # プロセスの完全な掃除
    ./kill_all.sh
    echo "Cleanup complete."
    exit 0
}

# Ctrl+C (SIGINT) を受け取ったら cleanup を実行
trap cleanup SIGINT

# ------------------------------------------
# 各システムの起動
# ------------------------------------------
echo "Starting Integrated Systems (Gazebo, SLAM, Nav2)..."
# 統合Launchをバックグラウンドで実行
ros2 launch meistar_description all_system.launch.py &
LAUNCH_PID=$!

# 少し待ってからテレオペ用ウィンドウを表示
sleep 5
echo "Opening Teleop window..."
gnome-terminal --title="Teleop Keyboard" -- bash -c "source /opt/ros/jazzy/setup.bash; source $SCRIPT_DIR/install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard"

echo "------------------------------------------"
echo "Running. Press Ctrl+C to stop everything."
echo "------------------------------------------"

# Launchプロセスの終了を待機（Ctrl+Cでトラップされるまで）
wait $LAUNCH_PID
