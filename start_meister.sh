#!/bin/bash
# ==========================================
# Meistar ROS 2 起動スクリプト
# 使い方: ./start_meister.sh
# ==========================================
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WORKSPACE_ROOT=$(cd "$SCRIPT_DIR" && pwd)

cd "$WORKSPACE_ROOT"

echo "=== ビルド中… ==="
colcon build --symlink-install --packages-select ros2_autonomous_nav meistar_description

ROS_SETUP=""
for _dir in /opt/ros/*/; do
  if [ -f "${_dir}setup.bash" ]; then
    ROS_SETUP="${_dir}setup.bash"
    break
  fi
done
if [ -z "$ROS_SETUP" ]; then
  echo "ERROR: ROS 2 setup.bash not found under /opt/ros/" >&2
  exit 1
fi
source "$ROS_SETUP"
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

echo "=== Gazebo の準備ができるまで待機中… ==="
# Dynamic readiness check: wait for /clock (Gazebo publishing sim time)
# instead of a fixed sleep. Times out after 30s to avoid hanging.
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q '^/clock$'; then
    echo "   Gazebo 準備完了 ($i 秒)"
    break
  fi
  sleep 1
done

echo "=== RViz を起動します ==="
"$WORKSPACE_ROOT/src/ros2_autonomous_nav/rviz.sh" &

echo "------------------------------------------"
echo "起動完了！"
echo "- 手動操作: 別ターミナルで teleop を実行。"
echo "- 自動移動: RVizで '2D Nav Goal' を指定。"
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $LAUNCH_PID
