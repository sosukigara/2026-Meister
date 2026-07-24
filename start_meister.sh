#!/bin/bash
# ==========================================
# Meistar ROS 2 起動スクリプト
# 使い方: ./start_meister.sh
# ==========================================
set -eo pipefail

# World selection: positional arg, default warehouse
WORLD="${1:-warehouse}"
case "$WORLD" in
    warehouse|maze|empty) ;;
    *)
        echo "Usage: $0 [warehouse|maze|empty]"
        exit 1
        ;;
esac

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

# Force Gazebo transport over TCP localhost
# Default UDP multicast discovery fails on machines with VPN (tun0)
export GZ_IP=127.0.0.1

# Force ROS 2 DDS to localhost only (same reason: VPN breaks multicast)
export ROS_LOCALHOST_ONLY=1

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
echo "=== World: $WORLD ==="
ros2 launch ros2_autonomous_nav mapping_nav.launch.py "world:=$WORLD" &
LAUNCH_PID=$!

echo "=== Gazebo の準備ができるまで待機中… ==="
echo "   (最大90秒待機。初回はシェーダコンパイルに時間がかかります)"
for i in $(seq 1 90); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -q '^/clock$'; then
    echo "   Gazebo 準備完了 ($i 秒)"
    break
  fi
  if [ $((i % 10)) -eq 0 ]; then
    echo "   待機中… ${i}秒経過"
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
