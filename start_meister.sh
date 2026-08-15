#!/bin/bash
# ==========================================
# Meistar ROS 2 起動スクリプト
# 使い方: ./start_meister.sh [warehouse|maze|empty]
# 事前に ./build.sh を実行しておくこと
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

if [ ! -f "install/setup.bash" ]; then
    echo "ERROR: install/setup.bash が見つかりません。先に ./build.sh を実行してください。" >&2
    exit 1
fi

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

# Force NVIDIA EGL ICD for gz sim rendering.
# Without this, gz's render engine (required by the lidar sensor) falls back
# to Mesa EGL on this host, fails ("driver (null)" / dri2 error), and the
# lidar silently publishes no /scan data -> SLAM/Nav2 never start.
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# 前回の起動が残っていると gz / Web UI(8088) / rviz が二重起動で競合するため、
# 先に全ての ROS 関連プロセスを終了してから起動する
echo "=== 既存の ROS プロセスを終了します ==="
"$WORKSPACE_ROOT/kill_ros.sh" || true

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
echo "- Web UI: http://localhost:8088 で地図上に複数地点を指定して移動できます。"
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $LAUNCH_PID
