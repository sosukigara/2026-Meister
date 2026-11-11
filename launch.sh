#!/bin/bash
# ==========================================
# Meistar ROS 2 起動スクリプト
# 使い方:
#   ./launch.sh                     # 実機モード (Gazeboなし。LD-D500 + ESP32が必要)
#   ./launch.sh --sim [warehouse|maze|empty]  # シミュレーションモード (Gazeboあり)
# 事前に ./build.sh を実行しておくこと
# ==========================================
set -eo pipefail

# Mode selection: default real robot, --sim for Gazebo simulation
SIM=0
if [ "${1:-}" = "--sim" ]; then
    SIM=1
    shift
fi

if [ "$SIM" -eq 1 ]; then
    # World selection: positional arg, default warehouse
    WORLD="${1:-warehouse}"
    case "$WORLD" in
        warehouse|maze|empty) ;;
        *)
            echo "Usage: $0 --sim [warehouse|maze|empty]"
            exit 1
            ;;
    esac
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WORKSPACE_ROOT=$(cd "$SCRIPT_DIR" && pwd)

cd "$WORKSPACE_ROOT"

if [ ! -f "ros/install/setup.bash" ]; then
    echo "ERROR: ros/install/setup.bash が見つかりません。先に ./build.sh を実行してください。" >&2
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
source ros/install/setup.bash

# Force Gazebo transport over TCP localhost
# Default UDP multicast discovery fails on machines with VPN (tun0)
export GZ_IP=127.0.0.1

# Force ROS 2 DDS to localhost only (same reason: VPN breaks multicast)
export ROS_LOCALHOST_ONLY=1

# 実験用の固定ドメイン (他の ROS プロセスとの競合防止)。環境変数で上書き可能。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"

# Force NVIDIA EGL ICD for gz sim rendering.
# Without this, gz's render engine (required by the lidar sensor) falls back
# to Mesa EGL on this host, fails ("driver (null)" / dri2 error), and the
# lidar silently publishes no /scan data -> SLAM/Nav2 never start.
# NVIDIA がない環境では設定しない (存在しない ICD を指すと描画が壊れる)。
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

# 前回の起動が残っていると gz / Web UI(8088) / rviz が二重起動で競合するため、
# 先に全ての ROS 関連プロセスを終了してから起動する
echo "=== 既存の ROS プロセスを終了します ==="
"$WORKSPACE_ROOT/kill.sh" || true

cleanup() {
    echo ""
    echo "システムを終了しています..."
    pkill -P $$
    pkill -f "mapping_nav.launch.py"
    pkill -f "rviz2"
    pkill -f "gz sim"
    # vision残留で次回起動の画像トピックが二重配信になるため確実に落とす
    pkill -f "detection_node"
    pkill -f "pc_camera"
    echo "完了。"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Serial devices (real mode only; sim ignores them).
# ESP32 ブリッジと LD-D500 で被らないこと。by-id/by-path 推奨。
export MEISTAR_SERIAL_PORT="${MEISTAR_SERIAL_PORT:-/dev/ttyUSB0}"
export MEISTAR_LIDAR_PORT="${MEISTAR_LIDAR_PORT:-/dev/ttyUSB1}"

echo "=== Gazebo + SLAM + Nav2 を起動します ==="
if [ "$SIM" -eq 1 ]; then
    echo "=== Mode: simulation (Gazebo) / World: $WORLD ==="
    export MEISTAR_USE_SIM_TIME=true
    ros2 launch ros2_autonomous_nav mapping_nav.launch.py "world:=$WORLD" &
else
    echo "=== Mode: real robot (LD-D500: $MEISTAR_LIDAR_PORT / ESP32: $MEISTAR_SERIAL_PORT) ==="
    export MEISTAR_USE_SIM_TIME=false
    # 320 モデルがあれば 2 スレッドで 10Hz (実測)。なければ 640 既定＋警告。
    VISION_MODEL=""
    VISION_THREADS="4"
    if [ -f "$WORKSPACE_ROOT/ros/install/meister_vision/share/meister_vision/models/yolov8n-320.onnx" ]; then
        VISION_MODEL="$WORKSPACE_ROOT/ros/install/meister_vision/share/meister_vision/models/yolov8n-320.onnx"
        VISION_THREADS="2"
        echo "=== Vision: yolov8n-320 + 2 threads (10Hz eco) ==="
    else
        echo "=== Vision: yolov8n-640 既定 (320 が無いため。5Hz 級) ==="
    fi
    ros2 launch ros2_autonomous_nav real_robot.launch.py \
        "serial_port:=$MEISTAR_SERIAL_PORT" "lidar_port:=$MEISTAR_LIDAR_PORT" \
        "vision_model:=$VISION_MODEL" "vision_threads:=$VISION_THREADS" &
fi
LAUNCH_PID=$!

if [ "$SIM" -eq 1 ]; then
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
else
    echo "=== 実機センサ (LD-D500 /scan) の準備ができるまで待機中… (最大30秒) ==="
    for i in $(seq 1 30); do
      if timeout 3 ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
        echo "   /scan 検出 ($i 秒)"
        break
      fi
      if [ "$i" -eq 30 ]; then
        echo "   警告: /scan が見つかりません。LD-D500 ($MEISTAR_LIDAR_PORT) の接続・権限(dialout)を確認してください。"
      fi
      sleep 1
    done
fi

echo "=== RViz を起動します ==="
"$WORKSPACE_ROOT/ros/src/ros2_autonomous_nav/rviz.sh" &

echo "------------------------------------------"
echo "起動完了！"
echo "- 手動操作: 別ターミナルで teleop を実行。"
echo "- 自動移動: RVizで '2D Nav Goal' を指定。"
echo "- Web UI: http://localhost:8088 で地図上に複数地点を指定して移動できます。"
echo "- 画像認識: /detection_imageをRVizで確認"
if [ "$SIM" -eq 1 ]; then
    echo "- ロボット目線カメラ: Gazebo映像を使用 (PCカメラはuse_pc_camera:=trueで切替可)"
else
    echo "- ロボット目線カメラ: PCカメラを使用 (/dev/video0)"
fi
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $LAUNCH_PID
