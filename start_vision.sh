#!/bin/bash
# 画像認識だけを起動する。Gazebo / SLAM / Nav2 / Web UI は起動しない。
# start_meister.sh と同じ pc_vision.launch.py 経路を使うため、ここに出る検出結果は
# 本運と同一ノード・同一パラメータのものになる (カメラだけが PC 内蔵に変わる)。
#
# 使い方: ./start_vision.sh
# 上書きは環境変数で:
#   DEVICE=/dev/video1 RATE=10.0 THREADS=4 CONF=0.3 ./start_vision.sh
set -eo pipefail

DEVICE="${DEVICE:-/dev/video0}"
RATE="${RATE:-3.0}"
THREADS="${THREADS:-2}"
CONF="${CONF:-0.25}"

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SCRIPT_DIR"

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

# kill_ros.sh と同じ規則で、ROS_DOMAIN_ID が一致する PID だけを消す。
# 別ドメイン (robocon 等) のスタックを巻き込まないため。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export ROS_LOCALHOST_ONLY=1

VISION_PROCS=(detection_node pc_camera rqt_image_view)

kill_vision() {
  for name in "${VISION_PROCS[@]}"; do
    for pid in $(pgrep -f "$name" 2>/dev/null || true); do
      if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
           | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; then
        kill "$pid" 2>/dev/null || true
      fi
    done
  done
}

echo "=== 既存の画像認識プロセスを終了します ==="
kill_vision
sleep 1

cleanup() {
    echo ""
    echo "画像認識を終了しています..."
    kill_vision
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "=== 画像認識を起動します ==="
echo "--- カメラ: $DEVICE / 処理レート: $RATE Hz / 推論スレッド: $THREADS ---"
ros2 launch meister_vision pc_vision.launch.py \
    use_pc_camera:=true \
    image_topic:=/camera/image_raw \
    rate:="$RATE" \
    inference_threads:="$THREADS" \
    conf_threshold:="$CONF" &
LAUNCH_PID=$!

ros2 run rqt_image_view rqt_image_view /detection_image &
VIEWER_PID=$!

# detection_node は購読者が 0 の間は推論を止める設計なので、
# 窓が開かなかったまま黙って「何も検出されない」状態になるのを防ぐ。
for _ in $(seq 1 15); do
  if ros2 topic info /detection_image 2>/dev/null \
       | grep -q "Subscription count: [1-9]"; then
    break
  fi
  sleep 1
done
if ! ros2 topic info /detection_image 2>/dev/null \
     | grep -q "Subscription count: [1-9]"; then
  echo "ERROR: rqt_image_view が /detection_image を購読していません。" >&2
  echo "       映像窓が開いていないか、rqt_image_view が入っていない可能性があります。" >&2
fi

echo "------------------------------------------"
echo "起動完了！ 枠線とラベル付きの映像が rqt_image_view の窓に出ます。"
echo "- 検出結果の数値:  ros2 topic echo /detections --once"
echo "- 実際の処理速度:  ros2 topic hz /detection_image"
echo "- 元の素映像:      ros2 run rqt_image_view rqt_image_view /camera/image_raw"
echo "終了するには Ctrl+C を押してください。"
echo "------------------------------------------"

wait $LAUNCH_PID