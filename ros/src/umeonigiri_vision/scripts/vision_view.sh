#!/usr/bin/env bash
# 画像認識の状況を RViz で見る。カメラ・検出・TF・RViz をまとめて起動する。
#
#   ros/src/umeonigiri_vision/scripts/vision_view.sh
#   DEVICE=/dev/video0 RATE=30 THREADS=2 MODEL=/path/to/yolov8n-320.onnx vision_view.sh
#
# 表示は detection.rviz（左: 検出枠つき /detection_image、右タブ: 生画像）。
# RViz の Fixed Frame は camera_link。TF を出すために robot_state_publisher も
# 起動する（これが無いと Global Status が Warn になり 3D ビューが空になる）。
set -eo pipefail

DEVICE="${DEVICE:-/dev/video0}"
RATE="${RATE:-30.0}"
THREADS="${THREADS:-2}"
CONF="${CONF:-0.25}"

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/../../../.." && pwd)
cd "$ROOT"

source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# 既定は速い 320 モデル。無ければ自動解決（640 = 遅いが高精細）。
if [ -z "${MODEL:-}" ]; then
  CAND="$ROOT/ros/install/umeonigiri_vision/share/umeonigiri_vision/models/yolov8n-320.onnx"
  [ -f "$CAND" ] && MODEL="$CAND"
fi

xacro ros/install/umeonigiri_description/share/umeonigiri_description/urdf/robot.urdf.xacro \
  > /tmp/vision_view_robot.urdf

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  exit 0
}
trap cleanup SIGINT SIGTERM EXIT

ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat /tmp/vision_view_robot.urdf)" \
  -p use_sim_time:=false > /tmp/vision_view_rsp.log 2>&1 &
pids+=($!)

ros/install/umeonigiri_vision/lib/umeonigiri_vision/pc_camera --ros-args \
  -p device:="$DEVICE" -p fps:=30.0 > /tmp/vision_view_cam.log 2>&1 &
pids+=($!)

# 検出ノードは購読者が 0 の間は推論しない。RViz が購読者になるので起動順は後でよい。
ros/install/umeonigiri_vision/lib/umeonigiri_vision/detection_node --ros-args \
  -p image_topic:=/camera/image_raw -p rate:="$RATE" \
  -p inference_threads:="$THREADS" -p conf_threshold:="$CONF" \
  ${MODEL:+-p model_path:="$MODEL"} > /tmp/vision_view_det.log 2>&1 &
pids+=($!)

echo "=== RViz を起動（検出枠が左パネル、生画像はタブ）==="
rviz2 -d "$ROOT/ros/install/umeonigiri_vision/share/umeonigiri_vision/rviz/detection.rviz" \
  > /tmp/vision_view_rviz.log 2>&1 &
pids+=($!)

echo "- 検出レート: ros2 topic hz /detections"
echo "- 描画画像:   ros2 topic hz /detection_image"
echo "- 生ログ:     /tmp/vision_view_{rsp,cam,det,rviz}.log"
echo "終了は Ctrl+C。"
wait "${pids[-1]}"
