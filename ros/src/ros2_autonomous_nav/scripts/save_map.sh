#!/bin/bash
source /opt/ros/jazzy/setup.bash
NAME="${1:-my_new_map}"
OUT_DIR="$(dirname "$0")/../maps"
mkdir -p "$OUT_DIR"
ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$NAME"
echo "地図を $OUT_DIR/$NAME に保存しました。"
