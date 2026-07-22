#!/bin/bash
# Save the current SLAM /map to disk as map.pgm + map.yaml
# Usage: bash save_map.sh [output_name]
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash

OUT_NAME=${1:-nav_world}
OUT_DIR=$(dirname "$0")/maps
mkdir -p "$OUT_DIR"

echo "Saving current map to $OUT_DIR/$OUT_NAME.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$OUT_NAME" --ros-args -p use_sim_time:=true
