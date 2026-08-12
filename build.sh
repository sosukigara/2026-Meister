#!/bin/bash
# ==========================================
# Meistar ROS 2 ビルドスクリプト
# 使い方: ./build.sh
# 並列ビルド + symlink-install で高速化
# ==========================================
set -eo pipefail

WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)
cd "$WORKSPACE_ROOT"

# ROS 2 環境を自動検出
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

# 並列ワーカー数 (CPU コア数、上限 8)
WORKERS=$(nproc)
[ "$WORKERS" -gt 8 ] && WORKERS=8

echo "=== ビルド開始 (workers=$WORKERS) ==="
colcon build --symlink-install --parallel-workers "$WORKERS" \
    --packages-select meistar_description meister_vision meister_web_nav ros2_autonomous_nav
echo "=== ビルド完了 ==="
