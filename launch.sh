#!/bin/bash
# Meister — One-command launcher
# Usage:
#   ./launch.sh                          # maze + SLAM（デフォルト）
#   ./launch.sh warehouse                # ワールド指定 + SLAM
#   ./launch.sh maze nav                 # プリビルドマップ + AMCL モード
#   ./launch.sh maze explore             # SLAM + 自律探索モード
#   ./launch.sh maze slam headless       # headless（GUIなし）
#
# Available worlds: maze, warehouse, house, hospital, office, corridor, obstacles, empty

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# ── Arguments ──────────────────────────────────────────────────────────────────
WORLD="${1:-maze}"
MODE="${2:-slam}"            # slam | explore | nav
HEADLESS="${3:-false}"       # true で Gazebo GUI + RViz を省略

# ── ROS2 setup ─────────────────────────────────────────────────────────────────
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "WARNING: install/setup.bash not found. Run 'colcon build' first."
fi

# 一意の DDS ドメイン ID（デフォルト 10 だと前回の残骸と衝突するため）
export ROS_DOMAIN_ID=20

# Gazebo がワールドファイルを見つけられるように
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/src/diff_drive_robot/worlds"

# gz-transport を lo に固定（VPN トンネルがマルチキャスト通信を妨害するのを回避）
export GZ_IP=127.0.0.1

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║               Meister                            ║"
echo "║   ROS 2 Autonomous Navigation Stack              ║"
echo "╠══════════════════════════════════════════════════╣"
echo "║  World:          $WORLD"
echo "║  Mode:           $MODE"
echo "║  Headless:       $HEADLESS"
echo "║  ROS_DISTRO:     $ROS_DISTRO"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── GPU detection ──────────────────────────────────────────────────────────────
if nvidia-smi &>/dev/null; then
    echo "→ GPU detected (NVIDIA)"
else
    echo "→ No GPU detected (CPU mode)"
fi
echo ""

# ── Launch ─────────────────────────────────────────────────────────────────────
EXTRA_ARGS=()

case "$MODE" in
    explore)
        echo "→ Mode: SLAM + autonomous frontier exploration"
        EXTRA_ARGS+=(explore:=true)
        ;;
    nav)
        echo "→ Mode: Pre-built map + AMCL localization"
        EXTRA_ARGS+=(slam:=false)
        ;;
    slam|*)
        echo "→ Mode: SLAM (live mapping) + Nav2"
        EXTRA_ARGS+=(explore:=false)
        ;;
esac

if [ "$HEADLESS" = "true" ]; then
    EXTRA_ARGS+=(headless:=true)
fi

set -x
ros2 launch diff_drive_robot slam_nav.launch.py \
    world_name:="$WORLD" \
    "${EXTRA_ARGS[@]}"
