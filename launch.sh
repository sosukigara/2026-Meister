#!/bin/bash
# launch.sh — One-shot Launcher for Meister ROS 2 + Gazebo + Nav2
#
# Usage:
#   ./launch.sh [options] [world_name]
#
# Examples:
#   ./launch.sh              # build → maze world → SLAM + Nav2 + RViz
#   ./launch.sh --no-build   # skip build, just launch sim
#   ./launch.sh warehouse    # build → warehouse world → sim
#   ./launch.sh --setup      # setup deps → build → sim
#   ./launch.sh --record     # build → sim → ros2 bag record
#   ./launch.sh --record --no-build warehouse  # sim + record in warehouse
#
# Options:
#   --setup       Run setup.sh first (apt install + rosdep)
#   --no-build    Skip colcon build (use existing binaries)
#   --record      Record ros2 bag alongside simulation
#   -h, --help    Show this help

cd "$(dirname "$0")"

# Source ROS 2 environment BEFORE set -u (ROS setup.bash references unset vars)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

set -euo pipefail

# ---- Parse arguments ----
DO_SETUP=false
DO_BUILD=true
DO_RECORD=false
WORLD="maze"
POSITIONAL=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --setup)    DO_SETUP=true; shift ;;
    --no-build) DO_BUILD=false; shift ;;
    --record)   DO_RECORD=true; shift ;;
    -h|--help)
      sed -n '2,19p' "$0" | sed 's/^# \?//'
      exit 0 ;;
    *)
      POSITIONAL+=("$1")
      shift ;;
  esac
done

# First positional arg = world name
if [[ ${#POSITIONAL[@]} -gt 0 ]]; then
  WORLD="${POSITIONAL[0]}"
fi

echo "=========================================="
echo " Meister Launcher"
echo " World:        $WORLD"
echo " Setup:        $DO_SETUP"
echo " Build:        $DO_BUILD"
echo " Record:       $DO_RECORD"
echo " ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "=========================================="

# ---- Phase 1: Setup (first-time only) ----
if $DO_SETUP; then
  echo ""
  echo "[1/3] Installing dependencies..."
  bash scripts/setup.sh
fi

# ---- Phase 2: Build ----
if $DO_BUILD; then
  echo ""
  echo "[2/3] Building workspace..."
  bash scripts/build.sh
fi

# ---- Phase 3: Launch Simulation ----
echo ""
echo "[3/3] Launching simulation (world=$WORLD)..."

if $DO_RECORD; then
  # Launch sim + bag record side-by-side, kill both on Ctrl-C
  BAG_DIR="bag_$(date +%Y%m%d_%H%M%S)"
  echo "Recording bag → $BAG_DIR"

  # Trap Ctrl-C to stop both processes cleanly
  trap 'kill 0' SIGINT SIGTERM EXIT

  ros2 launch meister_robot meister_slam_nav.launch.py \
    world_name:="$WORLD" explore:=true &
  PID_SIM=$!

  sleep 3
  ros2 bag record -o "$BAG_DIR" \
    /scan /tf /tf_static /odom /map /cmd_vel /goal_pose /clicked_point &
  PID_BAG=$!

  echo "Sim PID: $PID_SIM  |  Bag PID: $PID_BAG"
  echo "Press Ctrl-C to stop both."

  wait $PID_SIM
  wait $PID_BAG
else
  # Sim only (no record)
  exec ros2 launch meister_robot meister_slam_nav.launch.py \
    world_name:="$WORLD" explore:=true
fi
