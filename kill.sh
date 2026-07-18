#!/bin/bash
# Meister — Kill all ROS2 + Gazebo processes
#
# Usage:
#   ./kill.sh          # 通常のクリーンアップ
#   ./kill.sh -f       # 強制 kill（SIGKILL）
#   ./kill.sh -q       # サイレントモード

# ── Flags ──────────────────────────────────────────────────────────────────────
FORCE=false
QUIET=false
while getopts "fq" opt; do
    case "$opt" in
        f) FORCE=true ;;
        q) QUIET=true ;;
        *) ;;
    esac
done

SIG="TERM"
[ "$FORCE" = true ] && SIG="KILL"

# ── Helper ─────────────────────────────────────────────────────────────────────
_kill() {
    local pattern="$1"
    local sig="${2:-$SIG}"
    if [ "$QUIET" = false ]; then
        echo "  → pkill -$sig -f \"$pattern\""
    fi
    pkill -"$sig" -f "$pattern" 2>/dev/null || true
}

# ── Main ───────────────────────────────────────────────────────────────────────
if [ "$QUIET" = false ]; then
    echo ""
    echo "╔══════════════════════════════════════════════════╗"
    echo "║           Meister — Cleanup                      ║"
    echo "╠══════════════════════════════════════════════════╣"
    echo "║  Signal:  SIG$SIG"
    echo "╚══════════════════════════════════════════════════╝"
    echo ""
fi

# 1. ROS2 launch processes（slam_nav, multi_robot, robot_nav 等）
[ "$QUIET" = false ] && echo "→ Killing ROS2 launch processes..."
_kill "slam_nav.launch.py"
_kill "multi_robot.launch.py"
_kill "robot.launch.py"
_kill "bringup_launch.py"

# 2. ROS2 nodes を node list から kill
if command -v ros2 &>/dev/null; then
    [ "$QUIET" = false ] && echo "→ Killing ROS2 nodes (via ros2 node kill)..."
    ros2 node list 2>/dev/null | while IFS= read -r node; do
        ros2 node kill "$node" 2>/dev/null || true
    done
fi

# 3. Gazebo / gz-sim
[ "$QUIET" = false ] && echo "→ Killing Gazebo..."
_kill "gz sim"
_kill "gzserver"
_kill "gzclient"

# 4. RViz
[ "$QUIET" = false ] && echo "→ Killing RViz..."
_kill "rviz2"
_kill "rviz"

# 5. ros_gz_bridge
[ "$QUIET" = false ] && echo "→ Killing ros_gz_bridge..."
_kill "ros_gz_bridge"
_kill "parameter_bridge"

# 6. diff_drive_robot の Python スクリプト類
[ "$QUIET" = false ] && echo "→ Killing diff_drive_robot scripts..."
_kill "frontier_explorer.py"
_kill "mission_server.py"
_kill "collision_monitor.py"
_kill "fleet_manager.py"
_kill "fleet_gui.py"
_kill "llm_nav.py"
_kill "bt_server.py"
_kill "bt_executor.py"
_kill "multi_teleop.py"
_kill "waypoint_nav.py"
_kill "coverage_planner.py"
_kill "obstacle_tracker.py"
_kill "fleet_health.py"
_kill "task_allocator.py"

# 7. 残った ros2 プロセス
[ "$QUIET" = false ] && echo "→ Killing remaining ROS2 processes..."
_kill "ros2 launch"
_kill "ros2 run"

# 8. Cyclone DDS のドメイン参加者情報をクリア（残骸があると次回起動時に衝突）
rm -rf /tmp/cyclone*_default_domain 2>/dev/null || true
rm -f /tmp/diff_drive_nav2_patched_*.yaml 2>/dev/null

# ── Verification ───────────────────────────────────────────────────────────────
sleep 1
REMAINING=$(pgrep -f "ros2|gz sim|rviz2|parameter_bridge|diff_drive_robot/scripts" 2>/dev/null | wc -l)

if [ "$QUIET" = false ]; then
    echo ""
    if [ "$REMAINING" -gt 0 ]; then
        echo "⚠  $REMAINING process(es) still remaining."
        echo "   Run './kill.sh -f' to force kill."
        pgrep -af "ros2|gz sim|rviz2|diff_drive_robot/scripts" 2>/dev/null || true
    else
        echo "✓ All processes cleaned up."
    fi
    echo ""
fi
