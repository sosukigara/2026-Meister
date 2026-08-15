#!/usr/bin/env bash
# kill_ros.sh — ROS / Gazebo 関連プロセスを全て終了させる
# 使い方: ./kill_ros.sh
# 1) SIGINT で graceful 終了 → 2) 残っていれば SIGKILL → 3) ros2 daemon 停止
# 注意: このマシン上の全ての ROS スタック (Meister / robocon 等) が停止対象
set -u

PATTERNS=(
  "ros2 launch"
  "ros2 run"
  "ros2 bag record"
  "ros2 daemon"
  "gz sim"
  "gz launch"
  "slam_toolbox"
  "web_nav_server"
  "robot_state_publisher"
  "ros_gz_bridge"
  "parameter_bridge"
  "rviz2"
  "component_container"
  "component_container_mt"
  "lifecycle_manager"
  "bt_navigator"
  "controller_server"
  "planner_server"
  "smoother_server"
  "behavior_server"
  "waypoint_follower"
  "velocity_smoother"
  "map_server"
  "amcl"
  "param_dashboard"
  "smartphone_receiver"
  "unified_udp_bridge"
  "gamepad_command_to_joy"
  "mode_and_goal_manager"
  "monitor_node"
)

# 括弧トリックで kill_ros.sh 自身や grep にマッチしないようにする
kill_matches() {
  local sig="$1"
  for p in "${PATTERNS[@]}"; do
    pkill -"$sig" -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
}

count_left() {
  local n=0
  for p in "${PATTERNS[@]}"; do
    n=$((n + $(pgrep -fc "[${p:0:1}]${p:1}" 2>/dev/null || echo 0)))
  done
  echo "$n"
}

echo "==> ROS 関連プロセスを SIGINT で graceful 終了..."
kill_matches INT
sleep 3

echo "==> 残存プロセスを SIGKILL で強制終了..."
kill_matches KILL
sleep 1

echo "==> ros2 daemon を停止..."
ros2 daemon stop 2>/dev/null || true

left=$(count_left)
if [ "$left" -eq 0 ]; then
  echo "==> 全ての ROS 関連プロセスを停止しました"
else
  echo "==> 警告: $left プロセスが残っています"
  pgrep -af "ros2|gz sim|slam_toolbox|web_nav|bridge|component_container" \
    | grep -v "kill_ros" || true
fi
