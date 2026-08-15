#!/usr/bin/env bash
# kill_ros.sh — 指定ドメインの ROS / Gazebo 関連プロセスのみ終了させる
# 使い方: ./kill_ros.sh
#   - 対象ドメインは既定で 5 (ROS_DOMAIN_ID 環境変数で上書き可)
#   - ROS_DOMAIN_ID が一致するプロセスのみ SIGINT → SIGKILL → daemon 停止
# 注意: 他ドメイン (robocon 等) のスタックには一切触れない
set -u

TARGET_DOMAIN="${ROS_DOMAIN_ID:-5}"
export ROS_DOMAIN_ID="$TARGET_DOMAIN"

PATTERNS=(
  "ros-args"
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
  "ros2-daemon"
)

# 括弧トリックで kill_ros.sh 自身や grep にマッチしないようにする
# さらに /proc/<pid>/environ で ROS_DOMAIN_ID が一致するプロセスのみ対象にする
pid_on_target_domain() {
  local pid="$1"
  [ -r "/proc/$pid/environ" ] || return 1
  tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
    | grep -qx "ROS_DOMAIN_ID=$TARGET_DOMAIN"
}

kill_matches() {
  local sig="$1"
  for p in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f "[${p:0:1}]${p:1}" 2>/dev/null); do
      if pid_on_target_domain "$pid"; then
        kill -"$sig" "$pid" 2>/dev/null || true
      fi
    done
  done
}

count_left() {
  local n=0
  for p in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f "[${p:0:1}]${p:1}" 2>/dev/null); do
      if pid_on_target_domain "$pid"; then
        n=$((n + 1))
      fi
    done
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
  echo "==> domain $TARGET_DOMAIN の ROS 関連プロセスを全て停止しました"
else
  echo "==> 警告: domain $TARGET_DOMAIN に $left プロセスが残っています"
  pgrep -af "ros2|gz sim|slam_toolbox|web_nav|bridge|component_container" \
    | grep -v "kill_ros" \
    | while IFS= read -r line; do
        pid=$(echo "$line" | awk '{print $1}')
        if pid_on_target_domain "$pid"; then echo "$line"; fi
      done || true
fi
