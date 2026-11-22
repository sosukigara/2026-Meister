#!/usr/bin/env bash
# 本計測用ラッパー。理由は 2 つある。
#
# 1. Gazebo の GUI は DISPLAY が無いと Qt の初期化で abort する。
# 2. この環境では、launch を起動したプロセス群と別プロセスとの間で DDS が
#    可視でないことがある。起動と計測を同じプロセスツリーで行う必要がある。
#    さらに spawn に失敗して TF が張られないことがあるので、準備完了を
#    確認してから計測を始める（失敗したら起動し直す）。
#
# 使い方:
#   ros/src/umeonigiri_autonomy/scripts/measure.sh <出力JSON> [limit] [repeats] [map-id]
#   MEISTER_LAUNCH_ARGS="controller:=mppi" .../measure.sh out.json
set -uo pipefail

OUT=${1:?出力 JSON のパスを指定してください}
LIMIT=${2:-10}
REPEATS=${3:-3}
MAP_ID=${4:-slam-warehouse}

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/../../../.." && pwd)
cd "$ROOT" || exit 1

export DISPLAY=${DISPLAY:-:1}
export ROS_DOMAIN_ID=${MEISTER_DOMAIN:-5}
export ROS_LOCALHOST_ONLY=1

MAX_ATTEMPTS=${MEISTER_ATTEMPTS:-3}
ready=0
for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  echo "--- 起動試行 $attempt"
  ./launch.sh --sim warehouse > "/tmp/measure_launch_$attempt.log" 2>&1 &
  for _ in $(seq 1 50); do
    sleep 3
    status=$(curl -s --max-time 3 http://127.0.0.1:8088/api/status 2>/dev/null)
    pose=$(curl -s --max-time 3 http://127.0.0.1:8088/api/pose 2>/dev/null)
    if [[ $status == *'"nav2_ready":true'* && $pose == *'"has_pose":true'* ]]; then
      ready=1
      echo "準備完了"
      break
    fi
  done
  [ $ready -eq 1 ] && break
  echo "準備できず。停止して再試行"
  for pid in $(pgrep -f "ros2 launch umeonigiri_autonomy"); do kill -INT "$pid" 2>/dev/null; done
  sleep 8
done

if [ $ready -ne 1 ]; then
  echo "起動に失敗しました（/tmp/measure_launch_*.log を確認）" >&2
  exit 1
fi

# SLAM の地図ができてゴール検証が通るまで少し待つ。
sleep 10

# ROS の setup.bash は未定義変数を参照するので、この間だけ -u を外す。
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros/install/setup.bash"
set -u

"$ROOT/ros/install/umeonigiri_autonomy/lib/umeonigiri_autonomy/bench_nav" \
  --goals "$SCRIPT_DIR/../config/bench_goals.json" \
  --out "$OUT" \
  --limit "$LIMIT" \
  --repeats "$REPEATS" \
  --map-id "$MAP_ID" \
  --gui on 2>&1 | grep -vE "WARN"
