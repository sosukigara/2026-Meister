#!/usr/bin/env python3
"""ナビゲーションのベンチマーク。固定ゴールを順に走らせて JSON に記録する。

設計書 docs/superpowers/specs/2026-11-18-navigation-design.md §8.1/§8.2。
ROS に依存するのは main() だけで、指標の計算は bench_metrics.py に置いてある。

障害物セルは /global_costmap/costmap_raw（生の 0..255）から LETHAL 以上だけを
拾う。/global_costmap/costmap は 0..100 に変換済みで内接リング(253)を含むため、
そのまま使うと距離が実際より約 robot_radius ぶん小さく出る。
"""

import argparse
import json
import math
import os
import sys
import time
from types import SimpleNamespace

from bench_metrics import (
    cell_cost,
    contact_count,
    interval_stats,
    min_footprint_clearance,
    path_length,
    path_length_ratio,
    summarize,
)

# 機体寸法 [m]。設計書 §3: 0.6 x 0.4。
FOOTPRINT_HALF_LENGTH = 0.30
FOOTPRINT_HALF_WIDTH = 0.20

# 接触とみなす footprint 距離 [m]（設計書 §8.1）。
CONTACT_THRESHOLD = 0.02

# ゴール地点が「自由」とみなすコストマップ値（0..100 変換後）。
FREE_COST_MAX = 50

# nav2_msgs/Costmap の生値。LETHAL_OBSTACLE == 254。
LETHAL_RAW = 254

# CPU を積算する対象。launch が起動するノード名と対応させること（R2）。
CPU_TARGETS = (
    "controller_server",
    "planner_server",
    "bt_navigator",
    "behavior_server",
    "velocity_smoother",
    "amcl",
    "map_server",
    "slam_toolbox",
)


def load_goals(path):
    """ゴール定義 JSON を読む。"""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def validate_goals(doc, is_free):
    """ゴール定義の問題点を列挙する。空リストなら実行してよい。"""
    goals = doc.get("goals") or []
    if not goals:
        return ["no goals defined"]
    problems = []
    for i, goal in enumerate(goals):
        x, y = float(goal["x"]), float(goal["y"])
        if not is_free(x, y):
            problems.append(f"goal {i} ({x}, {y}) is not in free space")
    return problems


def _yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _read_cpu_ticks():
    """対象プロセスの {pid: (ノード名, utime+stime 秒)} を集める。"""
    found = {}
    clock = os.sysconf("SC_CLK_TCK")
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
            name = next((t for t in CPU_TARGETS if t in cmdline), None)
            if name is None:
                continue
            with open(f"/proc/{entry}/stat", "r", encoding="utf-8") as f:
                fields = f.read().rsplit(") ", 1)[1].split()
            found[entry] = (name, (int(fields[11]) + int(fields[12])) / clock)
        except (OSError, IndexError, ValueError):
            continue
    return found


def _grid_from_raw(costmap_msg):
    """nav2_msgs/Costmap を OccupancyGrid と同じ形のビューにし、占有セルを返す。"""
    meta = costmap_msg.metadata
    view = SimpleNamespace(
        data=costmap_msg.data,
        info=SimpleNamespace(
            resolution=meta.resolution,
            width=meta.size_x,
            height=meta.size_y,
            origin=SimpleNamespace(position=SimpleNamespace(
                x=meta.origin.position.x, y=meta.origin.position.y)),
        ),
    )
    res = meta.resolution
    ox, oy = meta.origin.position.x, meta.origin.position.y
    occupied = [
        (ox + (i % meta.size_x + 0.5) * res, oy + (i // meta.size_x + 0.5) * res)
        for i, value in enumerate(costmap_msg.data) if value >= LETHAL_RAW
    ]
    return view, occupied


def _empty_run(goal, reason):
    return {
        "goal": goal, "success": False, "reason": reason,
        "duration_s": 0.0, "path_length_m": 0.0,
        "path_length_ratio": None, "final_error_xy": None, "final_error_yaw": None,
        "min_clearance_m": None, "contact_count": None,
        "planner_latency_ms": None, "control_hz": None, "control_p99_ms": None,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goals", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--limit", type=int, default=None, help="先頭 N ゴールだけ実行する")
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--map-id", default="", help="使用地図の識別子（ファイル名やハッシュ）")
    args = parser.parse_args(argv)

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import ParameterValue
    from tf2_ros import Buffer, TransformListener

    doc = load_goals(args.goals)
    repeats = args.repeats if args.repeats is not None else int(doc.get("repeats", 1))
    goals = doc["goals"][: args.limit] if args.limit is not None else doc["goals"]

    rclpy.init()
    # シミュレーション時刻で動くスタックに合わせる。壁時計のままだとゴールの
    # stamp が TF と食い違い、Nav2 の transformPose が外挿エラーになる。
    node = Node("bench_nav",
                parameter_overrides=[ParameterValue("use_sim_time", True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    state = {"raw": None, "occupancy": None, "plan": None, "plan_at": None,
             "cmd_times": [], "plan_times": []}

    def _on_raw(msg):
        state["raw"] = msg

    def _on_occupancy(msg):
        state["occupancy"] = msg

    def _on_plan(msg):
        state["plan"] = msg
        state["plan_times"].append(time.monotonic())

    def _on_cmd(_msg):
        state["cmd_times"].append(time.monotonic())

    node.create_subscription(Costmap, "/global_costmap/costmap_raw", _on_raw, 10)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap", _on_occupancy, 10)
    node.create_subscription(Path, "plan", _on_plan, 10)
    node.create_subscription(Twist, "/cmd_vel_nav", _on_cmd, 10)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_for(8.0)
    if state["raw"] is None or state["occupancy"] is None:
        print(f"ERROR: コストマップを受信できません "
              f"(ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset')}, "
              f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', 'unset')}). "
              f"launch.sh と同じ環境変数を設定してください。", file=sys.stderr)
        return 2

    grid, occupied = _grid_from_raw(state["raw"])

    def is_free(x, y):
        cost = cell_cost(state["occupancy"], x, y)
        return cost is not None and 0 <= cost < FREE_COST_MAX

    problems = validate_goals({"goals": goals}, is_free)
    if problems:
        for p in problems:
            print(f"ERROR: {p}", file=sys.stderr)
        return 2

    if not client.wait_for_server(timeout_sec=10.0):
        print("ERROR: navigate_to_pose アクションサーバに接続できません", file=sys.stderr)
        return 2

    runs = []
    cpu_totals = {}
    cpu_wall = 0.0

    for repeat in range(repeats):
        for i, goal in enumerate(goals):
            gx, gy = float(goal["x"]), float(goal["y"])
            gyaw = float(goal.get("yaw", 0.0))
            label = {"x": gx, "y": gy, "yaw": gyaw}
            print(f"[{repeat + 1}/{repeats}] goal {i}: ({gx:.2f}, {gy:.2f}, {gyaw:.2f})")

            cpu_before = _read_cpu_ticks()
            wall_before = time.monotonic()
            state["plan_times"] = []
            state["cmd_times"] = []
            trajectory = []
            last_sample = 0.0
            started = time.monotonic()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp.sec = 0
            goal_msg.pose.header.stamp.nanosec = 0
            goal_msg.pose.pose.position.x = gx
            goal_msg.pose.pose.position.y = gy
            goal_msg.pose.pose.orientation.z = math.sin(gyaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(gyaw / 2.0)

            send_future = client.send_goal_async(goal_msg)
            while not send_future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - started > args.goal_timeout:
                    break
            handle = send_future.result()
            if handle is None or not handle.accepted:
                runs.append(_empty_run(label, "goal rejected"))
                continue

            result_future = handle.get_result_async()
            while not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if now - last_sample >= 1.0 / args.sample_hz:
                    last_sample = now
                    try:
                        tf = tf_buffer.lookup_transform("map", "base_footprint",
                                                        rclpy.time.Time())
                        p = tf.transform.translation
                        trajectory.append((p.x, p.y,
                                           _yaw_from_quaternion(tf.transform.rotation)))
                    except Exception:  # TF がまだ張られていない間は記録しない
                        pass
                if now - started > args.goal_timeout:
                    handle.cancel_goal_async()
                    break

            try:
                tf = tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                p = tf.transform.translation
                trajectory.append((p.x, p.y, _yaw_from_quaternion(tf.transform.rotation)))
            except Exception:
                pass

            result = result_future.result()
            success = (result is not None and result.status == GoalStatus.STATUS_SUCCEEDED)
            elapsed = time.monotonic() - started

            cpu_after = _read_cpu_ticks()
            span = time.monotonic() - wall_before
            cpu_wall += span
            for pid, (name, ticks) in cpu_after.items():
                if pid in cpu_before:
                    cpu_totals[name] = cpu_totals.get(name, 0.0) + (ticks - cpu_before[pid][1])

            if len(trajectory) < 2:
                runs.append(_empty_run(label, "no_trajectory"))
                continue

            final_x, final_y, final_yaw = trajectory[-1]
            planned = path_length([(p.pose.position.x, p.pose.position.y, 0.0)
                                   for p in state["plan"].poses]) if state["plan"] else 0.0
            latency_ms = ((state["plan_times"][0] - started) * 1000.0
                          if state["plan_times"] else None)
            control = interval_stats(state["cmd_times"])
            travelled = path_length(trajectory)
            runs.append({
                "goal": label,
                "success": bool(success),
                "duration_s": elapsed,
                "path_length_m": travelled,
                "planned_length_m": planned,
                "path_length_ratio": path_length_ratio(travelled, planned),
                "final_error_xy": math.hypot(final_x - gx, final_y - gy),
                "final_error_yaw": abs(math.atan2(math.sin(final_yaw - gyaw),
                                                  math.cos(final_yaw - gyaw))),
                "min_clearance_m": min_footprint_clearance(
                    trajectory, occupied, FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH),
                "contact_count": contact_count(
                    trajectory, occupied, FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH,
                    CONTACT_THRESHOLD),
                "planner_latency_ms": latency_ms,
                "control_hz": control["effective_hz"],
                "control_p99_ms": control["p99_ms"],
            })

    cpu_by_node = {name: round(v / cpu_wall, 3) for name, v in cpu_totals.items()} if cpu_wall else {}
    payload = {
        "world": doc.get("world", ""),
        "map_id": args.map_id,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "runs": runs,
        "cpu_core_fraction": cpu_by_node,
        "cpu_core_fraction_total": round(sum(cpu_by_node.values()), 3),
        "summary": summarize(runs),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
