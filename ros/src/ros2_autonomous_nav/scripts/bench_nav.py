#!/usr/bin/env python3
"""ナビゲーションのベンチマーク。固定ゴールを順に走らせて JSON に記録する。

設計書 docs/superpowers/specs/2026-11-18-navigation-design.md §8.2 の手順。
ROS に依存するのは main() だけで、指標の計算は bench_metrics.py に置いてある。
"""

import argparse
import json
import math
import os
import sys
import time

from bench_metrics import min_footprint_clearance, path_length, summarize

# 機体寸法 [m]。設計書 §3: 0.6 x 0.4。
FOOTPRINT_HALF_LENGTH = 0.30
FOOTPRINT_HALF_WIDTH = 0.20

# コストマップの値。nav2_costmap_2d の LETHAL_OBSTACLE は 254、INSCRIBED は 253。
FREE_COST_MAX = 50
OCCUPIED_COST_MIN = 99

# CPU を積算する対象。実行ファイル名で /proc を照合する。
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
    """対象プロセスの (pid -> utime+stime) を集める。"""
    ticks = {}
    clock = os.sysconf("SC_CLK_TCK")
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
            if not any(name in cmdline for name in CPU_TARGETS):
                continue
            with open(f"/proc/{entry}/stat", "r", encoding="utf-8") as f:
                fields = f.read().rsplit(") ", 1)[1].split()
            ticks[entry] = (int(fields[11]) + int(fields[12])) / clock
        except (OSError, IndexError, ValueError):
            continue
    return ticks


def _occupied_cells(costmap_msg):
    """LETHAL 以上を世界座標の点にする。"""
    info = costmap_msg.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    width, height = info.width, info.height
    cells = []
    for index, value in enumerate(costmap_msg.data):
        if value >= OCCUPIED_COST_MIN:
            gx, gy = index % width, index // width
            cells.append((ox + (gx + 0.5) * res, oy + (gy + 0.5) * res))
    return cells


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goals", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--limit", type=int, default=None, help="先頭 N ゴールだけ実行する")
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    args = parser.parse_args(argv)

    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    doc = load_goals(args.goals)
    repeats = args.repeats if args.repeats is not None else int(doc.get("repeats", 1))
    goals = doc["goals"][: args.limit] if args.limit else doc["goals"]

    rclpy.init()
    node = Node("bench_nav")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    costmap_holder = {}

    def _on_costmap(msg):
        costmap_holder["msg"] = msg

    node.create_subscription(OccupancyGrid, "/global_costmap/costmap", _on_costmap, 10)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_for(5.0)
    if "msg" not in costmap_holder:
        print("ERROR: /global_costmap/costmap を受信できません", file=sys.stderr)
        return 2

    costmap = costmap_holder["msg"]
    resolution = costmap.info.resolution
    origin = costmap.info.origin.position
    occupied = _occupied_cells(costmap)

    def is_free(x, y):
        gx = int((x - origin.x) / resolution)
        gy = int((y - origin.y) / resolution)
        if gx < 0 or gy < 0 or gx >= costmap.info.width or gy >= costmap.info.height:
            return False
        return costmap.data[gy * costmap.info.width + gx] < FREE_COST_MAX

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
            gx, gy, gyaw = float(goal["x"]), float(goal["y"]), float(goal.get("yaw", 0.0))
            print(f"[{repeat + 1}/{repeats}] goal {i}: ({gx:.2f}, {gy:.2f}, {gyaw:.2f})")

            cpu_before = _read_cpu_ticks()
            wall_before = time.monotonic()
            trajectory = []
            last_sample = 0.0
            started = time.monotonic()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = gx
            goal_msg.pose.pose.position.y = gy
            goal_msg.pose.pose.orientation.z = math.sin(gyaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(gyaw / 2.0)

            send_future = client.send_goal_async(goal_msg)
            while not send_future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                runs.append({"goal": goal, "success": False, "duration_s": 0.0,
                             "path_length_m": 0.0, "final_error_xy": float("nan"),
                             "final_error_yaw": float("nan"), "min_clearance_m": 0.0})
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
                    except Exception:  # TF がまだ無い間は記録しない
                        pass
                if now - started > args.goal_timeout:
                    handle.cancel_goal_async()
                    break

            result = result_future.result()
            success = (result is not None
                       and result.status == GoalStatus.STATUS_SUCCEEDED)
            elapsed = time.monotonic() - started

            cpu_after = _read_cpu_ticks()
            span = time.monotonic() - wall_before
            cpu_wall += span
            for pid, ticks in cpu_after.items():
                if pid in cpu_before:
                    cpu_totals[pid] = cpu_totals.get(pid, 0.0) + (ticks - cpu_before[pid])

            final_x, final_y, final_yaw = trajectory[-1] if trajectory else (float("nan"),) * 3
            runs.append({
                "goal": {"x": gx, "y": gy, "yaw": gyaw},
                "success": bool(success),
                "duration_s": elapsed,
                "path_length_m": path_length(trajectory),
                "final_error_xy": math.hypot(final_x - gx, final_y - gy),
                "final_error_yaw": abs(math.atan2(math.sin(final_yaw - gyaw),
                                                  math.cos(final_yaw - gyaw))),
                "min_clearance_m": min_footprint_clearance(
                    trajectory, occupied, FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH),
            })

    cpu_ratio = {pid: ticks / cpu_wall for pid, ticks in cpu_totals.items()} if cpu_wall else {}
    payload = {
        "world": doc.get("world", ""),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "runs": runs,
        "cpu_core_fraction": {pid: round(v, 3) for pid, v in cpu_ratio.items()},
        "summary": summarize(runs),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
