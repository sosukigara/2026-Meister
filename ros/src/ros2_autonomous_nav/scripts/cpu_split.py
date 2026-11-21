#!/usr/bin/env python3
"""区間内のプロセス別 CPU 使用率を積分して内訳を出す。

`/proc/<pid>/stat` の utime+stime の差分を経過時間で割る。%表記は 1 コア = 100%。
マシン全体の使用率は core 数で割って併記する。top の瞬間値は数秒で振れるため、
区間積分で比べる。ROS 非依存（rclpy も ROS も使わない）。

使い方:
    python3 ros/src/ros2_autonomous_nav/scripts/cpu_split.py --seconds 20
    ... --label "headless" で見出しを付ける
"""
from __future__ import annotations

import argparse
import os
import time
from collections import defaultdict

# 対象は「このスタックの構成要素」と「描画系」。名前で拾う。
PATTERNS = (
    "gz sim", "gz-sim", "gz_sim", "ruby",
    "detection_node", "pc_camera", "hand_landmarks",
    "rviz2", "controller_server", "planner_server", "bt_navigator",
    "behavior_server", "velocity_smoother", "amcl", "map_server",
    "map_saver", "slam_toolbox", "umeonigiri_webnav_server", "robot_state_publisher",
    "parameter_bridge", "lifecycle_manager", "nav_mode_manager",
    "gnome-shell", "Xwayland", "firefox", "chrome", "node",
)

# gz sim は ruby 経由で動くので、gui/server を区別して束ねる。
GZ_ALIASES = (("gz sim gui", "gz sim gui"), ("gz sim server", "gz sim server"))


def read_ticks(pid: str) -> int | None:
    try:
        with open(f"/proc/{pid}/stat") as f:
            raw = f.read()
        # comm は括弧の中にあり、空白や ')' を含み得る。単純に split すると
        # 以降のフィールドがずれ、CPU 時間を 0 と誤読する (実測: 144 tick を 0)。
        # C++ 版 (bench_proc.cpp) と同じく最後の ')' で切る。
        close = raw.rfind(")")
        if close < 0:
            return None
        # ") " の直後が field 3 (state)。utime は field 14、stime は field 15。
        fields = raw[close + 2:].split()
        return int(fields[11]) + int(fields[12])
    except (OSError, IndexError, ValueError):
        return None


def read_cmd(pid: str) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            raw = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
    except OSError:
        return ""
    for needle, alias in GZ_ALIASES:
        if needle in raw:
            return alias
    if "gz sim" in raw:
        return "gz sim"
    return raw[:72].strip()


def sample() -> dict[str, tuple[int, str]]:
    out: dict[str, tuple[int, str]] = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        cmd = read_cmd(pid)
        if not cmd or not any(p in cmd for p in PATTERNS):
            continue
        ticks = read_ticks(pid)
        if ticks is not None:
            out[pid] = (ticks, cmd)
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    before = sample()
    started = time.monotonic()
    time.sleep(args.seconds)
    elapsed = time.monotonic() - started
    after = sample()

    per_cmd: dict[str, float] = defaultdict(float)
    tick_hz = os.sysconf("SC_CLK_TCK")
    for pid, (ticks, cmd) in after.items():
        if pid in before:
            delta = ticks - before[pid][0]
            per_cmd[cmd] += delta / tick_hz / elapsed * 100.0

    if args.label:
        print(f"# {args.label}")
    for cmd, pct in sorted(per_cmd.items(), key=lambda kv: -kv[1]):
        if pct >= 0.5:
            print(f"{pct:7.1f}% of core  {cmd}")
    total = sum(per_cmd.values())
    cores = os.cpu_count() or 1
    print(f"--- total {total:.1f}% of one core = {total / cores:.1f}% of {cores} cores")


if __name__ == "__main__":
    main()
