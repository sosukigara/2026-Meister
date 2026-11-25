"""Measure /clock real-time factor over a fixed wall-clock window.

    python3 docs/perf/measure_rtf.py --seconds 20
"""
import argparse
import time

import rclpy
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("perf_rtf_probe")
    state = {"first": None, "last": None, "count": 0}

    def on_clock(message):
        seconds = message.clock.sec + message.clock.nanosec / 1e9
        if state["first"] is None:
            state["first"] = seconds
        state["last"] = seconds
        state["count"] += 1

    node.create_subscription(Clock, "/clock", on_clock, 10)
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.monotonic() - started
    sim = state["last"] - state["first"]
    print(f"clock_msgs={state['count']} wall={wall:.3f}s sim={sim:.3f}s "
          f"RTF={sim / wall:.3f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
