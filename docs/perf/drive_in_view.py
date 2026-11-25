"""Drive slowly in front of the fixed vision target for a long in-view window.

    python3 docs/perf/drive_in_view.py --seconds 75 --out /tmp/in_view.json

The stable-drive stack routes controller -> /cmd_vel_nav -> velocity_smoother
-> /cmd_vel. Publishing the gentle back-and-forth profile to /cmd_vel_nav
therefore drives the real robot path without competing with the smoother.
This is a *driving* session: /cmd_vel and /odom move for the whole window.
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=75.0)
    parser.add_argument("--topic", default="/cmd_vel_nav")
    parser.add_argument("--period", type=float, default=20.0,
                        help="back-and-forth period [s]")
    parser.add_argument("--amplitude", type=float, default=0.12,
                        help="peak linear.x [m/s]")
    parser.add_argument("--bias", type=float, default=0.0,
                        help="constant linear.x [m/s] (repositioning)")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--out", default="/tmp/in_view.json")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("drive_in_view")
    publisher = node.create_publisher(Twist, args.topic, 10)
    positions = []
    node.create_subscription(
        Odometry, "/odom", lambda msg: positions.append(
            (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
             msg.pose.pose.position.x, msg.pose.pose.position.y)), 10)

    start = time.monotonic()
    samples = 0
    while time.monotonic() - start < args.seconds:
        elapsed = time.monotonic() - start
        twist = Twist()
        twist.linear.x = args.bias + args.amplitude * math.sin(
            2.0 * math.pi * elapsed / args.period)
        publisher.publish(twist)
        samples += 1
        deadline = start + samples / args.rate
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.005)

    twist = Twist()
    for _ in range(5):
        publisher.publish(twist)
        time.sleep(0.05)

    span_x = (max(p[1] for p in positions) - min(p[1] for p in positions)
              if positions else 0.0)
    result = {
        "seconds": round(time.monotonic() - start, 3),
        "commands": samples,
        "topic": args.topic,
        "amplitude_mps": args.amplitude,
        "period_s": args.period,
        "odom_samples": len(positions),
        "odom_x_span_m": round(span_x, 3),
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
