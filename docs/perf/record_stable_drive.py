"""Record one driving session: /detections, /arm_camera/image_raw, /cmd_vel, /clock.

    python3 docs/perf/record_stable_drive.py --seconds 180 \
        --out docs/perf/logs/drive_run.jsonl --image-dir /tmp/drive_images

The JSONL is the raw evidence: every message is written with a wall-clock
timestamp.  Image frames are sampled for an independent "target visible" oracle
in summarize_stable_drive.py (they are not detector output).
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


def detection_record(message):
    results = []
    for detection in message.detections:
        hypothesis = detection.results[0].hypothesis if detection.results else None
        results.append({
            "id": detection.id,
            "class_id": None if hypothesis is None else hypothesis.class_id,
            "score": None if hypothesis is None else hypothesis.score,
            "cx": detection.bbox.center.position.x,
            "cy": detection.bbox.center.position.y,
            "w": detection.bbox.size_x,
            "h": detection.bbox.size_y,
        })
    return {"n": len(results), "items": results}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--out", default="/tmp/drive_session.jsonl")
    parser.add_argument("--image-dir", default="/tmp/drive_images")
    parser.add_argument("--image-every", type=int, default=5)
    args = parser.parse_args()
    os.makedirs(args.image_dir, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("stable_drive_recorder")
    handle = open(args.out, "w", encoding="utf-8")
    counts = {"detections": 0, "image": 0, "cmd_vel": 0, "clock": 0, "odom": 0}
    saved = 0

    def write(topic, data):
        handle.write(json.dumps({"topic": topic, "wall": time.monotonic(),
                                 "epoch": time.time(), "data": data}, ensure_ascii=False) + "\n")

    def on_detection(message):
        counts["detections"] += 1
        write("detections", detection_record(message))

    def on_image(message):
        nonlocal saved
        counts["image"] += 1
        saved_path = None
        if counts["image"] % args.image_every == 0 and message.encoding == "rgb8":
            image = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.width, 3)
            saved_path = os.path.join(args.image_dir, f"{counts['image']:06d}.png")
            cv2.imwrite(saved_path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            saved += 1
        write("image", {"sec": message.header.stamp.sec,
                        "nanosec": message.header.stamp.nanosec,
                        "encoding": message.encoding,
                        "width": message.width, "height": message.height,
                        "saved": saved_path})

    def on_twist(message):
        counts["cmd_vel"] += 1
        write("cmd_vel", {"linear_x": message.linear.x,
                          "angular_z": message.angular.z})

    def on_clock(message):
        counts["clock"] += 1
        write("clock", {"sec": message.clock.sec, "nanosec": message.clock.nanosec})

    def on_odom(message):
        counts["odom"] += 1
        write("odom", {"x": message.pose.pose.position.x,
                       "y": message.pose.pose.position.y})

    node.create_subscription(Detection2DArray, "/detections", on_detection, 20)
    node.create_subscription(Image, "/arm_camera/image_raw", on_image, 20)
    node.create_subscription(Twist, "/cmd_vel", on_twist, 20)
    node.create_subscription(Clock, "/clock", on_clock, 20)
    node.create_subscription(Odometry, "/odom", on_odom, 20)

    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        handle.close()
        node.destroy_node()
        rclpy.shutdown()
    elapsed = time.monotonic() - started
    print(json.dumps({"elapsed": round(elapsed, 3), "counts": counts,
                      "images_saved": saved, "out": args.out}), flush=True)


if __name__ == "__main__":
    main()
