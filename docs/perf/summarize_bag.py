"""Summarise a rosbag2 drive session recorded with `ros2 bag record`.

    ros2 bag record -o docs/perf/logs/drive_bag --topics \
        /detections /arm_camera/image_raw /cmd_vel /clock /odom -s sqlite3
    python3 docs/perf/summarize_bag.py --bag docs/perf/logs/drive_bag \
        --out docs/perf/logs/drive_bag_summary.json --image-dir /tmp/drive_bag_images

rosbag2 stores every message, so no Python subscription can drop samples.
Visibility is judged from sampled *images* against docs/perf/bus.jpg, never
from detector output, so continuity is not measured circularly.
"""
import argparse
import json
import math
import os
import statistics

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

TOPICS = {
    "/detections": Detection2DArray,
    "/arm_camera/image_raw": Image,
    "/cmd_vel": Twist,
    "/clock": Clock,
    "/odom": Odometry,
}


def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    reader.set_filter(StorageFilter(topics=list(TOPICS)))
    rows = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in TOPICS:
            rows.append((topic, stamp / 1e9, deserialize_message(data, TOPICS[topic])))
    return rows


def hist_of(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [18, 16], [0, 180, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def gaps(times, limit):
    return [round(b - a, 3) for a, b in zip(times, times[1:]) if b - a > limit]


def rtf_windows(clock_rows, window_s=20.0):
    if len(clock_rows) < 2:
        return []
    stamps = [(stamp, clock.clock.sec + clock.clock.nanosec / 1e9)
              for _, stamp, clock in clock_rows]
    start = stamps[0][0]
    result = []
    while start + window_s <= stamps[-1][0]:
        inside = [item for item in stamps if start <= item[0] <= start + window_s]
        if len(inside) >= 2:
            wall_span = inside[-1][0] - inside[0][0]
            sim_span = inside[-1][1] - inside[0][1]
            if wall_span > 0:
                result.append(round(sim_span / wall_span, 3))
        start += window_s
    return result


def summarise(args, rows):
    if not rows:
        raise RuntimeError("バッグにメッセージがありません")
    by_topic = {}
    for row in rows:
        by_topic.setdefault(row[0], []).append(row)
    start = min(row[1] for row in rows)
    span = max(row[1] for row in rows) - start
    det = by_topic.get("/detections", [])
    images = by_topic.get("/arm_camera/image_raw", [])
    cmd = by_topic.get("/cmd_vel", [])
    clocks = by_topic.get("/clock", [])
    odom = by_topic.get("/odom", [])
    if not det or not clocks:
        raise RuntimeError("/detections または /clock がありません")

    det_times = [row[1] for row in det]
    nonempty = [row for row in det if len(row[2].detections) > 0]
    labels = {}
    centers = {}
    for _, _, message in nonempty:
        for detection in message.detections:
            hypothesis = detection.results[0].hypothesis if detection.results else None
            labels[detection.id] = labels.get(detection.id, 0) + 1
            if detection.id == "person":
                centers.setdefault(detection.id, []).append(
                    (detection.bbox.center.position.x, detection.bbox.center.position.y))

    lin_jumps = ang_jumps = 0
    max_lin = max_ang = 0.0
    for a, b in zip(cmd, cmd[1:]):
        dlin = abs(b[2].linear.x - a[2].linear.x)
        dang = abs(b[2].angular.z - a[2].angular.z)
        max_lin = max(max_lin, dlin)
        max_ang = max(max_ang, dang)
        lin_jumps += dlin > 0.3
        ang_jumps += dang > 0.5

    reference = hist_of(cv2.imread(args.reference, cv2.IMREAD_COLOR))
    os.makedirs(args.image_dir, exist_ok=True)
    visible = []
    image_every = max(1, args.image_every)
    for index, (_, wall, message) in enumerate(images):
        if index % image_every or message.encoding != "rgb8":
            continue
        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3)
        path = os.path.join(args.image_dir, f"{index:06d}.png")
        cv2.imwrite(path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        distance = float(cv2.compareHist(reference, hist_of(cv2.imread(path)),
                                         cv2.HISTCMP_BHATTACHARYYA))
        visible.append((wall, distance < args.hist_threshold))

    in_view = out_view = in_view_nonempty = out_view_nonempty = 0
    for row in det:
        nearest = min(visible, key=lambda item: abs(item[0] - row[1]), default=None)
        if nearest is None or abs(nearest[0] - row[1]) > 0.5:
            continue
        if nearest[1]:
            in_view += 1
            in_view_nonempty += len(row[2].detections) > 0
        else:
            out_view += 1
            out_view_nonempty += len(row[2].detections) > 0

    center_stats = {}
    for label, points in centers.items():
        center_stats[label] = {
            "n": len(points),
            "std_x": round(statistics.pstdev([p[0] for p in points]), 3) if len(points) > 1 else None,
            "std_y": round(statistics.pstdev([p[1] for p in points]), 3) if len(points) > 1 else None,
        }

    windows = rtf_windows(clocks)
    return {
        "bag": os.path.abspath(args.bag),
        "span_s": round(span, 3),
        "messages": {topic: len(rows) for topic, rows in by_topic.items()},
        "image_hz": round(len(images) / span, 3),
        "detections_hz": round(len(det) / span, 3),
        "nonempty_detections": len(nonempty),
        "nonempty_ratio": round(len(nonempty) / len(det), 4),
        "detection_gaps_gt_200ms": gaps(det_times, 0.2),
        "labels": labels,
        "bbox_center_stats": center_stats,
        "cmd_vel": {
            "hz": round(len(cmd) / span, 3),
            "linear_jumps_gt_0.3": lin_jumps,
            "angular_jumps_gt_0.5": ang_jumps,
            "max_abs_dlinear": round(max_lin, 4),
            "max_abs_dangular": round(max_ang, 4),
        },
        "rtf_20s_windows": windows,
        "rtf_median": round(statistics.median(windows), 3) if windows else None,
        "visibility": {
            "reference": args.reference,
            "hist_threshold": args.hist_threshold,
            "sampled_frames": len(visible),
            "visible_frames": sum(1 for _, is_visible in visible if is_visible),
            "detections_in_view": in_view,
            "nonempty_in_view": in_view_nonempty,
            "continuity_in_view": (round(in_view_nonempty / in_view, 4) if in_view else None),
            "detections_out_of_view": out_view,
            "nonempty_out_of_view": out_view_nonempty,
            "false_positive_out_of_view": (round(out_view_nonempty / out_view, 4)
                                           if out_view else None),
        },
        "odom_path_m": round(math.fsum(
            math.hypot(b[2].pose.pose.position.x - a[2].pose.pose.position.x,
                       b[2].pose.pose.position.y - a[2].pose.pose.position.y)
            for a, b in zip(odom, odom[1:])), 3),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", default="/tmp/bag_summary.json")
    parser.add_argument("--reference",
                        default=os.path.join(os.path.dirname(__file__), "bus.jpg"))
    parser.add_argument("--hist-threshold", type=float, default=0.65)
    parser.add_argument("--image-dir", default="/tmp/drive_bag_images")
    parser.add_argument("--image-every", type=int, default=5)
    args = parser.parse_args()
    summary = summarise(args, read_bag(args.bag))
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
