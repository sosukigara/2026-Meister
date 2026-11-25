"""Summarise a recorded drive session (raw JSONL from record_stable_drive.py).

    python3 docs/perf/summarize_stable_drive.py \
        --jsonl docs/perf/logs/drive_run.jsonl \
        --out docs/perf/logs/drive_summary.json

Visibility is judged from sampled *images* with an HSV histogram against
docs/perf/bus.jpg, never from detector output, so detection continuity is not
measured circularly.  The threshold is reported with the results.
"""
import argparse
import json
import math
import os
import statistics

import cv2
import numpy as np


def load(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def hist_of(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [18, 16], [0, 180, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def rate(times, span):
    return len(times) / span if span > 0 else 0.0


def gaps(times, limit):
    result = []
    for previous, current in zip(times, times[1:]):
        delta = current - previous
        if delta > limit:
            result.append(round(delta, 3))
    return result


def rtf_windows(clock_rows, window_s=20.0):
    if len(clock_rows) < 2:
        return []
    base = clock_rows[0]["wall"]
    end = clock_rows[-1]["wall"]
    result = []
    start = base
    while start + window_s <= end:
        inside = [row for row in clock_rows
                  if start <= row["wall"] <= start + window_s]
        if len(inside) >= 2:
            sim = (inside[-1]["data"]["sec"] + inside[-1]["data"]["nanosec"] / 1e9
                   - inside[0]["data"]["sec"] - inside[0]["data"]["nanosec"] / 1e9)
            wall = inside[-1]["wall"] - inside[0]["wall"]
            if wall > 0:
                result.append(round(sim / wall, 3))
        start += window_s
    return result


def summarise(args, rows):
    by_topic = {}
    for row in rows:
        by_topic.setdefault(row["topic"], []).append(row)
    detections = by_topic.get("detections", [])
    images = by_topic.get("image", [])
    cmd_vel = by_topic.get("cmd_vel", [])
    clocks = by_topic.get("clock", [])
    odom = by_topic.get("odom", [])
    if not detections or not clocks:
        raise RuntimeError("detections / clock がありません。 Recording が成功したか確認してください")

    span = max(row["wall"] for row in rows) - min(row["wall"] for row in rows)
    det_times = [row["wall"] for row in detections]
    image_times = [row["wall"] for row in images]
    cmd_times = [row["wall"] for row in cmd_vel]
    nonempty = [row for row in detections if row["data"]["n"] > 0]
    labels = {}
    centers = {}
    for row in nonempty:
        for item in row["data"]["items"]:
            labels[item["id"]] = labels.get(item["id"], 0) + 1
            centers.setdefault(item["id"], []).append((item["cx"], item["cy"]))

    lin_jumps = 0
    ang_jumps = 0
    max_lin = 0.0
    max_ang = 0.0
    for previous, current in zip(cmd_vel, cmd_vel[1:]):
        dlin = abs(current["data"]["linear_x"] - previous["data"]["linear_x"])
        dang = abs(current["data"]["angular_z"] - previous["data"]["angular_z"])
        max_lin = max(max_lin, dlin)
        max_ang = max(max_ang, dang)
        lin_jumps += dlin > 0.3
        ang_jumps += dang > 0.5

    # Independent target-visibility oracle: sampled image vs. the texture reference.
    reference = hist_of(cv2.imread(args.reference, cv2.IMREAD_COLOR))
    saved = [row for row in images if row["data"].get("saved")]
    visible_frames = []
    distances = []
    for row in saved:
        image = cv2.imread(row["data"]["saved"], cv2.IMREAD_COLOR)
        if image is None:
            continue
        distance = float(cv2.compareHist(reference, hist_of(image),
                                         cv2.HISTCMP_BHATTACHARYYA))
        distances.append(distance)
        visible_frames.append((row["wall"], distance < args.hist_threshold))

    visible_detections = 0
    visible_nonempty = 0
    hidden_detections = 0
    hidden_nonempty = 0
    for row in detections:
        nearest = min(visible_frames, key=lambda f: abs(f[0] - row["wall"]), default=None)
        if nearest is None or abs(nearest[0] - row["wall"]) > 0.5:
            continue
        if nearest[1]:
            visible_detections += 1
            visible_nonempty += row["data"]["n"] > 0
        else:
            hidden_detections += 1
            hidden_nonempty += row["data"]["n"] > 0

    center_stats = {}
    for label, points in centers.items():
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        center_stats[label] = {
            "n": len(points),
            "std_x": round(statistics.pstdev(xs), 3) if len(xs) > 1 else None,
            "std_y": round(statistics.pstdev(ys), 3) if len(ys) > 1 else None,
            "mean_x": round(statistics.fmean(xs), 3),
            "mean_y": round(statistics.fmean(ys), 3),
        }

    windows = rtf_windows(clocks)
    summary = {
        "span_s": round(span, 3),
        "messages": {topic: len(rows) for topic, rows in by_topic.items()},
        "image_hz": round(rate(image_times, span), 3),
        "detections_hz": round(rate(det_times, span), 3),
        "nonempty_detections": len(nonempty),
        "nonempty_ratio": round(len(nonempty) / len(detections), 4),
        "detection_gaps_gt_200ms": gaps(det_times, 0.2),
        "labels": labels,
        "bbox_center_stats": center_stats,
        "cmd_vel": {
            "hz": round(rate(cmd_times, span), 3),
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
            "sampled_frames": len(distances),
            "visible_frames": sum(1 for _, visible in visible_frames if visible),
            "hist_distance_min": round(min(distances), 4) if distances else None,
            "hist_distance_median": round(statistics.median(distances), 4) if distances else None,
            "hist_distance_max": round(max(distances), 4) if distances else None,
            "detections_in_view": visible_detections,
            "nonempty_in_view": visible_nonempty,
            "continuity_in_view": (round(visible_nonempty / visible_detections, 4)
                                   if visible_detections else None),
            "detections_out_of_view": hidden_detections,
            "nonempty_out_of_view": hidden_nonempty,
            "false_positive_out_of_view": (round(hidden_nonempty / hidden_detections, 4)
                                           if hidden_detections else None),
        },
        "odom_path_m": round(math.fsum(
            math.hypot(b["data"]["x"] - a["data"]["x"], b["data"]["y"] - a["data"]["y"])
            for a, b in zip(odom, odom[1:])), 3),
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--jsonl", required=True)
    parser.add_argument("--reference",
                        default=os.path.join(os.path.dirname(__file__), "bus.jpg"))
    parser.add_argument("--hist-threshold", type=float, default=0.35)
    parser.add_argument("--out", default="/tmp/drive_summary.json")
    args = parser.parse_args()
    summary = summarise(args, load(args.jsonl))
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
