"""Record live /detections messages and summarise non-empty results.

    python3 docs/perf/capture_detections_live.py --seconds 20 \
        --out docs/perf/logs/live_detections_20s.txt

The raw log keeps full message excerpts so class names and numeric IDs can be
checked without trusting a summary.
"""
import argparse
import time

import rclpy
from vision_msgs.msg import Detection2DArray


def describe(detection):
    hypothesis = detection.results[0].hypothesis if detection.results else None
    return (f"id={detection.id!r} "
            f"class_id={hypothesis.class_id!r} score={hypothesis.score:.3f}"
            if hypothesis else f"id={detection.id!r} NO_HYPOTHESIS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--out", default="/tmp/live_detections.txt")
    parser.add_argument("--excerpts", type=int, default=3)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("live_detection_probe")
    state = {"received": 0, "nonempty": 0, "detections": 0, "excerpts": []}
    labels = {}

    def on_message(message):
        state["received"] += 1
        count = len(message.detections)
        if count:
            state["nonempty"] += 1
            state["detections"] += count
            for detection in message.detections:
                key = describe(detection)
                labels[key] = labels.get(key, 0) + 1
            if len(state["excerpts"]) < args.excerpts:
                state["excerpts"].append(str(message))

    node.create_subscription(Detection2DArray, "/detections", on_message, 10)
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.monotonic() - started

    lines = [
        f"window_wall_s={elapsed:.3f}",
        f"messages={state['received']} hz={state['received'] / elapsed:.3f}",
        f"nonempty_messages={state['nonempty']} "
        f"nonempty_hz={state['nonempty'] / elapsed:.3f}",
        f"detection_count_total={state['detections']}",
    ]
    for key, count in sorted(labels.items()):
        lines.append(f"label[{key}] count={count}")
    lines.append("--- raw message excerpts ---")
    lines.extend(state["excerpts"] or ["NO MESSAGE"])
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    print("\n".join(lines[:8]), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
