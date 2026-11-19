#!/usr/bin/env python3
"""`/detections` を指定秒数だけ購読し、フレームごとの検出を JSON に保存する。

画像認識の A/B（同じ録画を 2 つのモデル設定に流して比べる）で使う。
`detection_node` は購読者が 0 の間は推論しないので、この記録器が
購読者になることで推論が動く。

使い方:
    python3 record_detections.py --seconds 26 --out /tmp/ab_640.json
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray


class DetectionRecorder(Node):
    def __init__(self) -> None:
        super().__init__("detection_recorder")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.frames: list[dict] = []
        self.create_subscription(Detection2DArray, "/detections", self.on_msg, qos)

    def on_msg(self, msg: Detection2DArray) -> None:
        items = []
        for det in msg.detections:
            if det.results:
                label = det.results[0].hypothesis.class_id
                score = float(det.results[0].hypothesis.score)
            else:
                label, score = "", 0.0
            items.append({
                "class": label,
                "score": score,
                "cx": float(det.bbox.center.position.x),
                "cy": float(det.bbox.center.position.y),
                "w": float(det.bbox.size_x),
                "h": float(det.bbox.size_y),
            })
        self.frames.append({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "detections": items,
        })


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=25.0)
    parser.add_argument("--out", required=True, help="保存先 JSON")
    args = parser.parse_args()

    rclpy.init()
    node = DetectionRecorder()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    with open(args.out, "w") as f:
        json.dump(node.frames, f)
    print(f"frames={len(node.frames)} -> {args.out}")


if __name__ == "__main__":
    main()
