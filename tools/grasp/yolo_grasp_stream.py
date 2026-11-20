#!/usr/bin/env python3
"""yolov8n (RKNN, NPU) で物体名と枠を出し、枠の中を OpenCV で輪郭解析して
把持候補の中心・角度・幅を求める。結果を /grasp_image に配信して RViz で見る。

  tools/grasp/yolo_grasp_stream.py --model tools/rknn/models/yolov8n-zoo.rknn

推論は NPU（実測 20.7 ms / 48 fps）。表示はカメラレートで回し、
推論結果は最新のものを使う（古いフレームは捨てる）。
"""
from __future__ import annotations

import argparse
import math
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

IMG_SIZE = (640, 640)          # モデル入力 (w, h)
OBJ_THRES = 0.25
NMS_THRES = 0.45
MAX_DET = 30


def load_labels(path: str) -> List[str]:
    with open(path, encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip()]


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def dfl(position: np.ndarray) -> np.ndarray:
    """DFL の期待値。torch を使うと毎フレーム数十 ms かかるので numpy で書く。"""
    n, c, h, w = position.shape
    p_num = 4
    mc = c // p_num
    y = position.reshape(n, p_num, mc, h, w)
    y = np.exp(y - y.max(axis=2, keepdims=True))
    y /= y.sum(axis=2, keepdims=True)
    acc = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
    return (y * acc).sum(axis=2)


def box_process(position: np.ndarray) -> np.ndarray:
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
    grid = np.stack((col, row), axis=0).reshape(1, 2, grid_h, grid_w).astype(np.float32)
    stride = np.array([IMG_SIZE[1] // grid_h, IMG_SIZE[0] // grid_w],
                      dtype=np.float32).reshape(1, 2, 1, 1)
    p = dfl(position)
    xy = grid + 0.5 - p[:, 0:2]
    xy2 = grid + 0.5 + p[:, 2:4]
    return np.concatenate((xy * stride, xy2 * stride), axis=1)


def post_process(outputs: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """3 スケール (reg, cls, sum) × 3 の出力を [N,4], [N,80], [N] へ。"""
    boxes, classes, scores = [], [], []
    all_boxes, all_cls = [], []
    branch = 3
    per = len(outputs) // branch
    for i in range(branch):
        all_boxes.append(box_process(outputs[per * i]))
        all_cls.append(sigmoid(outputs[per * i + 1]))
    for b, c in zip(all_boxes, all_cls):
        b = b.transpose(0, 2, 3, 1).reshape(-1, 4)
        c = c.transpose(0, 2, 3, 1).reshape(-1, c.shape[1])
        boxes.append(b)
        classes.append(c)
        scores.append(c.max(axis=1))

    boxes = np.concatenate(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    keep = scores > OBJ_THRES
    boxes, classes, scores = boxes[keep], classes[keep], scores[keep]
    if len(boxes) == 0:
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.int32), np.zeros((0,), np.float32)

    xywh = np.concatenate(
        ((boxes[:, 0:2] + boxes[:, 2:4]) / 2,
         boxes[:, 2:4] - boxes[:, 0:2]), axis=1).tolist()
    ids = cv2.dnn.NMSBoxes(xywh, scores.tolist(), OBJ_THRES, NMS_THRES)
    ids = np.array(ids).flatten()[:MAX_DET]
    return boxes[ids], classes[ids].argmax(axis=1), scores[ids]


class GraspTarget:
    """枠の内側を二値化して輪郭から中心・角度・幅を出す（1 ms 程度）。"""

    def __init__(self, min_area_ratio: float = 0.15) -> None:
        self.min_area_ratio = min_area_ratio

    def compute(self, image_bgr: np.ndarray,
                box: Tuple[int, int, int, int]) -> Optional[dict]:
        x0, y0, x1, y1 = box
        x0, y0 = max(0, x0), max(0, y0)
        x1 = min(image_bgr.shape[1] - 1, x1)
        y1 = min(image_bgr.shape[0] - 1, y1)
        if x1 - x0 < 12 or y1 - y0 < 12:
            return None
        roi = image_bgr[y0:y1, x0:x1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        roi_area = float(roi.shape[0] * roi.shape[1])
        best = None
        for contour in contours:
            area = cv2.contourArea(contour)
            # 枠いっぱいの背景と、小さすぎるノイズを落とす
            if area < roi_area * self.min_area_ratio or area > roi_area * 0.95:
                continue
            if best is None or area > best[0]:
                best = (area, contour)
        if best is None:
            return None
        rect = cv2.minAreaRect(best[1])
        (cx, cy), (w, h), angle = rect
        # OpenCV の角度は (-90, 0]。長辺を「指を開く向き」に揃える。
        if w < h:
            w, h = h, w
            angle += 90.0
        if angle > 90.0:
            angle -= 180.0
        return {
            "center": (int(x0 + cx), int(y0 + cy)),
            "size": (float(w), float(h)),
            "angle_deg": float(angle),
            "rect": ((float(x0 + cx), float(y0 + cy)), (float(w), float(h)), float(angle)),
        }


class GraspNode(Node):
    def __init__(self, model: str, labels: List[str], device: str, topic: str,
                 capture_w: int, capture_h: int) -> None:
        super().__init__("yolo_grasp")
        from rknnlite.api import RKNNLite

        self._labels = labels
        self._grasp = GraspTarget()
        self._pub = self.create_publisher(Image, topic, 1)
        self._device = device
        self._topic = topic

        self._rknn = RKNNLite()
        if self._rknn.load_rknn(model) != 0:
            raise RuntimeError(f"モデルを読めません: {model}")
        if self._rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
            raise RuntimeError("NPU の初期化に失敗")

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, capture_w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_h)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._cap.isOpened():
            raise RuntimeError(f"カメラを開けません: {device}")

        self._lock = threading.Lock()
        self._has_frame = False
        self._latest = None
        self._mask: Optional[np.ndarray] = None
        self._detections: List[dict] = []
        self._infer_ms = 0.0
        self._frames = 0
        self._stop = False
        self.get_logger().info(f"入力 {IMG_SIZE} / ラベル {len(labels)} 件 / {device}")

    def run(self) -> None:
        infer = threading.Thread(target=self._infer_loop, daemon=True)
        infer.start()
        try:
            self._capture_loop()
        finally:
            self._stop = True
            infer.join(timeout=1.0)
            self._cap.release()

    def _capture_loop(self) -> None:
        while rclpy.ok() and not self._stop:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                continue
            with self._lock:
                self._latest = frame
                self._has_frame = True
            canvas = self._compose(frame)
            self._publish(canvas)
            self._frames += 1
            if self._frames % 60 == 0:
                self.get_logger().info(
                    f"表示 {self._frames} フレーム  推論 {self._infer_ms:.1f} ms  "
                    f"検出 {len(self._detections)}")

    def _infer_loop(self) -> None:
        while rclpy.ok() and not self._stop:
            with self._lock:
                if not self._has_frame:
                    frame = None
                else:
                    frame = self._latest
                    self._has_frame = False
            if frame is None:
                time.sleep(0.002)
                continue
            t0 = time.perf_counter()
            detections = self._detect(frame)
            self._infer_ms = (time.perf_counter() - t0) * 1000.0
            with self._lock:
                self._detections = detections

    def _detect(self, frame: np.ndarray) -> List[dict]:
        # 縦横比を保って 640x640 に収める（潰すと精度が落ちる）
        h, w = frame.shape[:2]
        scale = min(IMG_SIZE[0] / w, IMG_SIZE[1] / h)
        nw, nh = int(w * scale), int(h * scale)
        canvas = np.full((IMG_SIZE[1], IMG_SIZE[0], 3), 114, np.uint8)
        dx, dy = (IMG_SIZE[0] - nw) // 2, (IMG_SIZE[1] - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(frame, (nw, nh))
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)[None, ...]

        outputs = self._rknn.inference(inputs=[rgb], data_format="nhwc")
        boxes, class_ids, scores = post_process(outputs)

        results = []
        for box, cid, score in zip(boxes, class_ids, scores):
            # パディングを外して元画像座標へ
            x0 = int((box[0] - dx) / scale)
            y0 = int((box[1] - dy) / scale)
            x1 = int((box[2] - dx) / scale)
            y1 = int((box[3] - dy) / scale)
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(w - 1, x1), min(h - 1, y1)
            if x1 - x0 < 8 or y1 - y0 < 8:
                continue
            grasp = self._grasp.compute(frame, (x0, y0, x1, y1))
            results.append({
                "label": self._labels[int(cid)] if int(cid) < len(self._labels) else str(cid),
                "score": float(score),
                "box": (x0, y0, x1, y1),
                "grasp": grasp,
            })
        # 枠が大きいものほど手前にあるとみなして把持候補の先頭に置く
        results.sort(key=lambda r: (r["box"][2] - r["box"][0]) * (r["box"][3] - r["box"][1]),
                     reverse=True)
        return results

    def _compose(self, frame: np.ndarray) -> np.ndarray:
        with self._lock:
            detections = list(self._detections)
            infer_ms = self._infer_ms
        canvas = frame.copy()
        for i, det in enumerate(detections):
            x0, y0, x1, y1 = det["box"]
            is_target = i == 0
            color = (0, 255, 0) if is_target else (200, 200, 200)
            cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 2 if is_target else 1)
            cv2.putText(canvas, f"{det['label']} {det['score']:.2f}",
                        (x0, max(14, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            grasp = det["grasp"]
            if grasp is None:
                continue
            rect = grasp["rect"]
            pts = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(canvas, [pts], 0, (0, 255, 255), 2)
            cx, cy = grasp["center"]
            cv2.circle(canvas, (cx, cy), 5, (0, 255, 255), -1)
            if is_target:
                width, _height = grasp["size"]
                cv2.putText(canvas,
                            f"TARGET {det['label']} ({cx},{cy}) "
                            f"angle {grasp['angle_deg']:.1f} deg  width {width:.0f}px",
                            (8, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 2)
        cv2.putText(canvas, f"infer {infer_ms:.1f} ms  det {len(detections)}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return canvas

    def _publish(self, canvas: np.ndarray) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = canvas.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = canvas.shape[1] * 3
        msg.data = canvas.tobytes()
        self._pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="tools/rknn/models/yolov8n-zoo.rknn")
    parser.add_argument("--labels", default="tools/grasp/coco_labels.txt")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--topic", default="/grasp_image")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    rclpy.init()
    try:
        node = GraspNode(args.model, load_labels(args.labels), args.device,
                         args.topic, args.width, args.height)
        node.run()
    except Exception as exc:  # noqa: BLE001 - 起動失敗はそのまま出して終わる
        print(f"起動に失敗: {exc}")
        rclpy.shutdown()
        return 1
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
