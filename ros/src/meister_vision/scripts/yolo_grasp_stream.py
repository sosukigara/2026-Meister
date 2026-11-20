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
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

IMG_SIZE = (640, 640)          # モデル入力 (w, h)
OBJ_THRES = 0.45          # 既定。--conf で変えられる
NMS_THRES = 0.45
MAX_DET = 30


def load_labels(path: str) -> List[str]:
    with open(path, encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip()]


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def anchor_decode(reg: np.ndarray, cls: np.ndarray, logit_thres: float):
    """スコアで足切りしてから DFL を計算する。

    全アンカー（80x80+40x40+20x20 = 8400 本）で DFL を回すと、exp と softmax だけで
    数十 ms かかる。sigmoid は単調なので生ロジットで先に足切りすれば、
    実際に DFL が要るのは数〜数十本になる。
    """
    score_map = cls[0].max(axis=0)                 # (h, w) 生ロジット
    ys, xs = np.nonzero(score_map > logit_thres)
    if len(ys) == 0:
        return None
    grid_h, grid_w = score_map.shape
    stride_x, stride_y = IMG_SIZE[0] / grid_w, IMG_SIZE[1] / grid_h

    sel = reg[0][:, ys, xs]                        # (64, N)
    v = sel.reshape(4, 16, -1)
    v = v - v.max(axis=1, keepdims=True)
    np.exp(v, out=v)
    v /= v.sum(axis=1, keepdims=True)
    acc = np.arange(16, dtype=np.float32).reshape(1, 16, 1)
    dist = (v * acc).sum(axis=1)                   # (4, N)

    cx = (xs + 0.5 - dist[0]) * stride_x
    cy = (ys + 0.5 - dist[1]) * stride_y
    cx2 = (xs + 0.5 + dist[2]) * stride_x
    cy2 = (ys + 0.5 + dist[3]) * stride_y
    boxes = np.stack((cx, cy, cx2, cy2), axis=1).astype(np.float32)

    probs = cls[0][:, ys, xs].T                    # (N, 80)
    probs = 1.0 / (1.0 + np.exp(-probs))
    class_ids = probs.argmax(axis=1)
    scores = probs[np.arange(len(class_ids)), class_ids]
    return boxes, class_ids, scores


def post_process(outputs: List[np.ndarray], obj_thres: float):
    """3 スケール (reg, cls, sum) をまとめて [N,4], [N], [N] にする。"""
    # sigmoid(x) > t  <=>  x > log(t/(1-t))
    logit_thres = math.log(obj_thres / (1.0 - obj_thres))
    boxes, class_ids, scores = [], [], []
    per = len(outputs) // 3
    for i in range(3):
        decoded = anchor_decode(outputs[per * i], outputs[per * i + 1], logit_thres)
        if decoded is None:
            continue
        boxes.append(decoded[0])
        class_ids.append(decoded[1])
        scores.append(decoded[2])
    if not boxes:
        return (np.zeros((0, 4), np.float32), np.zeros((0,), np.int32),
                np.zeros((0,), np.float32))

    boxes = np.concatenate(boxes)
    class_ids = np.concatenate(class_ids)
    scores = np.concatenate(scores)

    xywh = np.concatenate(((boxes[:, 0:2] + boxes[:, 2:4]) / 2,
                           boxes[:, 2:4] - boxes[:, 0:2]), axis=1).tolist()
    ids = cv2.dnn.NMSBoxes(xywh, scores.tolist(), obj_thres, NMS_THRES)
    ids = np.array(ids).flatten()[:MAX_DET]
    return boxes[ids], class_ids[ids], scores[ids]


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
                 capture_w: int, capture_h: int, conf: float,
                 display_fps: float) -> None:
        super().__init__("yolo_grasp")
        from rknnlite.api import RKNNLite

        self._labels = labels
        self._conf = conf
        self._display_period = 1.0 / display_fps if display_fps > 0 else 0.0
        self._grasp = GraspTarget()
        # 既存の Web UI とアーム制御はこの 2 本を見ている。NPU 実装でも同じ名前に
        # 出すことで、ONNX 版と差し替え可能にする。
        self._pub = self.create_publisher(Image, topic, 1)
        self._pub_detections = self.create_publisher(Detection2DArray, "detections", 10)
        self._pub_image = self.create_publisher(Image, "detection_image", 10)
        # アームは最新の 1 件だけ見る。depth=1 で古い指令を捨てる。
        self._pub_target = self.create_publisher(
            Detection2DArray, "target",
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE))
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
        self._last_publish = 0.0
        self._read_failures = 0
        self._capture_w = capture_w
        self._capture_h = capture_h
        self._stop = False
        self.get_logger().info(f"入力 {IMG_SIZE} / ラベル {len(labels)} 件 / {device}")

    def stop(self) -> None:
        self._stop = True
        self._cap.release()

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
                self._read_failures += 1
                if self._read_failures % 50 == 0:
                    self.get_logger().warn("カメラを開き直します")
                    self._cap.release()
                    self._cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
                    self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._capture_w)
                    self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._capture_h)
                    self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(0.005)
                continue
            self._read_failures = 0
            with self._lock:
                self._latest = frame
                self._has_frame = True
            now = time.monotonic()
            if now - self._last_publish < self._display_period:
                continue  # 配信は上限レートに抑える（カメラは読み続けて最新を保つ）
            self._last_publish = now
            canvas = self._compose(frame)
            stamp = self.get_clock().now().to_msg()
            self._publish(canvas, stamp)
            self._publish_annotated(canvas, stamp)
            with self._lock:
                detections = list(self._detections)
            self._publish_detections(stamp, detections)
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
        boxes, class_ids, scores = post_process(outputs, self._conf)

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

    def _publish_detections(self, stamp, detections: List[dict]) -> None:
        message = Detection2DArray()
        message.header.stamp = stamp
        message.header.frame_id = "camera_link"
        for index, det in enumerate(detections):
            box = Detection2D()
            x0, y0, x1, y1 = det["box"]
            box.bbox.center.position.x = float((x0 + x1) / 2)
            box.bbox.center.position.y = float((y0 + y1) / 2)
            box.bbox.size_x = float(x1 - x0)
            box.bbox.size_y = float(y1 - y0)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det["label"]
            hypothesis.hypothesis.score = det["score"]
            box.results.append(hypothesis)
            message.detections.append(box)
        self._pub_detections.publish(message)
        # 先頭（最も手前）だけをアーム向けに流す
        if detections:
            target = Detection2DArray()
            target.header = message.header
            target.detections = [message.detections[0]]
            self._pub_target.publish(target)

    def _publish_annotated(self, canvas: np.ndarray, stamp) -> None:
        self._pub_image.publish(self._to_msg(canvas, stamp))

    def _to_msg(self, canvas: np.ndarray, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = canvas.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = canvas.shape[1] * 3
        msg.data = canvas.tobytes()
        return msg

    def _publish(self, canvas: np.ndarray, stamp) -> None:
        self._pub.publish(self._to_msg(canvas, stamp))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="tools/rknn/models/yolov8n-zoo.rknn")
    parser.add_argument("--labels", default="tools/grasp/coco_labels.txt")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--topic", default="/grasp_image")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--conf", type=float, default=OBJ_THRES,
                        help="信頼度しきい値。誤検出が多いときは上げる")
    parser.add_argument("--display-fps", type=float, default=15.0,
                        help="配信レート上限。CPU を下げたいときは下げる")
    # launch から起動されると --ros-args -r __node:=... が後ろに付く。
    # 自前の引数だけ解釈し、ROS 用は rclpy.init に渡す。
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        node = GraspNode(args.model, load_labels(args.labels), args.device,
                         args.topic, args.width, args.height, args.conf,
                         args.display_fps)
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - 起動失敗はそのまま出して終わる
        print(f"起動に失敗: {exc}")
        return 1
    finally:
        if node is not None:
            node.stop()
        # SIGTERM では rclpy 側が先に shutdown している。二重に呼ぶと例外になる。
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
