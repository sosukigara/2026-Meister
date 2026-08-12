"""ROS2 物体検出ノード。

画像トピック (既定: image_raw) を購読し、YOLOv8n ONNX で物体検出して
vision_msgs/Detection2DArray を配信する。オプションで検出枠を描画した
画像 (detection_image) も配信する。

レート制限: 既定で 10 Hz に制限し、負荷を軽く保つ。

パラメータ:
  model_path         (str,  既定: "")   モデルパス。空なら自動解決
  conf_threshold     (float, 既定: 0.25) 信頼度しきい値
  iou_threshold      (float, 既定: 0.45) NMS の IoU しきい値
  image_topic        (str,  既定: "image_raw") 購読する画像トピック
  publish_annotated  (bool, 既定: true) 描画済み画像を配信するか
  rate               (float, 既定: 10.0) 最大処理レート [Hz]

トピック:
  購読:  image_topic         (sensor_msgs/Image)
  配信:  detections          (vision_msgs/Detection2DArray)
  配信:  detection_image     (sensor_msgs/Image, 描画済み)
"""
from __future__ import annotations

import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesis, ObjectHypothesisWithPose)

from .yolo_detector import YOLODetector

# cv_bridge は numpy 1.x 向けにビルドされている。numpy 2.x 環境では
# import 自体は通るものの変換処理が実行時に SIGSEGV を起こすため、
# numpy メジャーバージョンに応じて手動変換へフォールバックする。
_NUMPY_MAJOR = int(np.__version__.split(".")[0])
if _NUMPY_MAJOR < 2:
    try:
        from cv_bridge import CvBridge
        _BRIDGE = CvBridge()
        _HAS_CV_BRIDGE = True
    except Exception:  # noqa: BLE001
        _BRIDGE = None
        _HAS_CV_BRIDGE = False
else:
    _BRIDGE = None
    _HAS_CV_BRIDGE = False

# 手動変換用のエンコーディング → (dtype, channels) マップ
_ENCODING_INFO = {
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "bgra8": (np.uint8, 4),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
    "8UC3": (np.uint8, 3),
}

_BOX_COLOR = (0, 255, 0)  # BGR: 緑
_TEXT_COLOR = (255, 255, 255)
_TEXT_BG = (0, 0, 0)
_TEXT_THICKNESS = 1
_BOX_THICKNESS = 2


def _bgr_from_image_msg(msg: ImageMsg) -> np.ndarray:
    """sensor_msgs/Image を BGR ndarray に変換する (cv_bridge または手動)。"""
    if _HAS_CV_BRIDGE:
        return _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    encoding = msg.encoding
    if encoding in _ENCODING_INFO:
        dtype, channels = _ENCODING_INFO[encoding]
    elif encoding.endswith("8"):
        dtype, channels = np.uint8, 1
    elif encoding.endswith("16"):
        dtype, channels = np.uint16, 1
    elif encoding.endswith("32F"):
        dtype, channels = np.float32, 1
    else:
        raise ValueError(f"未対応のエンコーディング: {encoding}")

    arr = np.frombuffer(msg.data, dtype=dtype).reshape(
        msg.height, msg.width, -1)
    if channels == 1:
        arr = arr[:, :, 0]
    if encoding == "rgb8":
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    elif encoding == "mono8":
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    elif encoding == "rgba8" or encoding == "bgra8":
        arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR
                           if encoding == "rgba8" else cv2.COLOR_BGRA2BGR)
    return arr


class DetectionNode(Node):
    """画像を購読して物体検出結果を配信するノード。"""

    def __init__(self) -> None:
        super().__init__("meister_vision")

        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("rate", 10.0)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value or None
        conf_threshold = self.get_parameter("conf_threshold").value
        iou_threshold = self.get_parameter("iou_threshold").value
        image_topic = self.get_parameter("image_topic").value
        self._publish_annotated = self.get_parameter("publish_annotated").value
        self._min_interval = 1.0 / float(self.get_parameter("rate").value)

        self._detector = YOLODetector(
            model_path=model_path,
            conf_threshold=float(conf_threshold),
            iou_threshold=float(iou_threshold),
        )
        self.get_logger().info(
            f"モデルを読み込みました: {self._detector.model_path}")
        bridge_name = "cv_bridge" if _HAS_CV_BRIDGE else "numpy (手動変換)"
        self.get_logger().info(f"画像変換に {bridge_name} を使用します")

        self._pub_detections = self.create_publisher(
            Detection2DArray, "detections", 10)
        self._pub_annotated = self.create_publisher(
            ImageMsg, "detection_image", 10)
        self._sub_image = self.create_subscription(
            ImageMsg, image_topic, self._image_callback, 10)
        self.get_logger().info(
            f"画像トピック '{image_topic}' を購読開始 (最大 "
            f"{self.get_parameter('rate').value} Hz)")

        self._last_process_time = 0.0
        self._process_count = 0

    def _build_detection_array(
        self, msg: ImageMsg, detections: List,
    ) -> Detection2DArray:
        """Detection リストを vision_msgs/Detection2DArray に変換する。"""
        array = Detection2DArray()
        array.header = msg.header
        for det in detections:
            d2d = Detection2D()
            d2d.header = msg.header
            d2d.id = det.class_name  # 読みやすいようクラス名を ID として使う
            hypothesis = ObjectHypothesis()
            hypothesis.class_id = str(det.class_id)
            hypothesis.score = float(det.confidence)
            hyp_with_pose = ObjectHypothesisWithPose()
            hyp_with_pose.hypothesis = hypothesis
            d2d.results.append(hyp_with_pose)

            x1, y1, x2, y2 = det.xyxy
            d2d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d2d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d2d.bbox.size_x = float(x2 - x1)
            d2d.bbox.size_y = float(y2 - y1)
            array.detections.append(d2d)
        return array

    def _draw_detections(self, bgr: np.ndarray, detections: List) -> np.ndarray:
        """検出枠とラベルを描画した画像を返す。"""
        annotated = bgr.copy()
        for det in detections:
            x1, y1, x2, y2 = (int(v) for v in det.xyxy)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), _BOX_COLOR,
                          _BOX_THICKNESS)
            label = f"{det.class_name} {det.confidence:.2f}"
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, _TEXT_THICKNESS)
            top = max(y1 - text_h - baseline - 4, 0)
            cv2.rectangle(annotated, (x1, top),
                          (x1 + text_w + 4, y1 - baseline), _TEXT_BG, -1)
            cv2.putText(annotated, label, (x1 + 2, y1 - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, _TEXT_COLOR,
                        _TEXT_THICKNESS, cv2.LINE_AA)
        return annotated

    def _image_callback(self, msg: ImageMsg) -> None:
        # レート制限: 前回処理から min_interval 未満ならスキップ
        now = time.monotonic()
        if now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now
        self._process_count += 1

        try:
            bgr = _bgr_from_image_msg(msg)
            detections = self._detector.detect(bgr)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"検出処理に失敗しました: {exc}")
            return

        array = self._build_detection_array(msg, detections)
        self._pub_detections.publish(array)

        if self._publish_annotated:
            annotated = self._draw_detections(bgr, detections)
            if _HAS_CV_BRIDGE:
                annotated_msg = _BRIDGE.cv2_to_imgmsg(annotated, encoding="bgr8")
            else:
                annotated_msg = self._numpy_to_image_msg(annotated, msg)
            annotated_msg.header = msg.header
            self._pub_annotated.publish(annotated_msg)

        self.get_logger().debug(
            f"[{self._process_count}] {len(detections)} 件検出")

    @staticmethod
    def _numpy_to_image_msg(arr: np.ndarray, ref: ImageMsg) -> ImageMsg:
        """BGR ndarray を sensor_msgs/Image に変換する (cv_bridge フォールバック)。"""
        msg = ImageMsg()
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = arr.shape[1] * 3
        msg.data = np.ascontiguousarray(arr).tobytes()
        return msg


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
