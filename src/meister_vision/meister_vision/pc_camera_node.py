"""PC内蔵カメラ配信用ノード。"""
from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg


class PcCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("pc_camera")

        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "camera_link")

        fps = float(self.get_parameter("fps").value)
        period = 1.0 / fps if fps > 0.0 else 1.0 / 15.0
        self._cap: cv2.VideoCapture | None = None
        self._pub = self.create_publisher(
            ImageMsg, "/camera/image_raw", 10)
        self._timer = self.create_timer(period, self._on_timer)

    def _open_device(self) -> bool:
        device = self.get_parameter("device").value
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        # VideoCaptureは数値インデックスとデバイスパスの両方を受け付けるため
        # 数字文字列は整数に寄せてバックエンド選択の誤動作を避ける
        target = int(device) if str(device).isdigit() else device
        cap = cv2.VideoCapture(target)
        if not cap.isOpened():
            cap.release()
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0.0:
            cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap = cap
        return True

    def _on_timer(self) -> None:
        if self._cap is None or not self._cap.isOpened():
            if self._cap is not None:
                self._cap.release()
                self._cap = None
            # 起動時にデバイスが不在でも落とさず待ち受けるため
            # エラーを出して次周期に開き直す
            device = self.get_parameter("device").value
            self.get_logger().error(
                f"カメラを開けませんでした: {device}、リトライします")
            self._open_device()
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().error("フレーム取得に失敗しました、再接続します")
            self._cap.release()
            self._cap = None
            return
        msg = ImageMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        self._pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PcCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._cap is not None:
            node._cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
