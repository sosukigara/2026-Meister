"""NPU 検出ノードの設定を 1 箇所に集約する。

arm_vision_slam.launch.py と stable_drive_vision.launch.py の両方から使う。
detection_fps=0 は必須: 30 Hz ゲートは推論 (~25 ms) と位相がぶつかり
実測で 18.6 Hz に落ちた (docs/pitfalls.md P22)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo
from launch_ros.actions import Node


def arm_vision_node():
    """RKNN モデルが無い環境では落ちるノードを増やさず、取得方法だけ伝える。"""
    pkg_vision = get_package_share_directory("umeonigiri_vision")
    model = os.path.join(pkg_vision, "models", "yolov8n-zoo.rknn")
    if not os.path.isfile(model):
        return LogInfo(msg="[vision] RKNN モデルが無いため NPU 検出を起動しません。"
                           " tools/rknn/README.md の手順で生成してください。")
    return Node(
        package="umeonigiri_vision",
        executable="yolo_grasp_node",
        name="yolo_grasp",
        output="screen",
        parameters=[{
            "model": model,
            "labels": os.path.join(pkg_vision, "config", "coco_labels.txt"),
            "device": "/dev/video0",
            "image_topic": "/arm_camera/image_raw",
            "use_topic_input": True,
            "detection_fps": 0.0,
            "display_fps": 30.0,
            "frame_id": "arm_camera_link",
            "conf": 0.35,
            "iou": 0.45,
            "use_sim_time": True,
        }],
    )
