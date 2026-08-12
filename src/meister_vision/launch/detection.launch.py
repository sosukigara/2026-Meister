"""detection_node 起動用ランチャー。

使い方:
    ros2 launch meister_vision detection.launch.py
    ros2 launch meister_vision detection.launch.py image_topic:=/camera/image_raw \
        conf_threshold:=0.3 model_path:=$HOME/models/yolov8n.onnx
    ros2 launch meister_vision detection.launch.py start_rviz:=true \
        rviz_config:=src/meister_vision/rviz/detection.rviz
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory("meister_vision"), "rviz", "detection.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path", default_value="",
            description="YOLOv8n ONNX モデルのパス (空なら自動解決)"),
        DeclareLaunchArgument(
            "conf_threshold", default_value="0.25",
            description="信頼度しきい値"),
        DeclareLaunchArgument(
            "iou_threshold", default_value="0.45",
            description="NMS の IoU しきい値"),
        DeclareLaunchArgument(
            "image_topic", default_value="image_raw",
            description="購読する画像トピック名"),
        DeclareLaunchArgument(
            "publish_annotated", default_value="true",
            description="検出枠を描画した画像 (detection_image) を配信するか"),
        DeclareLaunchArgument(
            "rate", default_value="10.0",
            description="最大処理レート [Hz]"),
        DeclareLaunchArgument(
            "start_rviz", default_value="false",
            description="rviz2 を起動して検出画像を表示するか"),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="rviz2 設定ファイルのパス"),

        Node(
            package="meister_vision",
            executable="detection_node",
            name="meister_vision",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "iou_threshold": LaunchConfiguration("iou_threshold"),
                "image_topic": LaunchConfiguration("image_topic"),
                "publish_annotated": LaunchConfiguration("publish_annotated"),
                "rate": LaunchConfiguration("rate"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=conditions.IfCondition(
                LaunchConfiguration("start_rviz")),
        ),
    ])
