"""PCカメラとGazeboカメラを切り替えられる検出起動ランチャー。
    ros2 launch meister_vision pc_vision.launch.py
    ros2 launch meister_vision pc_vision.launch.py use_pc_camera:=true
"""
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/image_raw",
            description="検出ノードが購読する画像トピック名"),
        DeclareLaunchArgument(
            "conf_threshold", default_value="0.25",
            description="信頼度しきい値"),
        DeclareLaunchArgument(
            "iou_threshold", default_value="0.45",
            description="NMS の IoU しきい値"),
        DeclareLaunchArgument(
            "rate", default_value="3.0",
            description="最大処理レート [Hz]"),
        DeclareLaunchArgument(
            "inference_threads", default_value="2",
            description="ONNX 推論スレッド数"),
        DeclareLaunchArgument(
            "use_pc_camera", default_value="false",
            description="PC内蔵カメラを使うか (既定はGazeboロボット目線カメラ)"),

        Node(
            package="meister_vision",
            executable="pc_camera",
            name="pc_camera",
            output="screen",
            condition=conditions.IfCondition(
                LaunchConfiguration("use_pc_camera")),
        ),
        Node(
            package="meister_vision",
            executable="detection_node",
            name="meister_vision",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "iou_threshold": LaunchConfiguration("iou_threshold"),
                "rate": LaunchConfiguration("rate"),
                "inference_threads": LaunchConfiguration("inference_threads"),
            }],
        ),
    ])
