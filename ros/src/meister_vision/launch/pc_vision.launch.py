"""PCカメラとGazeboカメラを切り替えられる検出起動ランチャー。

    ros2 launch meister_vision pc_vision.launch.py
    ros2 launch meister_vision pc_vision.launch.py use_pc_camera:=true
    ros2 launch meister_vision pc_vision.launch.py backend:=onnx   # CPU 版へ戻す

backend:=npu（既定）は RK3588 の NPU で yolov8n を回す。どちらの実装も
./detections と ./detection_image を出すので、Web UI とアーム制御は共通。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("meister_vision")
    labels = os.path.join(share, "config", "coco_labels.txt")
    on_npu = conditions.LaunchConfigurationEquals("backend", "npu")
    on_onnx = conditions.LaunchConfigurationEquals("backend", "onnx")
    # pc_camera は ONNX 版のための供給源。NPU 版は自前で /dev/video0 を開くので、
    # backend:=npu のときは use_pc_camera:=true にしないこと（同時に開けない）。
    camera_on = conditions.IfCondition(LaunchConfiguration("use_pc_camera"))

    return LaunchDescription([
        DeclareLaunchArgument(
            "device", default_value="/dev/video0",
            description="PCカメラのデバイスパス"),
        DeclareLaunchArgument(
            "camera_fps", default_value="30.0",
            description="PCカメラの配信フレームレート [Hz]"),
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/image_raw",
            description="検出ノードが購読する画像トピック名"),
        DeclareLaunchArgument(
            "conf_threshold", default_value="0.45",
            description="信頼度しきい値。誤検出が多いときは上げる"),
        DeclareLaunchArgument(
            "iou_threshold", default_value="0.45",
            description="NMS の IoU しきい値"),
        DeclareLaunchArgument(
            "rate", default_value="30.0",
            description="最大処理レート [Hz] (推論が追いつく上限で頭打ち)"),
        DeclareLaunchArgument(
            "inference_threads", default_value="4",
            description="ONNX 推論スレッド数 (backend:=onnx のときだけ使う)"),
        DeclareLaunchArgument(
            "target_class", default_value="-1",
            description="追跡するクラス ID (-1 は指定なし)"),
        DeclareLaunchArgument(
            "target_lock_radius", default_value="120.0",
            description="同一対象とみなす移動量 [px]"),
        DeclareLaunchArgument(
            "use_pc_camera", default_value="false",
            description="PC内蔵カメラを使うか (既定はGazeboロボット目線カメラ)"),
        DeclareLaunchArgument(
            "model_path", default_value="",
            description="YOLOモデルへの明示パス。空なら自動解決 "
                        "(640既定。320を使う場合はパスを指定)"),
        DeclareLaunchArgument(
            "backend", default_value="npu",
            description="推論の実行先。npu=RKNN(この機体のNPU)、onnx=ONNX Runtime(CPU)"),
        DeclareLaunchArgument(
            "rknn_model",
            default_value=os.path.join(share, "models", "yolov8n-zoo.rknn"),
            description="RKNN モデルのパス（tools/rknn の手順で生成する）"),
        DeclareLaunchArgument(
            "display_fps", default_value="15.0",
            description="npu 時の配信レート上限 [Hz]"),

        Node(
            package="meister_vision",
            executable="pc_camera",
            name="pc_camera",
            output="screen",
            parameters=[{
                "device": LaunchConfiguration("device"),
                "fps": LaunchConfiguration("camera_fps"),
            }],
            condition=camera_on,
        ),
        # NPU 版（C++）。int8 のままデコードするので Python 版より 5 倍速い。
        Node(
            package="meister_vision",
            executable="yolo_grasp_node",
            name="yolo_grasp",
            output="screen",
            parameters=[{
                "model": LaunchConfiguration("rknn_model"),
                "labels": labels,
                "device": LaunchConfiguration("device"),
                "conf": LaunchConfiguration("conf_threshold"),
                "iou": LaunchConfiguration("iou_threshold"),
                "display_fps": LaunchConfiguration("display_fps"),
            }],
            condition=on_npu,
        ),
        Node(
            package="meister_vision",
            executable="detection_node",
            name="meister_vision",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "model_path": LaunchConfiguration("model_path"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "iou_threshold": LaunchConfiguration("iou_threshold"),
                "rate": LaunchConfiguration("rate"),
                "inference_threads": LaunchConfiguration("inference_threads"),
                "target_class": LaunchConfiguration("target_class"),
                "target_lock_radius": LaunchConfiguration("target_lock_radius"),
            }],
            condition=on_onnx,
        ),
    ])
