"""PCカメラとGazeboカメラを切り替えられる検出起動ランチャー。

    ros2 launch umeonigiri_vision pc_vision.launch.py
    ros2 launch umeonigiri_vision pc_vision.launch.py use_pc_camera:=true backend:=onnx

backend:=npu（既定）は RK3588 の 3 NPU コア並列プールで YOLO を回す。
どちらの実装も ./detections と ./detection_image を出すので、Web UI とアーム制御は共通。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def default_rknn_model(share):
    """高精度モデルから順に探索し、無ければ yolov8s / yolov8n へフォールバックする。"""
    candidates = (
        "yolo11m-zoo.rknn",
        "yolov8m-zoo.rknn",
        "yolo11s-zoo.rknn",
        "yolov8s-zoo.rknn",
        "yolov8n-zoo.rknn",
    )
    for name in candidates:
        path = os.path.join(share, "models", name)
        if os.path.isfile(path):
            return path
    return os.path.join(share, "models", "yolov8s-zoo.rknn")


def generate_launch_description():
    share = get_package_share_directory("umeonigiri_vision")
    labels = os.path.join(share, "config", "coco_labels.txt")
    on_npu = conditions.LaunchConfigurationEquals("backend", "npu")
    on_onnx = conditions.LaunchConfigurationEquals("backend", "onnx")
    # NPU 版は自前で /dev/video0 を開くため、ONNX 版かつ use_pc_camera:=true のときだけ
    # pc_camera を起動し、V4L2 デバイス競合を防ぐ。
    camera_on = conditions.IfCondition(PythonExpression([
        "'", LaunchConfiguration("use_pc_camera"), "' == 'true' and '",
        LaunchConfiguration("backend"), "' == 'onnx'"
    ]))

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
            "conf_threshold", default_value="0.35",
            description="信頼度しきい値（トラッカの単発FP除去と併用）"),
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
            description="PC内蔵カメラを使うか (ONNX時のみ pc_camera ノードを起動)"),
        DeclareLaunchArgument(
            "model_path", default_value="",
            description="YOLOモデルへの明示パス。空なら自動解決"),
        DeclareLaunchArgument(
            "backend", default_value="npu",
            description="推論の実行先。npu=RKNN(この機体のNPU)、onnx=ONNX Runtime(CPU)"),
        DeclareLaunchArgument(
            "rknn_model",
            default_value=default_rknn_model(share),
            description="RKNN モデルのパス（yolo11m/yolov8m/yolo11s/yolov8s/yolov8n の順で探索）"),
        DeclareLaunchArgument(
            "display_fps", default_value="12.0",
            description="npu 時の配信レート上限 [Hz]"),
        DeclareLaunchArgument(
            "output_fps", default_value="25.0",
            description="npu の /detections 固定配信レート [Hz]。0 はカメラ同期"),
        DeclareLaunchArgument(
            "annotated_width", default_value="320",
            description="表示用画像の幅 [px]。0 で元解像度"),
        DeclareLaunchArgument(
            "npu_workers", default_value="3",
            description="RK3588 NPU 並列ワーカー数 (1..3, CORE_0/1/2 へ割当)"),
        DeclareLaunchArgument(
            "opencv_threads", default_value="2",
            description="OpenCV 内部スレッド上限（Nav2/SLAM とのコア競合防止）"),
        DeclareLaunchArgument(
            "tracker_enabled", default_value="true",
            description="軽量マルチ物体トラッカ（単発FP除去・ID維持）を有効化"),
        DeclareLaunchArgument(
            "tracker_min_hits", default_value="2",
            description="トラッカ確定までの連続フレーム数"),
        DeclareLaunchArgument(
            "tracker_max_missed", default_value="3",
            description="トラッカ見失い許容フレーム数"),

        Node(
            package="umeonigiri_vision",
            executable="pc_camera",
            name="pc_camera",
            output="screen",
            parameters=[{
                "device": LaunchConfiguration("device"),
                "fps": LaunchConfiguration("camera_fps"),
            }],
            condition=camera_on,
        ),
        Node(
            package="umeonigiri_vision",
            executable="yolo_grasp_node",
            name="yolo_grasp",
            output="screen",
            additional_env={
                "OMP_NUM_THREADS": "2",
                "OPENBLAS_NUM_THREADS": "2",
            },
            parameters=[{
                "model": LaunchConfiguration("rknn_model"),
                "labels": labels,
                "device": LaunchConfiguration("device"),
                "conf": LaunchConfiguration("conf_threshold"),
                "iou": LaunchConfiguration("iou_threshold"),
                "display_fps": LaunchConfiguration("display_fps"),
                "output_fps": LaunchConfiguration("output_fps"),
                "annotated_width": LaunchConfiguration("annotated_width"),
                "npu_workers": LaunchConfiguration("npu_workers"),
                "opencv_threads": LaunchConfiguration("opencv_threads"),
                "tracker_enabled": LaunchConfiguration("tracker_enabled"),
                "tracker_min_hits": LaunchConfiguration("tracker_min_hits"),
                "tracker_max_missed": LaunchConfiguration("tracker_max_missed"),
            }],
            condition=on_npu,
        ),
        Node(
            package="umeonigiri_vision",
            executable="detection_node",
            name="umeonigiri_vision",
            output="screen",
            additional_env={
                "OMP_NUM_THREADS": "2",
                "OPENBLAS_NUM_THREADS": "2",
            },
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
