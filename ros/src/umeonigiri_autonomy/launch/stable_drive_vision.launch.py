"""自動走行 + 手先カメラ NPU 検出をヘッドレスで 1 コマンド起動する。

    ros2 launch umeonigiri_autonomy stable_drive_vision.launch.py \
        world:=maze vision_target:=true

Gazebo サーバのみ (-s)、RViz / gz GUI なし。SLAM + Nav2 + Web UI + NPU 検出を
同時に起動する。計測手順は docs/perf/stable-drive-accurate-vision.md。

既存の arm_vision_slam.launch.py / mapping_nav.launch.py の既定挙動は変更しない。
NPU ノードのパラメータは vision_node_common.py、Nav2 は mapping_nav_params.yaml、
SDF 差し込みは simulation.launch.py の 1 箇所だけに置く (R2)。
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vision_node_common import arm_vision_node  # noqa: E402


def _include(pkg, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), "launch", launch_file)),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="maze",
                              description="maze / warehouse / empty"),
        DeclareLaunchArgument("sensor_profile", default_value="lite",
                              description="lite=360点/5Hz (llvmpipe 向け)"),
        DeclareLaunchArgument("arm_camera_rate", default_value="45",
                              description="手先カメラの gz update_rate"),
        DeclareLaunchArgument("arm_camera_width", default_value="80"),
        DeclareLaunchArgument("arm_camera_height", default_value="60"),
        DeclareLaunchArgument("physics_step", default_value="0.01",
                              description="/tmp SDF コピーの max_step_size"),
        DeclareLaunchArgument("vision_target", default_value="true",
                              description="計測用テクスチャ対象 (maze 専用)"),
        # base カメラは Web UI 契約を保つため残し、CPU を手先カメラへ集中させる。
        _include("umeonigiri_autonomy", "robot_state_publisher.launch.py", {
            "sensor_profile": LaunchConfiguration("sensor_profile"),
            "arm_camera_rate": LaunchConfiguration("arm_camera_rate"),
            "arm_camera_width": LaunchConfiguration("arm_camera_width"),
            "arm_camera_height": LaunchConfiguration("arm_camera_height"),
            "base_camera_rate": "1",
        }),
        _include("umeonigiri_autonomy", "simulation.launch.py", {
            "world": LaunchConfiguration("world"),
            "gui": "false",
            "physics_step": LaunchConfiguration("physics_step"),
            "vision_target": LaunchConfiguration("vision_target"),
        }),
        _include("umeonigiri_autonomy", "slam.launch.py"),
        _include("umeonigiri_autonomy", "navigation.launch.py", {
            "use_sim_time": "true",
        }),
        _include("umeonigiri_webnav", "umeonigiri_webnav.launch.py", {
            "use_sim_time": "true",
            "world": LaunchConfiguration("world"),
        }),
        arm_vision_node(),
    ])
