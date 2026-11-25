"""手先カメラ + NPU 検出 + SLAM をヘッドレスで 1 コマンドで起動する。

    ros2 launch umeonigiri_autonomy arm_vision_slam.launch.py

RViz も gz GUI も起動しない。計測手順は docs/perf/arm-cam-30fps-slam.md。
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vision_node_common import arm_vision_node as arm_vision  # noqa: E402


def generate_launch_description():
    pkg_nav = get_package_share_directory("umeonigiri_autonomy")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={
            "sensor_profile": LaunchConfiguration("sensor_profile"),
            "arm_camera_rate": LaunchConfiguration("arm_camera_rate"),
            "arm_camera_width": LaunchConfiguration("arm_camera_width"),
            "arm_camera_height": LaunchConfiguration("arm_camera_height"),
            # 手先カメラに CPU を集中させるため base カメラは 1 Hz に落とす。
            # base 経路自体は残る (Web UI 契約を壊さない)。
            "base_camera_rate": "1",
        }.items())

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "simulation.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": "false",
            "sim_update_rate": LaunchConfiguration("sim_update_rate"),
            "physics_step": LaunchConfiguration("physics_step"),
            "vision_target": LaunchConfiguration("vision_target"),
        }.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "slam.launch.py")))

    # slam_toolbox の configure/activate は固定 delay ではなく状態遷移で待つ (P12)。
    nav_mode_manager = Node(
        package="umeonigiri_autonomy",
        executable="nav_mode_manager",
        name="nav_mode_manager",
        output="screen",
        parameters=[{"slam_node": "/slam_toolbox",
                     "initial_mode": "mapping"}])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="maze",
                              description="maze / warehouse / empty"),
        DeclareLaunchArgument("sensor_profile", default_value="lite",
                              description="lite=360点/5Hz・320x240 (llvmpipe 向け)"),
        DeclareLaunchArgument("arm_camera_rate", default_value="45",
                              description="手先カメラの gz update_rate (sim Hz)"),
        # llvmpipe の描画コストを抑えるため 80x60。NPU 入力は 640 へ拡大する。
        DeclareLaunchArgument("arm_camera_width", default_value="80",
                              description="手先カメラの画像幅 [px]"),
        DeclareLaunchArgument("arm_camera_height", default_value="60",
                              description="手先カメラの画像高さ [px]"),
        DeclareLaunchArgument("sim_update_rate", default_value="",
                              description="gz sim -z [Hz]。既定は空 (ワールドの step を使う)"),
        # 1ms -> 10ms step。gz の iteration を 1/10 にして RTF を 0.86 まで上げる (実測)。
        DeclareLaunchArgument("physics_step", default_value="0.01",
                              description="/tmp の SDF コピーに使う max_step_size"),
        DeclareLaunchArgument("vision_target", default_value="false",
                              description="maze に計測用テクスチャ対象を追加 (既定は false)"),
        robot_state_publisher,
        simulation,
        slam,
        nav_mode_manager,
        arm_vision(),
    ])
