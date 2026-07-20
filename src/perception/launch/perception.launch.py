import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _executable_path(pkg_name: str, exe: str) -> str | None:
    """Return the full path to a ROS2 package executable, or None."""
    try:
        base = get_package_share_directory(pkg_name)
        # The package install root is two levels up from share/<pkg>
        install_root = os.path.dirname(os.path.dirname(base))
        candidates = [
            os.path.join(install_root, 'lib', pkg_name, exe),
            os.path.join(install_root, 'bin', exe),
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        return None
    except (PackageNotFoundError, Exception):
        return None


def generate_launch_description():
    """Launch yolo_ros and human_tracker_node for Meister perception."""
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Input camera topic for object detection',
    )
    use_3d_arg = DeclareLaunchArgument(
        'use_3d',
        default_value='false',
        description='Enable 3D position estimation',
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug visualization',
    )

    config_file = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'yolo_ros_params.yaml',
    )

    actions = [camera_topic_arg, use_3d_arg, debug_arg]

    # yolo_ros node — skip gracefully if package not installed
    try:
        get_package_share_directory('yolo_ros')
        actions.append(Node(
            package='yolo_ros',
            executable='yolo_ros_node',
            name='yolo_ros',
            parameters=[config_file],
            remappings=[
                ('/camera/image_raw', LaunchConfiguration('camera_topic')),
            ],
        ))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[perception.launch] yolo_ros not installed — skipping object detection'))

    # human_tracker node — skip gracefully if executable not found
    exe_path = _executable_path('perception', 'human_tracker_node')
    if exe_path:
        actions.append(Node(
            package='perception',
            executable='human_tracker_node',
            name='human_tracker_node',
            parameters=[config_file],
            remappings=[
                ('/camera/image_raw', LaunchConfiguration('camera_topic')),
            ],
        ))
    else:
        actions.append(LogInfo(msg='[perception.launch] human_tracker_node not found — skipping human tracking'))

    return LaunchDescription(actions)
