import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch yolo_ros and human_tracker_node for Meister perception."""
    # Launch arguments
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

    # Path to yolo_ros parameter file
    config_file = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'yolo_ros_params.yaml',
    )

    # yolo_ros node
    yolo_ros_node = Node(
        package='yolo_ros',
        executable='yolo_ros_node',
        name='yolo_ros',
        parameters=[config_file],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ],
    )

    # human_tracker node (from this package)
    human_tracker_node = Node(
        package='perception',
        executable='human_tracker_node',
        name='human_tracker_node',
        parameters=[config_file],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ],
    )

    return LaunchDescription([
        camera_topic_arg,
        use_3d_arg,
        debug_arg,
        yolo_ros_node,
        human_tracker_node,
    ])
