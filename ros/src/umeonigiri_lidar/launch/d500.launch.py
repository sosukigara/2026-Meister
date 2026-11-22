from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("umeonigiri_lidar"),
        "config",
        "d500.yaml"
    ])

    return LaunchDescription([
        Node(
            package="umeonigiri_lidar",
            executable="d500_node",
            name="d500_node",
            output="screen",
            parameters=[config_file]
        )
    ])
