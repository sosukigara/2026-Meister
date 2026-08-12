from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='meister_web_nav',
            executable='web_nav_server',
            name='web_nav_server',
            output='screen',
        ),
    ])
