import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    slam_params_file = os.path.join(pkg_nav, 'config', 'mapper_params_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # configure/activate は lifecycle_manager に任せる。固定秒数の activate は
    # configure 完了前に走って SLAM が inactive のまま固まる（docs/pitfalls.md P12）。
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    # 地図生成側のライフサイクル。nav_mode_manager が PAUSE / RESUME で操作する。
    lifecycle_manager_mapping = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['slam_toolbox'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox,
        lifecycle_manager_mapping,
    ])
