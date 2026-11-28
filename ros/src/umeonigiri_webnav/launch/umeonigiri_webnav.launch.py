import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # name を付けないこと。付けると __node:= リマップになり、プロセス内の
    # 3 ノード (map_listener / vision_listener / commander) が全て同名で上書きされる。
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    node_prefix = LaunchConfiguration('node_prefix')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument(
            'node_prefix', default_value='',
            description='プロセス起動プレフィックス（RK3588 小コア隔離時は taskset -c 0-3）'),
        Node(
            package='umeonigiri_webnav',
            executable='umeonigiri_webnav_server',
            prefix=node_prefix,
            additional_env={
                'OMP_NUM_THREADS': '1',
                'OPENBLAS_NUM_THREADS': '1',
            },
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'maps_dir': os.path.join(
                    get_package_share_directory('umeonigiri_autonomy'), 'maps'),
                'world': world,
            }],
        ),
    ])
