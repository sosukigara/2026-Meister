import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # name を付けないこと。付けると __node:= リマップになり、プロセス内の
    # 3 ノード (map_listener / vision_listener / commander) が全て同名で上書きされ、
    # 同名ノードが 3 重登録・/map 購読が見えなくなる。ノード名は C++ 側の明示名を使う。
    # プロセス内の 3 ノードは同じパラメータ上書きを共有する。シミュレーションでは
    # use_sim_time を立てないと initialpose を実時間でスタンプし、AMCL が
    # TF と噛み合わず初期化できない。
    use_sim_time = LaunchConfiguration('use_sim_time')
    # 保存地図の既定名 <world>_<timestamp> に使う。空なら map_<timestamp>。
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('world', default_value=''),
        Node(
            package='umeonigiri_webnav',
            executable='umeonigiri_webnav_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # symlink-install ではソースの maps/ を指す。保存地図を
                # install 側に散らかさない。
                'maps_dir': os.path.join(
                    get_package_share_directory('umeonigiri_autonomy'), 'maps'),
                'world': world,
            }],
        ),
    ])
