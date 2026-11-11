from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # name を付けないこと。付けると __node:= リマップになり、プロセス内の
    # 3 ノード (map_listener / vision_listener / commander) が全て同名で上書きされ、
    # 同名ノードが 3 重登録・/map 購読が見えなくなる。ノード名は C++ 側の明示名を使う。
    return LaunchDescription([
        Node(
            package='meister_web_nav',
            executable='web_nav_server',
            output='screen',
        ),
    ])
