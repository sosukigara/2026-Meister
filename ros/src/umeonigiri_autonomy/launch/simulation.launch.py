import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ロボットの出現位置。world 名と SDF の対応は config/worlds.yaml が持つ
# （robot_nav.launch.py と共有）。高さは機体の足回りで決まるのでここに置く。
SPAWN = {
    'warehouse': (0, 0, 0.1),
    'maze': (-4.0, -4.0, 0.1),
    'empty': (0, 0, 0.1),
}


def load_worlds():
    """config/worlds.yaml を読む。SDF 名と gz の world 名の唯一の定義。"""
    path = os.path.join(get_package_share_directory('umeonigiri_autonomy'),
                        'config', 'worlds.yaml')
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _resolve_world(context):
    world_key = context.launch_configurations['world']
    gui = context.launch_configurations.get('gui', 'true') == 'true'
    entry = load_worlds()[world_key]
    spawn = SPAWN[world_key]
    pkg_nav = get_package_share_directory('umeonigiri_autonomy')
    world_path = os.path.join(pkg_nav, 'worlds', entry['sdf'])
    # GUI は表示だけ。計測やヘッドレス運用では切ってよい（物理とセンサはサーバ側）。
    # `-s` はサーバ単体起動。付けないと GUI も一緒に上がり、ソフトウェア描画で
    # 1 コア近くを GUI が食う（実測 66%）。
    gz_args = f'-r {world_path}' if gui else f'-s -r {world_path}'
    return [
        SetLaunchConfiguration('gz_args', gz_args),
        SetLaunchConfiguration('spawn_x', str(spawn[0])),
        SetLaunchConfiguration('spawn_y', str(spawn[1])),
        SetLaunchConfiguration('spawn_z', str(spawn[2])),
        SetLaunchConfiguration('world_name', entry['world_name']),
        SetLaunchConfiguration('clock_topic', f'/world/{entry["world_name"]}/clock'),
    ]


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'umeonigiri_bot', '-topic', '/robot_description', '-x',
                   LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                   '-z', LaunchConfiguration('spawn_z')],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [LaunchConfiguration('clock_topic'), '@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            '/model/umeonigiri_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/umeonigiri_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/umeonigiri_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            (LaunchConfiguration('clock_topic'), '/clock'),
            ('/model/umeonigiri_bot/cmd_vel', '/cmd_vel'),
            ('/model/umeonigiri_bot/odometry', '/odom'),
            ('/model/umeonigiri_bot/tf', '/tf'),
            ('/camera_image', '/camera/image_raw'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World to load: warehouse, maze, or empty'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI を出すか。false でサーバ単体 (ヘッドレス)'),
        OpaqueFunction(function=_resolve_world),
        gazebo,
        spawn_robot,
        bridge,
    ])
