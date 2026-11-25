import os
import re

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
    # 物理ステップを間引くと、同じ sim 時間を少ない iteration で進められる。
    # ワールド SDF を書き換えず /tmp のコピーだけを gz に渡す (既定は無変更)。
    physics_step = context.launch_configurations.get('physics_step', '')
    if physics_step:
        with open(world_path, encoding='utf-8') as handle:
            text = handle.read()
        text, count = re.subn(
            r'<max_step_size>[^<]+</max_step_size>',
            f'<max_step_size>{physics_step}</max_step_size>', text, count=1)
        if count != 1:
            raise RuntimeError(f'max_step_size が 1 つではありません: {world_path}')
        world_path = os.path.join('/tmp', f'meister_{world_key}_step_{physics_step}.sdf')
        with open(world_path, 'w', encoding='utf-8') as handle:
            handle.write(text)
    # GUI は表示だけ。計測やヘッドレス運用では切ってよい（物理とセンサはサーバ側）。
    # `-s` はサーバ単体起動。付けないと GUI も一緒に上がり、ソフトウェア描画で
    # 1 コア近くを GUI が食う（実測 66%）。
    gz_args = f'-r {world_path}' if gui else f'-s -r {world_path}'
    # -z は明示指定時だけ使う。実測ではセンサ更新まで遅くなった (9.6 Hz)。
    update_rate = context.launch_configurations.get('sim_update_rate', '')
    if update_rate:
        gz_args += f' -z {update_rate}'
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
            '/arm_camera_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/umeonigiri_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            (LaunchConfiguration('clock_topic'), '/clock'),
            ('/model/umeonigiri_bot/cmd_vel', '/cmd_vel'),
            ('/model/umeonigiri_bot/odometry', '/odom'),
            ('/model/umeonigiri_bot/tf', '/tf'),
            ('/camera_image', '/camera/image_raw'),
            ('/arm_camera_image', '/arm_camera/image_raw'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World to load: warehouse, maze, or empty'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI を出すか。false でサーバ単体 (ヘッドレス)'),
        DeclareLaunchArgument('sim_update_rate', default_value='',
                              description='gz sim -z。空ならワールド既定 (1ms step)'),
        DeclareLaunchArgument('physics_step', default_value='',
                              description='/tmp にコピーした SDF の max_step_size。空なら無変更'),
        OpaqueFunction(function=_resolve_world),
        gazebo,
        spawn_robot,
        bridge,
    ])
