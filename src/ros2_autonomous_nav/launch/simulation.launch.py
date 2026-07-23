import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = {
    'warehouse': {'sdf': 'warehouse.sdf', 'world_name': 'warehouse', 'spawn': (0, 0, 0.5)},
    'maze': {'sdf': 'maze.sdf', 'world_name': 'my_custom_world', 'spawn': (-4.0, -4.0, 0.1)},
    'empty': {'sdf': 'empty.sdf', 'world_name': 'empty', 'spawn': (0, 0, 0.1)},
}


def _resolve_world(context):
    world_key = context.launch_configurations['world']
    entry = WORLDS[world_key]
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    world_path = os.path.join(pkg_nav, 'worlds', entry['sdf'])
    gz_args = f'-r {world_path}'
    return [
        SetLaunchConfiguration('gz_args', gz_args),
        SetLaunchConfiguration('spawn_x', str(entry['spawn'][0])),
        SetLaunchConfiguration('spawn_y', str(entry['spawn'][1])),
        SetLaunchConfiguration('spawn_z', str(entry['spawn'][2])),
        SetLaunchConfiguration('world_name', entry['world_name']),
        SetLaunchConfiguration('clock_topic', f'/world/{entry["world_name"]}/clock'),
    ]


def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'meistar_bot', '-topic', '/robot_description', '-x',
                   LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                   '-z', LaunchConfiguration('spawn_z')],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [LaunchConfiguration('clock_topic'), '@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            '/model/meistar_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/meistar_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/meistar_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            (LaunchConfiguration('clock_topic'), '/clock'),
            ('/model/meistar_bot/cmd_vel', '/cmd_vel'),
            ('/model/meistar_bot/odometry', '/odom'),
            ('/model/meistar_bot/tf', '/tf')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World to load: warehouse, maze, or empty'),
        OpaqueFunction(function=_resolve_world),
        gazebo,
        spawn_robot,
        bridge,
    ])
