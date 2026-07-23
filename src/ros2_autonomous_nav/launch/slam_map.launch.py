import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, ExecuteProcess, LogInfo,
    DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def detect_gpu():
    """Detect if an NVIDIA GPU is available."""
    try:
        result = subprocess.run(
            ['nvidia-smi'], capture_output=True, timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


WORLDS = {
    'warehouse': {'sdf': 'warehouse.sdf', 'world_name': 'warehouse', 'spawn': (0, 0, 0.5)},
    'maze': {'sdf': 'maze.sdf', 'world_name': 'my_custom_world', 'spawn': (-4.0, -4.0, 0.1)},
    'empty': {'sdf': 'empty.sdf', 'world_name': 'empty', 'spawn': (0, 0, 0.1)},
}


def generate_launch_description():

    has_gpu = detect_gpu()
    mode = 'GPU' if has_gpu else 'CPU'

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_share = os.path.dirname(launch_dir)

    urdf_name = 'my_robot_gpu.urdf' if has_gpu else 'my_robot_cpu.urdf'
    urdf_file = os.path.join(pkg_share, 'urdf', urdf_name)
    slam_params_file = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    def _resolve_world(context):
        world_key = context.launch_configurations['world']
        entry = WORLDS[world_key]
        launch_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_share = os.path.dirname(launch_dir)
        world_path = os.path.join(pkg_share, 'worlds', entry['sdf'])
        gz_args = f'-r {world_path}'
        if not has_gpu:
            gz_args = f'-r --render-engine ogre2 {world_path}'
        return [
            SetLaunchConfiguration('gz_args', gz_args),
            SetLaunchConfiguration('spawn_x', str(entry['spawn'][0])),
            SetLaunchConfiguration('spawn_y', str(entry['spawn'][1])),
            SetLaunchConfiguration('spawn_z', str(entry['spawn'][2])),
            SetLaunchConfiguration('world_name', entry['world_name']),
            SetLaunchConfiguration('clock_topic', f'/world/{entry["world_name"]}/clock'),
        ]

    declare_world_arg = DeclareLaunchArgument('world', default_value='warehouse',
                                              description='World to load: warehouse, maze, or empty')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-z', LaunchConfiguration('spawn_z'),
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [LaunchConfiguration('clock_topic'), '@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            (LaunchConfiguration('clock_topic'), '/clock'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True},
        ],
    )

    configure_slam = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            ),
        ]
    )

    activate_slam = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        declare_world_arg,
        OpaqueFunction(function=_resolve_world),
        LogInfo(msg=f'=== SLAM mode — Running in {mode} mode ({urdf_name}) ==='),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        slam_toolbox,
        configure_slam,
        activate_slam,
    ])
