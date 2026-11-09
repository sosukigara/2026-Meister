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

    # Pick GPU or CPU URDF
    urdf_name = 'my_robot_gpu.urdf' if has_gpu else 'my_robot_cpu.urdf'
    urdf_file = os.path.join(pkg_share, 'urdf', urdf_name)
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'nav_world.yaml')

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

    # --- Nodes ---

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

    # --- Nav2 stack ---

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file,
                     'use_sim_time': True}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[nav2_params_file],
    )

    return LaunchDescription([
        declare_world_arg,
        OpaqueFunction(function=_resolve_world),
        LogInfo(msg=f'=== Running in {mode} mode ({urdf_name}) ==='),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        # Delay Nav2 to let Gazebo + bridge start first
        TimerAction(period=8.0, actions=[
            map_server,
            amcl,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]),
    ])
