import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, ExecuteProcess, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def generate_launch_description():

    has_gpu = detect_gpu()
    mode = 'GPU' if has_gpu else 'CPU'

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_share = os.path.dirname(launch_dir)

    urdf_name = 'my_robot_gpu.urdf' if has_gpu else 'my_robot_cpu.urdf'
    urdf_file = os.path.join(pkg_share, 'urdf', urdf_name)
    world_file = os.path.join(pkg_share, 'worlds', 'my_custom_world.sdf')
    slam_params_file = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml'
    )
    nav2_params_file = os.path.join(
        pkg_share, 'config', 'mapping_nav_params.yaml'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gz_args = f'-r {world_file}'
    if not has_gpu:
        gz_args = f'-r --render-engine ogre2 {world_file}'

    # --- Standard Nodes ---

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
        launch_arguments={'gz_args': gz_args}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',
            '-x', '-4.0', '-y', '-4.0', '-z', '0.1',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/my_custom_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/my_custom_world/clock', '/clock'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- SLAM ---

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

    # --- Nav2 ---

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
        LogInfo(msg=f'=== All-in-One Mapping + Nav — Running in {mode} mode ==='),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        slam_toolbox,
        configure_slam,
        activate_slam,
        # Delay Nav2 to let SLAM start first
        TimerAction(period=25.0, actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]),
    ])
