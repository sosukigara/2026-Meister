import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, ExecuteProcess, LogInfo, DeclareLaunchArgument
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_autonomous_nav')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device connected to ESP32 UART (GPIO16/17)')

    serial_port = LaunchConfiguration('serial_port')

    # 使用するURDFファイル（実機でもロボットの形状定義は必要）
    # 通常は CPU 用のシンプルなものを使用します
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot_cpu.urdf')
    
    # 実機用のパラメータファイル
    slam_params_file = os.path.join(
        pkg_share, 'config', 'real_mapper_params_online_async.yaml'
    )
    nav2_params_file = os.path.join(
        pkg_share, 'config', 'real_nav2_params.yaml'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # --- Standard Nodes ---

    # ロボットの状態（TF）を配信
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
        output='screen'
    )

    # --- SLAM (Slam Toolbox) ---
    # 実機のLiDARデータ (/scan) を元に地図を作成
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False},
        ],
    )

    # SLAMのライフサイクル管理 (実機コマンドを利用)
    configure_slam = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            ),
        ]
    )

    activate_slam = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            ),
        ]
    )

    # --- Nav2 Stack ---
    # 実機のオドメトリ (/odom) と地図を元に経路計画と制御を行う

    # /cmd_vel -> ESP32 UART (ステアリング舵角 + モータ PWM フレーム)
    serial_bridge = Node(
        package='meister_serial_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud': 115200,
        }],
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

    # cmd_vel_nav -> cmd_vel のブリッジ (これがないと実機のモータに指令が届かない)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
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
        serial_port_arg,
        LogInfo(msg='=== Starting Real Robot Navigation (use_sim_time: False) ==='),
        LogInfo(msg='Make sure your hardware drivers (LiDAR, Odom, Micro-ROS) are running!'),

        robot_state_publisher,
        slam_toolbox,
        configure_slam,
        activate_slam,

        # /cmd_vel -> ESP32 UART (PWM/ステアリング変換)
        serial_bridge,

        # SLAMが安定してからNav2を起動
        TimerAction(period=10.0, actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            lifecycle_manager,
        ]),
    ])
