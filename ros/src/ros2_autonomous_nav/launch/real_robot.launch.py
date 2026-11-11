import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, ExecuteProcess, LogInfo, DeclareLaunchArgument,
    IncludeLaunchDescription, RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_autonomous_nav')
    pkg_web_nav = get_package_share_directory('meister_web_nav')
    pkg_vision = get_package_share_directory('meister_vision')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device connected to ESP32 UART (GPIO16/17)')

    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
        description='Baudrate for ESP32 UART')

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB1',
        description='Serial device connected to LD-D500 LiDAR (serial_bridge と被らないこと)')

    vision_model_arg = DeclareLaunchArgument(
        'vision_model', default_value='',
        description='YOLO model path (empty=auto 640. 320=model small/fast)')

    vision_threads_arg = DeclareLaunchArgument(
        'vision_threads', default_value='4',
        description='ONNX inference threads (320x2 reaches 10Hz on RK3588)')

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    lidar_port = LaunchConfiguration('lidar_port')

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

    # --- odom->base_footprint (static, 不動仮定) ---
    # 実機には odom 発行源 (Gazebo odometry / ESP32 odom) がない。SLAM の message filter は
    # odom フレームの存在を要求するため、identity を置き、ずれは SLAM の map->odom が補正する。
    # ESP32 odom ができたらこのノードを置き換えること。
    odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='real_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': False}],
    )

    # --- SLAM (Slam Toolbox) ---
    # 実機のLiDARデータ (/scan) を元に地図を作成
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False},
        ],
    )

    # SLAMのライフサイクル管理。configure 完了 (inactive) を待ってから activate する。
    # 固定秒数で activate を待つと configure 前に走り、SLAM が unconfigured のまま固まる。
    configure_slam = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            ),
        ]
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            goal_state='inactive',
            entities=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                ),
            ],
        )
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
            'baud': serial_baud,
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

    # --- LD-D500 LiDAR (/scan) ---
    # SLAM の地図作成・自己位置推定の入力。frame_id は URDF の lidar_link と一致させる。
    lidar = Node(
        package='ros2_ldlidar_d500',
        executable='d500_node',
        name='d500_node',
        output='screen',
        parameters=[{
            'port': lidar_port,
            'baudrate': 230400,
            'frame_id': 'lidar_link',
            'topic_name': '/scan',
        }],
    )

    # --- Web UI (map view + click-to-send waypoints) — http://localhost:8088 ---
    web_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_web_nav, 'launch', 'web_nav.launch.py'))
    )

    # --- Vision (PCカメラ+YOLOv8n検出。実機に Gazeboカメラはないため use_pc_camera=true 固定) ---
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vision, 'launch', 'pc_vision.launch.py')),
        launch_arguments={
            'use_pc_camera': 'true',
            'model_path': LaunchConfiguration('vision_model'),
            'inference_threads': LaunchConfiguration('vision_threads'),
        }.items()
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        lidar_port_arg,
        vision_model_arg,
        vision_threads_arg,
        LogInfo(msg='=== Starting Real Robot Navigation (use_sim_time: False) ==='),
        LogInfo(msg='Make sure your hardware (LD-D500 LiDAR, ESP32) is connected!'),

        robot_state_publisher,
        odom_static_tf,
        lidar,
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

        web_nav,
        vision,
    ])
