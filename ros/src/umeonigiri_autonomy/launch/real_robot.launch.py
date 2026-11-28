import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, ExecuteProcess, LogInfo, DeclareLaunchArgument,
    IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('umeonigiri_autonomy')
    pkg_web_nav = get_package_share_directory('umeonigiri_webnav')
    pkg_vision = get_package_share_directory('umeonigiri_vision')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device connected to ESP32 UART (GPIO16/17)')

    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
        description='Baudrate for ESP32 UART')

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB1',
        description='Serial device connected to LD-D500 LiDAR (serial_bridge と被らないこと)')

    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode', default_value='mapping',
        description='自己位置モード: mapping（SLAM地図作成）または localization（事前地図自律走行）')

    localization_backend_arg = DeclareLaunchArgument(
        'localization_backend', default_value='amcl',
        description='localization 時の推定器: amcl または slam_toolbox')

    controller_profile_arg = DeclareLaunchArgument(
        'controller_profile', default_value='mppi',
        description='追従コントローラ: mppi（Ackermann MPPI・動的回避）または rpp（超低CPU切り返しRPP）')

    rk3588_affinity_arg = DeclareLaunchArgument(
        'rk3588_affinity', default_value='true',
        description='RK3588 big.LITTLE コア割当（A76=4-7: Nav2/SLAM/Vision, A55=0-3: I/O・軽量ノード）')

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='事前地図パス（amcl は .yaml、slam_toolbox は posegraph プレフィックス）')

    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='0.60',
        description='前後ステア間ホイールベース [m]（実測後に上書き）')

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel', default_value='0.28',
        description='駆動上限速度 [m/s]（LA8308 KV90 / 12V / 20:1 / φ100mm 理論上限 0.283）')

    use_encoder_feedback_arg = DeclareLaunchArgument(
        'use_encoder_feedback', default_value='false',
        description='FB_STATE の実測車輪エンコーダをオドメトリに使用するか')

    vision_model_arg = DeclareLaunchArgument(
        'vision_model', default_value='',
        description='YOLO model path (empty=auto)')

    vision_backend_arg = DeclareLaunchArgument(
        'vision_backend', default_value='npu',
        description='推論の実行先。npu=RKNN、onnx=ONNX Runtime(CPU)')
    vision_threads_arg = DeclareLaunchArgument(
        'vision_threads', default_value='4',
        description='ONNX inference threads (backend:=onnx 時のみ使用)')
    vision_output_fps_arg = DeclareLaunchArgument(
        'vision_output_fps', default_value='25.0',
        description='NPU 検出の固定配信レート [Hz]')
    vision_annotated_width_arg = DeclareLaunchArgument(
        'vision_annotated_width', default_value='320',
        description='表示用画像の幅 [px]。0 は元解像度')
    vision_npu_workers_arg = DeclareLaunchArgument(
        'vision_npu_workers', default_value='3',
        description='RK3588 NPU 並列ワーカー数 (1..3)')

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    lidar_port = LaunchConfiguration('lidar_port')
    nav_mode = LaunchConfiguration('nav_mode')
    localization_backend = LaunchConfiguration('localization_backend')
    controller_profile = LaunchConfiguration('controller_profile')
    rk3588_affinity = LaunchConfiguration('rk3588_affinity')
    map_path = LaunchConfiguration('map')

    is_mapping = IfCondition(PythonExpression(["'", nav_mode, "' == 'mapping'"]))
    is_amcl = IfCondition(PythonExpression([
        "'", nav_mode, "' == 'localization' and '", localization_backend, "' == 'amcl'"
    ]))
    is_slam_loc = IfCondition(PythonExpression([
        "'", nav_mode, "' == 'localization' and '", localization_backend, "' == 'slam_toolbox'"
    ]))
    use_rpp = IfCondition(PythonExpression(["'", controller_profile, "' == 'rpp'"]))
    use_mppi = IfCondition(PythonExpression(["'", controller_profile, "' != 'rpp'"]))

    # RK3588: CPU 0-3 = Cortex-A55 (小コア), CPU 4-7 = Cortex-A76 (大コア)
    big_core_prefix = PythonExpression([
        "'taskset -c 4-7' if '", rk3588_affinity, "' == 'true' else ''"
    ])
    little_core_prefix = PythonExpression([
        "'taskset -c 0-3' if '", rk3588_affinity, "' == 'true' else ''"
    ])

    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot_cpu.urdf')

    slam_mapping_params = os.path.join(
        pkg_share, 'config', 'real_mapper_params_online_async.yaml'
    )
    slam_loc_params = os.path.join(
        pkg_share, 'config', 'real_mapper_params_localization.yaml'
    )
    nav2_params_file = os.path.join(
        pkg_share, 'config', 'real_nav2_params.yaml'
    )
    rpp_overlay_file = os.path.join(
        pkg_share, 'config', 'rpp_lowspeed_overlay.yaml'
    )
    bt_xml_file = os.path.join(
        pkg_share, 'config', 'bt_lowspeed_replan.xml'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # OpenMP / BLAS が全 8 コアを無制限に消費してリアルタイムノードを圧迫するのを防ぐ
    omp_env = SetEnvironmentVariable('OMP_NUM_THREADS', '2')
    openblas_env = SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '2')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        prefix=little_core_prefix,
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
        output='screen'
    )

    # --- Mode A: SLAM Toolbox (Mapping) ---
    slam_toolbox_mapping = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        prefix=big_core_prefix,
        output='screen',
        condition=is_mapping,
        parameters=[
            slam_mapping_params,
            {'use_sim_time': False},
        ],
    )

    configure_slam_mapping = TimerAction(
        period=2.0,
        condition=is_mapping,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            ),
        ]
    )

    activate_slam_mapping = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_mapping,
            goal_state='inactive',
            entities=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                ),
            ],
        ),
        condition=is_mapping,
    )

    # --- Mode B1: SLAM Toolbox (Localization) ---
    slam_toolbox_loc = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        prefix=big_core_prefix,
        output='screen',
        condition=is_slam_loc,
        parameters=[
            slam_loc_params,
            {'use_sim_time': False, 'map_file_name': map_path},
        ],
    )

    configure_slam_loc = TimerAction(
        period=2.0,
        condition=is_slam_loc,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            ),
        ]
    )

    activate_slam_loc = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_loc,
            goal_state='inactive',
            entities=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                ),
            ],
        ),
        condition=is_slam_loc,
    )

    # --- Mode B2: Map Server + AMCL (Localization) ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        prefix=little_core_prefix,
        output='screen',
        condition=is_amcl,
        parameters=[nav2_params_file, {'yaml_filename': map_path}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        prefix=big_core_prefix,
        output='screen',
        condition=is_amcl,
        parameters=[nav2_params_file],
    )

    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        prefix=little_core_prefix,
        output='screen',
        condition=is_amcl,
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
    )

    # /cmd_vel -> ESP32 UART (ステアリング舵角 + モータ PWM フレーム)
    serial_bridge = Node(
        package='umeonigiri_serial_bridge',
        executable='umeonigiri_serial_bridge',
        name='umeonigiri_serial_bridge',
        prefix=little_core_prefix,
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud': serial_baud,
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'use_encoder_feedback': LaunchConfiguration('use_encoder_feedback'),
            'imu_use_differential_yaw': True,
            'publish_odom': True,
            'publish_odom_tf': True,
            'odom_rate_hz': 50.0,
        }],
    )

    controller_server_mppi = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        prefix=big_core_prefix,
        output='screen',
        condition=use_mppi,
        parameters=[nav2_params_file],
    )

    controller_server_rpp = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        prefix=big_core_prefix,
        output='screen',
        condition=use_rpp,
        parameters=[nav2_params_file, rpp_overlay_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        prefix=big_core_prefix,
        output='screen',
        parameters=[nav2_params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        prefix=little_core_prefix,
        output='screen',
        parameters=[nav2_params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        prefix=little_core_prefix,
        output='screen',
        parameters=[nav2_params_file, {'default_nav_to_pose_bt_xml': bt_xml_file}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        prefix=little_core_prefix,
        output='screen',
        parameters=[nav2_params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        prefix=little_core_prefix,
        output='screen',
        parameters=[nav2_params_file],
    )

    lidar = Node(
        package='umeonigiri_lidar',
        executable='d500_node',
        name='d500_node',
        prefix=little_core_prefix,
        output='screen',
        parameters=[{
            'port': lidar_port,
            'baudrate': 230400,
            'frame_id': 'lidar_link',
            'topic_name': '/scan',
            'odom_topic': '/odom',
            'range_min': 0.12,
            'speckle_filter_enabled': True,
            'deskew_enabled': True,
        }],
    )

    web_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_web_nav, 'launch', 'umeonigiri_webnav.launch.py')),
        launch_arguments={'node_prefix': little_core_prefix}.items(),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vision, 'launch', 'pc_vision.launch.py')),
        launch_arguments={
            'use_pc_camera': 'false',
            'backend': LaunchConfiguration('vision_backend'),
            'model_path': LaunchConfiguration('vision_model'),
            'inference_threads': LaunchConfiguration('vision_threads'),
            'output_fps': LaunchConfiguration('vision_output_fps'),
            'annotated_width': LaunchConfiguration('vision_annotated_width'),
            'npu_workers': LaunchConfiguration('vision_npu_workers'),
        }.items()
    )

    return LaunchDescription([
        omp_env,
        openblas_env,
        serial_port_arg,
        serial_baud_arg,
        lidar_port_arg,
        nav_mode_arg,
        localization_backend_arg,
        controller_profile_arg,
        rk3588_affinity_arg,
        map_arg,
        wheelbase_arg,
        max_linear_vel_arg,
        use_encoder_feedback_arg,
        vision_model_arg,
        vision_backend_arg,
        vision_threads_arg,
        vision_output_fps_arg,
        vision_annotated_width_arg,
        vision_npu_workers_arg,
        LogInfo(msg='=== Starting Real Robot Navigation (Orange Pi 5 Max / RK3588 Optimized) ==='),
        LogInfo(msg='Make sure your hardware (LD-D500 LiDAR, ESP32) is connected!'),

        robot_state_publisher,
        lidar,
        slam_toolbox_mapping,
        configure_slam_mapping,
        activate_slam_mapping,
        slam_toolbox_loc,
        configure_slam_loc,
        activate_slam_loc,
        map_server,
        amcl,
        lifecycle_manager_loc,

        serial_bridge,

        TimerAction(period=6.0, actions=[
            controller_server_mppi,
            controller_server_rpp,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            lifecycle_manager,
        ]),

        web_nav,
        vision,
    ])
