import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('umeonigiri_autonomy')
    nav2_params_file = os.path.join(pkg_nav, 'config', 'mapping_nav_params.yaml')
    default_map = os.path.join(pkg_nav, 'maps', 'nav_world.yaml')

    mppi_overlay = os.path.join(pkg_nav, 'config', 'mppi_overlay.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    # 制御器は比較のために切り替えられる。既定は RPP（設計 D-4 のベースライン）。
    controller = LaunchConfiguration('controller', default='rpp')
    use_rpp = IfCondition(PythonExpression(["'", controller, "' == 'rpp'"]))
    use_mppi = IfCondition(PythonExpression(["'", controller, "' == 'mppi'"]))

    def controller_node(context, extra, condition):
        params = [nav2_params_file] + (extra or [])
        remappings = []
        if context.launch_configurations.get('use_smoother_path', 'false') == 'true':
            remappings = [('cmd_vel', 'cmd_vel_nav')]
        return [Node(package='nav2_controller', executable='controller_server',
                     name='controller_server', output='screen', parameters=params,
                     remappings=remappings, condition=condition)]

    def rpp_controller(context):
        return controller_node(context, [], use_rpp)

    def mppi_controller(context):
        return controller_node(context, [mppi_overlay], use_mppi)

    def smoother_node(context):
        overlay = context.launch_configurations.get('nav_overlay', '')
        params = [nav2_params_file] + ([overlay] if overlay else [])
        remappings = []
        if context.launch_configurations.get('use_smoother_path', 'false') == 'true':
            # Jazzy の smoother は cmd_vel を購読し cmd_vel_smoothed を出す。
            remappings = [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        return [Node(package='nav2_velocity_smoother',
                     executable='velocity_smoother',
                     name='velocity_smoother', output='screen',
                     parameters=params, remappings=remappings)]

    nav2_nodes = [
        # 同名ノードを条件で切り替える。lifecycle_manager は名前で管理するので
        # 有効な方だけが管理対象になる。
        OpaqueFunction(function=rpp_controller),
        OpaqueFunction(function=mppi_controller),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params_file]),
        OpaqueFunction(function=smoother_node),
    ]

    lifecycle_manager_nav2 = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'bond_timeout': 10.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'behavior_server', 'bt_navigator',
                                    'velocity_smoother', 'map_saver']}])

    # ---- 保存地図での自己位置推定。起動時は inactive で待たせる ----
    map_server = LifecycleNode(
        package='nav2_map_server', executable='map_server', name='map_server',
        namespace='', output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml,
                                       'use_sim_time': use_sim_time}])

    amcl = LifecycleNode(
        package='nav2_amcl', executable='amcl', name='amcl', namespace='', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # 地図の保存は map_saver のサービス経由でしか行わない。
    map_saver = LifecycleNode(
        package='nav2_map_server', executable='map_saver_server', name='map_saver',
        namespace='', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'save_map_timeout': 30.0,
                     'map_subscribe_transient_local': True}])

    # map→odom を publish してよいのは常に1つ。切替はこのノードだけが行う。
    nav_mode_manager = Node(
        package='umeonigiri_autonomy', executable='nav_mode_manager',
        name='nav_mode_manager', output='screen',
        parameters=[{'slam_node': '/slam_toolbox',
                     'map_server_node': '/map_server',
                     'amcl_node': '/amcl',
                     'initial_mode': 'mapping'}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav_overlay', default_value='',
                              description='走行計測用の velocity_smoother 上書き yaml (空なら既定)'),
        DeclareLaunchArgument('use_smoother_path', default_value='false',
                              description='controller -> smoother -> /cmd_vel の経路を使う'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='保存地図モードで使う map yaml'),
        *nav2_nodes,
        lifecycle_manager_nav2,
        map_server,
        amcl,
        map_saver,
        nav_mode_manager,
    ])
