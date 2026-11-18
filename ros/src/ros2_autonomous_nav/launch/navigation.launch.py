import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    nav2_params_file = os.path.join(pkg_nav, 'config', 'mapping_nav_params.yaml')
    default_map = os.path.join(pkg_nav, 'maps', 'nav_world.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)

    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[nav2_params_file]),
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
        package='ros2_autonomous_nav', executable='nav_mode_manager',
        name='nav_mode_manager', output='screen',
        parameters=[{'slam_node': '/slam_toolbox',
                     'map_server_node': '/map_server',
                     'amcl_node': '/amcl',
                     'initial_mode': 'mapping'}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='保存地図モードで使う map yaml'),
        *nav2_nodes,
        lifecycle_manager_nav2,
        map_server,
        amcl,
        map_saver,
        nav_mode_manager,
    ])
