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
                                    'velocity_smoother']}])

    # ---- 保存地図での自己位置推定。起動時は inactive で待たせる ----
    map_server = LifecycleNode(
        package='nav2_map_server', executable='map_server', name='map_server',
        namespace='', output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml,
                                       'use_sim_time': use_sim_time}])

    amcl = LifecycleNode(
        package='nav2_amcl', executable='amcl', name='amcl', namespace='', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': False,
                     'bond_timeout': 10.0,
                     'node_names': ['map_server', 'amcl']}])

    # map→odom を publish してよいのは常に1つ。切替はこのノードだけが行う。
    nav_mode_manager = Node(
        package='ros2_autonomous_nav', executable='nav_mode_manager',
        name='nav_mode_manager', output='screen',
        parameters=[{'mapping_manager': '/lifecycle_manager_mapping',
                     'localization_manager': '/lifecycle_manager_localization',
                     'initial_mode': 'mapping'}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='保存地図モードで使う map yaml'),
        *nav2_nodes,
        lifecycle_manager_nav2,
        map_server,
        amcl,
        lifecycle_manager_localization,
        nav_mode_manager,
    ])
