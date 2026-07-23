import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    nav2_params_file = os.path.join(pkg_nav, 'config', 'mapping_nav_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=[nav2_params_file]),
        # Delay lifecycle_manager so all managed nodes are initialized first
        # with sufficient bond_timeout for slow-starting nodes (e.g., velocity_smoother)
        TimerAction(period=5.0, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'bond_timeout': 10.0,
                              'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']}])
        ])
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes
    ])
