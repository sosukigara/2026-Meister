import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    pkg_web_nav = get_package_share_directory('meister_web_nav')

    declare_world_arg = DeclareLaunchArgument('world', default_value='warehouse',
                                              description='World to load: warehouse, maze, empty')

    # 1. Robot State Publisher (Xacro dynamic processing)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'robot_state_publisher.launch.py'))
    )

    # 2. Simulation (Gazebo + Bridge)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'simulation.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 3. SLAM (slam_toolbox)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'slam.launch.py'))
    )

    # 4. Navigation (Nav2) — starts immediately alongside SLAM/Gazebo
    # Nav2 nodes are lifecycle-managed; they start unconfigured and wait.
    # lifecycle_manager discovers them via bond_timeout=10.0.
    # No artificial delay needed — overlap is handled by lifecycle transitions.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'navigation.launch.py'))
    )

    # 5. Web UI (map view + click-to-send waypoints) — http://localhost:8088
    web_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_web_nav, 'launch', 'web_nav.launch.py'))
    )

    return LaunchDescription([
        declare_world_arg,
        robot_state_publisher,
        simulation,
        slam,
        navigation,
        web_nav
    ])
