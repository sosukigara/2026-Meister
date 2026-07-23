import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')

    # 1. Robot State Publisher (Xacro dynamic processing)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'robot_state_publisher.launch.py'))
    )

    # 2. Simulation (Gazebo + Bridge)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'simulation.launch.py'))
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

    return LaunchDescription([
        robot_state_publisher,
        simulation,
        slam,
        navigation
    ])
