import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my_custom_world',
        description='Name of the Gazebo world file (without .sdf extension)'
    )

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

    # 4. Navigation (Nav2) - Delay to wait for SLAM
    navigation = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'navigation.launch.py'))
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        simulation,
        slam,
        navigation
    ])
