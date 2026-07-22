"""
meister_slam_nav.launch.py
==========================
One-shot launch: Gazebo + SLAM + Nav2 + RViz for the Meister mecanum robot.

Thin wrapper around rosnav's slam_nav.launch.py that overrides robot-specific
paths and defaults for the meister robot platform.

Usage:
  ros2 launch meister_robot meister_slam_nav.launch.py [world_name:=maze] [explore:=true]

Arguments forwarded to slam_nav.launch.py:
  world_name  - Gazebo world name in rosnav worlds/  (default: maze)
  explore     - Auto-start frontier explorer          (default: true)
  slam        - true=SLAM Toolbox, false=AMCL         (default: true)
  rviz        - Launch RViz2                          (default: true)
  headless    - Skip Gazebo GUI + RViz                (default: false)
  safety      - Launch collision monitor              (default: true)

Notes:
  URDF, Nav2 params, and RViz config paths are passed as launch arguments
  for forward compatibility. These take effect when slam_nav.launch.py is
  updated to consume them from its argument interface.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_rosnav = get_package_share_directory('diff_drive_robot')
    pkg_meister = 'meister_robot'

    # ── Custom paths for the Meister robot ──────────────────────────────
    urdf_path = PathJoinSubstitution([
        get_package_share_directory(pkg_meister),
        'urdf', 'robot.urdf.xacro',
    ])

    params_file = PathJoinSubstitution([
        get_package_share_directory(pkg_meister),
        'config', 'nav2_params.yaml',
    ])

    rviz_config = PathJoinSubstitution([
        get_package_share_directory(pkg_meister),
        'rviz', 'meister_nav.rviz',
    ])

    # ── Launch arguments ────────────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='maze',
            description='Gazebo world name in rosnav worlds/ directory',
        ),
        DeclareLaunchArgument(
            'explore',
            default_value='true',
            description='Auto-start frontier explorer (SLAM mode only)',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='true=SLAM Toolbox (live mapping), false=AMCL on pre-built map',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Skip Gazebo GUI and RViz (server + nav only)',
        ),
        DeclareLaunchArgument(
            'safety',
            default_value='true',
            description='Launch collision monitor safety layer',
        ),
        DeclareLaunchArgument(
            'drive_type',
            default_value='mecanum',
            description='Drive base type passed to rosnav (mecanum)',
        ),

        # ── Include rosnav's slam_nav.launch.py with overrides ──────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rosnav, 'launch', 'slam_nav.launch.py'),
            ),
            launch_arguments={
                # Standard args that slam_nav.launch.py already supports:
                'world_name': LaunchConfiguration('world_name'),
                'explore':    LaunchConfiguration('explore'),
                'slam':       LaunchConfiguration('slam'),
                'rviz':       LaunchConfiguration('rviz'),
                'headless':   LaunchConfiguration('headless'),
                'safety':     LaunchConfiguration('safety'),
                # Custom overrides for the Meister robot (forward-compatible;
                # active once slam_nav.launch.py declares these arguments):
                'urdf':         urdf_path,
                'params_file':  params_file,
                'rviz_config':  rviz_config,
                'drive_type':   LaunchConfiguration('drive_type'),
            }.items(),
        ),
    ])
