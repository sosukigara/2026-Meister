import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('umeonigiri_description')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    
    # センサの重さ。ソフトウェア描画 (llvmpipe) では lite にしないと
    # シミュレーションが実時間の数 % しか進まない。
    sensor_profile = LaunchConfiguration('sensor_profile', default='full')

    # Process xacro to urdf
    robot_description_config = Command(
        ['xacro ', xacro_file, ' sensor_profile:=', sensor_profile])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              description='full=640点/10Hz, lite=360点/5Hz (ソフトウェア描画向け)'),
        node_robot_state_publisher
    ])
