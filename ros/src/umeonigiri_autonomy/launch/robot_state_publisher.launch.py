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
    # 空文字なら各 Xacro 側の既定値を使う (R2: 設定は 1 箇所)。
    arm_camera_rate = LaunchConfiguration('arm_camera_rate', default='')
    arm_camera_width = LaunchConfiguration('arm_camera_width', default='')
    arm_camera_height = LaunchConfiguration('arm_camera_height', default='')
    base_camera_rate = LaunchConfiguration('base_camera_rate', default='')

    # Process xacro to urdf
    robot_description_config = Command(
        ['xacro ', xacro_file, ' sensor_profile:=', sensor_profile,
         ' arm_camera_rate:=', arm_camera_rate,
         ' arm_camera_width:=', arm_camera_width,
         ' arm_camera_height:=', arm_camera_height,
         ' base_camera_rate:=', base_camera_rate])
    
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
        DeclareLaunchArgument('arm_camera_rate', default_value='',
                              description='手先カメラの gz update_rate。空なら Xacro の既定値'),
        DeclareLaunchArgument('arm_camera_width', default_value='',
                              description='手先カメラの幅。空なら Xacro の既定値'),
        DeclareLaunchArgument('arm_camera_height', default_value='',
                              description='手先カメラの高さ。空なら Xacro の既定値'),
        DeclareLaunchArgument('base_camera_rate', default_value='',
                              description='base カメラの update_rate。空なら sensor_profile の既定値'),
        node_robot_state_publisher
    ])
