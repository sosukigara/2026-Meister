import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    world_file = os.path.join(pkg_nav, 'worlds', 'my_custom_world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'meistar_bot', '-topic', '/robot_description', '-x', '-4.0', '-y', '-4.0', '-z', '0.1'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/my_custom_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/meistar_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/meistar_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/meistar_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/my_custom_world/clock', '/clock'),
            ('/model/meistar_bot/cmd_vel', '/cmd_vel'),
            ('/model/meistar_bot/odometry', '/odom'),
            ('/model/meistar_bot/tf', '/tf')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge
    ])
