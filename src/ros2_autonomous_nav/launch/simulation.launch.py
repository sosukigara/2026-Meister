import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my_custom_world',
        description='Name of the Gazebo world file (without .sdf extension)'
    )

    world_name = LaunchConfiguration('world')
    world_file = PathJoinSubstitution([pkg_nav, 'worlds', [world_name, '.sdf']])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
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
            ['/world/', world_name, '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            '/model/meistar_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/meistar_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/meistar_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            (['/world/', world_name, '/clock'], '/clock'),
            ('/model/meistar_bot/cmd_vel', '/cmd_vel'),
            ('/model/meistar_bot/odometry', '/odom'),
            ('/model/meistar_bot/tf', '/tf')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_robot,
        bridge
    ])
