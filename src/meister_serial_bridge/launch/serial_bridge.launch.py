from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device connected to ESP32 UART (GPIO16/17)')

    declare_baud = DeclareLaunchArgument(
        'baud', default_value='115200')

    bridge = Node(
        package='meister_serial_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud': baud,
        }],
    )

    return LaunchDescription([
        declare_serial_port,
        declare_baud,
        bridge,
    ])
