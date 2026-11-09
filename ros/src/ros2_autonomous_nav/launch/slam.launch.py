import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    pkg_nav = get_package_share_directory('ros2_autonomous_nav')
    slam_params_file = os.path.join(pkg_nav, 'config', 'mapper_params_online_async.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    configure_slam = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            ),
        ]
    )

    # configure が完了(inactive)してから activate する。固定秒数で activate を
    # 待つと configure 前に走り、SLAM が inactive のまま固まって
    # map フレームが出ず Nav2 の global_costmap が起動できなくなる。
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            goal_state='inactive',
            entities=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox,
        configure_slam,
        activate_slam
    ])
