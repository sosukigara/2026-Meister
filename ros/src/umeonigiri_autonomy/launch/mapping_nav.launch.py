import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def vision_launch():
    """シミュレーション用の画像認識。Gazebo カメラを読める ONNX 版だけを使う。

    NPU 版 (yolo_grasp) は /dev/video0 を直接開く実装で、Gazebo が配る
    /camera/image_raw を読めない。実機用の NPU を既定のまま選ぶと、
    シミュレーションでは PC カメラを見てしまい、モデルが無い環境では
    起動直後に落ちる。モデルは *.onnx が gitignore 対象なので、
    無い場合は落ちるノードを増やさず取得方法だけ伝える。
    """
    pkg_vision = get_package_share_directory('umeonigiri_vision')
    models = os.path.join(pkg_vision, 'models')
    for name in ('yolov8n-320.onnx', 'yolov8n.onnx'):
        model_path = os.path.join(models, name)
        if os.path.isfile(model_path):
            return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_vision, 'launch', 'pc_vision.launch.py')),
                launch_arguments={
                    'backend': 'onnx',
                    'model_path': model_path,
                    'use_pc_camera': 'false',
                    # シミュレーションは gz の描画も CPU で回している。ONNX を
                    # 4 スレッドにすると gz と食い合い、実時間比が 0.83 -> 0.64 に
                    # 落ちた (実測 lite/maze/320)。2 スレッドでもカメラ (5 Hz) に
                    # 追従できるので、ここだけ絞る。
                    'inference_threads': '2',
                }.items())
    return LogInfo(msg='[vision] ONNX モデルが無いため画像認識を起動しません。'
                       ' `ros2 run umeonigiri_vision download_model` で取得してください。')


def generate_launch_description():
    pkg_nav = get_package_share_directory('umeonigiri_autonomy')
    pkg_web_nav = get_package_share_directory('umeonigiri_webnav')

    declare_world_arg = DeclareLaunchArgument('world', default_value='warehouse',
                                              description='World to load: warehouse, maze, empty')
    declare_gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                            description='Gazebo GUI を出すか (false=ヘッドレス)')
    declare_profile_arg = DeclareLaunchArgument(
        'sensor_profile', default_value='full',
        description='センサの重さ。full=640点/10Hz、lite=360点/5Hz (ソフトウェア描画向け)')

    # 1. Robot State Publisher (Xacro dynamic processing)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'sensor_profile': LaunchConfiguration('sensor_profile')}.items()
    )

    # 2. Simulation (Gazebo + Bridge)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'simulation.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui')}.items()
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
        PythonLaunchDescriptionSource(os.path.join(pkg_web_nav, 'launch', 'umeonigiri_webnav.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'world': LaunchConfiguration('world')}.items()
    )

    # 6. Vision (Gazeboロボット目線カメラ+YOLOv8n検出。/camera/image_raw→/detections,/detection_image)
    vision = vision_launch()

    return LaunchDescription([
        declare_world_arg,
        declare_gui_arg,
        declare_profile_arg,
        robot_state_publisher,
        simulation,
        slam,
        navigation,
        web_nav,
        vision
    ])
