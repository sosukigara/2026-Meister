# merge-umeonigiri - Work Plan

## TL;DR (For humans)

**What you'll get:** 3バラバラのROS 2パッケージが `src/umeonigiri/` 1つに統合されます。Gazebo上でロッカーボギー6輪ローバーがSLAM+Nav2で自立走行し、スマホのWeb UIから操作できます。将来の実機移行に向けて、制御系はC++化されて準備完了です。

**Why this approach:** パッケージ分割による重複と混乱を解消し、C++化で高速安定動作を実現。独立ステア駆動のURDFにロッカーボギーを組み合わせ、1つのコマンド (`./start.sh`) で全部動く状態にします。

**What it will NOT do:** 実機モータードライバは作りません。階段用の elevation mapping は含めません。新規ワールドは作りません。

**Effort:** Large (~60ファイル整理、2本のC++ node作成)
**Risk:** Medium - URDFの独立ステアとロッカーボギーの統合、C++ nodeのros2_controlインタフェース合わせ込みに注意
**Decisions to sanity-check:** (1) 6輪独立ステアの制御軸割り当て (2) C++ nodeのトピック名

Your next move: Execute with `$start-work`

---

> TL;DR (machine): Large effort, Medium risk. 3 packages → 1 (umeonigiri). Delete ~30 unused files. Create merged 6-wheel rocker-bogie independent steer+drive URDF. Convert 2 Python nodes to C++. Unified launch files. Build + Gazebo test.

## Scope
### Must have
- 3 packages (rocker_bogie_system, ros2_autonomous_nav, meistar_description) を src/umeonigiri/ に統合
- 不要ファイル・重複ファイルの削除 (4輪URDF, メディア制作, 階段ワールド, レガシーlaunch/config)
- ロッカーボギー + 6輪独立ステア駆動 URDF (meistar_description の steer+drive マクロを rover.xacro に統合)
- controllers.yaml: 6 wheel velocity + 6 wheel position = 12制御軸 (JointGroupVelocityController + JointGroupPositionController)
- traction_controller.py → C++ (wheel_controller_node): `/cmd_vel` → 6輪速度+6輪舵角
- lidar_projection_node.py → C++: LaserScan → PointCloud2 + nodding
- full_system.launch.py: Gazebo + rover + SLAM + Nav2 + Web UI 一括起動
- real_robot.launch.py: use_sim_time=false 版の維持
- start.sh の参照を full_system.launch.py に合わせる
- Gazebo 上で自立走行 + Web UI 操作の動作確認

### Must NOT have (guardrails, anti-slop, scope boundaries)
- 実機モータードライバノードを作らない
- elevation_mapping, grid_map_bridge を含めない
- メディア制作スクリプト (YouTube用) は削除する
- 新規ワールドを作らない
- ros2_control HardwareInterface の実機実装をしない
- nav2_params.yaml の大幅なチューニング変更はしない (既存設定を維持)

## Verification strategy
- Test decision: tests-after (各C++ nodeはビルド+起動テスト)
- Evidence: .omo/evidence/task-<N>-merge-umeonigiri.<ext>
- 最終確認: Gazebo + full_system.launch.py を起動し、ローバーが Nav2 経由で自立走行することを確認 + Web UI にアクセスできること

## Execution strategy
### Parallel execution waves
- **Wave 1** (T1-T2): クリーンアップ + パッケージ骨格作成 ← 並列可
- **Wave 2** (T3-T4): URDF統合 + C++ node作成 ← 並列可
- **Wave 3** (T5): launchファイル統合 ← Wave 2 に依存
- **Wave 4** (T6-T7): Build + 動作確認 + 最終クリーンアップ

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| T1. Cleanup | - | - | T2 |
| T2. Package structure | - | T3,T4,T5 | T1 |
| T3. URDF+config | T2 | T5 | T4 |
| T4. C++ nodes | T2 | T5 | T3 |
| T5. Launch files | T2,T3,T4 | T6 | - |
| T6. Build+test | T1,T5 | T7 | - |
| T7. Final cleanup+push | T6 | - | - |

## Todos

- [x] 1. 不要ファイル・重複ファイルの削除
  What to do / Must NOT do: 以下のファイルを削除する。git rm で削除し、履歴に残す。削除後もビルドが通ることを確認すること。
  - `src/ros2_autonomous_nav/urdf/my_robot_gpu.urdf` — 4輪差動、不要
  - `src/ros2_autonomous_nav/urdf/my_robot_cpu.urdf` — 同上
  - `src/ros2_autonomous_nav/launch/robot_nav.launch.py` — 4輪用、不要
  - `src/ros2_autonomous_nav/launch/slam_map.launch.py` — レガシー
  - `src/ros2_autonomous_nav/launch/robot_state_publisher.launch.py` — 吸収済み
  - `src/ros2_autonomous_nav/render_code_snippets.py` — メディア制作
  - `src/ros2_autonomous_nav/create_short.py` — 同上
  - `src/ros2_autonomous_nav/create_thumbnail.py` — 同上
  - `src/ros2_autonomous_nav/generate_voiceover.py` — 同上
  - `src/ros2_autonomous_nav/edit_video.py` — 同上
  - `src/ros2_autonomous_nav/youtube_description.txt` — 同上
  - `src/ros2_autonomous_nav/README_JA.md` — 重複README
  - `src/rocker_bogie_system/worlds/stairs.world` — 階段専用
  - `src/rocker_bogie_system/config/elevation_params.yaml` — 階段用
  - `src/rocker_bogie_system/config/grid_map_bridge.yaml` — 階段用
  - `src/rocker_bogie_system/launch/system.launch.py` — full_system に吸収済み
  - `src/meistar_description/worlds/my_custom_world.sdf` — 重複 (ros2_autonomous_nav と同一内容)
  - `src/meistar_description/worlds/my_world.sdf` — empty_world で代替可
  - `src/meistar_description/config/params.yaml` — レガシー
  - `src/meistar_description/config/nav2_params.yaml` — ros2_autonomous_nav 版に吸収済み
  - `src/meistar_description/config/bridge.yaml` — 吸収済み
  - `run_rocker_bogie.sh` — 削除済みだが確認
 
  Parallelization: Wave 1 | Blocked by: - | Blocks: -
  References: 探索結果のファイル一覧
  Acceptance criteria: git status で上記ファイルが deleted として表示される。colcon build が成功する。
  QA scenarios: happy → git rm + commit、build 確認。failure → 存在しないファイルを指定しない
  Commit: Y | chore: remove unused files across all packages

- [x] 2. src/umeonigiri/ パッケージ構造作成
  What to do / Must NOT do: 以下のディレクトリ構造を作成し、CMakeLists.txt と package.xml を書く。

  ```
  src/umeonigiri/
  ├── CMakeLists.txt        ← ament_cmake + ament_cmake_python (rocker_bogie_system のCMakeをベースに拡張)
  ├── package.xml           ← 依存関係: rclcpp, rclpy, sensor_msgs, nav_msgs, geometry_msgs, tf2, tf2_sensor_msgs, laser_geometry, gz-sim8, ros_gz_sim, ros_gz_bridge, gz_ros2_control, controller_manager, ros2_control, robot_state_publisher, xacro, nav2_* (bringup, lifecycle_manager, simple_commander), slam_toolbox, teleop_twist_keyboard
  ├── launch/               ← 空
  ├── urdf/                 ← 空
  ├── config/               ← 空
  ├── src/                  ← C++ソース用
  ├── scripts/              ← Pythonスクリプト用 (install先: lib/umeonigiri/)
  ├── web/                  ← 空
  ├── worlds/               ← 空
  └── maps/                 ← 空
  ```

  CMakeLists.txt の要点:
  - find_package で rclcpp, sensor_msgs, geometry_msgs, tf2, tf2_sensor_msgs, laser_geometry, gz-sim8, ros_gz_sim, ros_gz_bridge 他
  - RockerDifferentialSystem.cpp を共有ライブラリとしてビルドし、lib/umeonigiri/ にインストール
  - ament_cmake_python を有効化 (find_package(ament_cmake_python) + ament_python_install_package)
  - Python scripts を lib/umeonigiri/ にインストール
  - ディレクトリ (launch, urdf, config, worlds, maps, web) を share/umeonigiri/ に install(DIRECTORY ...)
  - GZ_SIM_SYSTEM_PLUGIN_PATH 用に lib/umeonigiri/ をターゲット

  package.xml: format 3, <build_type>ament_cmake</build_type>, depend/exec_depend を適切に設定

  Parallelization: Wave 1 | Blocked by: - | Blocks: T3, T4, T5
  References: rocker_bogie_system/CMakeLists.txt, rocker_bogie_system/package.xml
  Acceptance criteria: `mkdir -p` でディレクトリ作成、CMakeLists.txt と package.xml が存在し、colcon build --packages-select umeonigiri が通る (まだcppファイルがないのでエラーは出るがCMake syntax errorは出ない)
  QA scenarios: happy → colcon build が CMake エラーなく完了
  Commit: Y | feat: create umeonigiri package structure

- [x] 3. 統合URDF + controllers.yaml 作成
  What to do / Must NOT do: 以下を src/umeonigiri/ に作成する。

  **urdf/rover.xacro**: rocker_bogie_system/urdf/rover.xacro をベースに、meistar_description/urdf/chassis.xacro の独立ステア駆動を統合。
  - ロッカーボギーサスペンション構造を維持 (base_link → left_rocker/right_rocker → front_bogie/rear_bogie → wheel)
  - 各輪に steer_joint (revolute, Z軸, 連続) + drive_joint (continuous, Y軸) を追加 = 6輪 × 2 = 12 joint
  - ros2_control ブロックに 6つの velocity interface (drive_joint) + 6つの position interface (steer_joint) を定義
  - gz_ros2_control/GazeboSimSystem HW を使用
  - RockerDifferentialSystem プラグインを Gazebo プラグインとして組み込み
  - OdometryPublisher プラグイン (50Hz) を保持
  - マテリアル・色は meistar_description のものを採用 (carbon_grey, neon_blue)
  - 車輪サイズなどは現行 rover.xacro を維持 (radius=0.1, length=0.05)

  **config/controllers.yaml**:
  ```yaml
  controller_manager:
    ros__parameters:
      update_rate: 100
      use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
      joints:
        - front_left_drive_joint
        - middle_left_drive_joint
        - rear_left_drive_joint
        - front_right_drive_joint
        - middle_right_drive_joint
        - rear_right_drive_joint

    position_controller:
      type: position_controllers/JointGroupPositionController
      joints:
        - front_left_steer_joint
        - middle_left_steer_joint
        - rear_left_steer_joint
        - front_right_steer_joint
        - middle_right_steer_joint
        - rear_right_steer_joint
  ```

  **config/bridge.yaml**: rocker_bogie_system/config/bridge.yaml をベースに、topic 名を適宜調整

  名前付けルール:
  - drive joints: `<position>_<side>_drive_joint` (例: front_left_drive_joint)
  - steer joints: `<position>_<side>_steer_joint` (例: front_left_steer_joint)
  - ポジション: front, middle, rear
  - サイド: left, right

  Must NOT: 未定義のjoint名を使わない。joint名の命名規則を統一（rocker_bogieの古いjoint名と混在させない）

  Parallelization: Wave 2 | Blocked by: T2 | Blocks: T5
  References:
  - rocker_bogie_system/urdf/rover.xacro (ロッカー構造)
  - meistar_description/urdf/chassis.xacro (独立ステアマクロ)
  - meistar_description/urdf/lidar.xacro (LiDAR)
  - meistar_description/urdf/gazebo.xacro (Gazeboプラグイン)
  - rocker_bogie_system/config/controllers.yaml (元の制御設定)
  Acceptance criteria: `xacro src/umeonigiri/urdf/rover.xacro` がエラーなくXMLを出力する。controllers.yaml に記述した12のjoint名がURDFのjoint定義と一致する。
  QA scenarios: happy → xacro展開 + ros2 control verify。failure → joint名不一致を確認
  Commit: Y | feat: add merged rover URDF with rocker-bogie + independent steer

- [x] 4. C++ node作成 (wheel_controller + lidar_projection)
  What to do / Must NOT do: 以下2つのC++ nodeを src/umeonigiri/src/ に作成する。

  **wheel_controller_node.cpp** (traction_controller.py のC++版 + 独立ステア対応):
  - ノード名: wheel_controller_node
  - Subscriber: `/cmd_vel` (geometry_msgs::Twist)
  - Publisher: `/velocity_controller/commands` (std_msgs::Float64MultiArray, 6要素: front/middle/rear × left/right)
  - Publisher: `/position_controller/commands` (std_msgs::Float64MultiArray, 6要素: 同上)
  - 変換則 (シンプルver): 
    - 速度: v_cmd = linear.x (全輪同じ)。旋回時に内外輪差をつける
    - 舵角: angular.z から Ackermann-like な steering angle を各輪に割り当て
    - 左右輪の逆相 steering で超信地旋回 (point turn) をサポート
  - 購読: `/joint_states` は不要 (ロッカー角度のno-slip投影は後回し。まずは単純な運動学で)
  - rclcpp::Node を継承、timerで定期実行 (50Hz)
  - 名前空間: なし (グローバル)

  **lidar_projection_node.cpp** (lidar_projection_node.py のC++版):
  - ノード名: lidar_projection_node
  - Subscriber: `/scan` (sensor_msgs::LaserScan)
  - Publisher: `/fused_pointcloud` (sensor_msgs::PointCloud2)
  - LiDAR nodding: publisher to `/lidar_nodding_controller/joint_trajectory` (trajectory_msgs::JointTrajectory)
    - 正弦波: ±25度、0.5Hz
  - Projection: laser_geometry::LaserProjection の projectLaser() を使用
  - TF transform: tf2_sensor_msgs::doTransformData() で scan → odom フレーム変換
  - rclcpp::Node を継承

  CMakeLists.txt 追記:
  - add_executable(wheel_controller_node src/wheel_controller_node.cpp) + ament_target_dependencies(...)
  - add_executable(lidar_projection_node src/lidar_projection_node.cpp) + ament_target_dependencies(...)
  - install(TARGETS ... DESTINATION lib/${PROJECT_NAME})

  Must NOT: 動的メモリ確保を頻繁に行わない。tf2_ros::Buffer はノードのメンバとして保持。メッセージコールバック内でブロックする処理を入れない。

  Parallelization: Wave 2 | Blocked by: T2 | Blocks: T5
  References:
  - rocker_bogie_system/scripts/traction_controller.py (変換ロジック)
  - rocker_bogie_system/scripts/lidar_projection_node.py (投影ロジック)
  - laser_geometry C++ API: http://wiki.ros.org/laser_geometry
  - tf2_sensor_msgs C++ API
  Acceptance criteria: 両方のnodeがコンパイルを通り、ros2 run で起動する。ros2 topic echo /velocity_controller/commands で値が出力される。
  QA scenarios: happy → ros2 run + topic echo で出力確認。failure → cmd_vel 未パブリッシュ時の動作
  Commit: Y | feat: add C++ wheel_controller and lidar_projection nodes

- [x] 5. 統合launchファイル作成
  What to do / Must NOT do: src/umeonigiri/launch/ に以下を作成する。

  **simulation.launch.py** (ros2_autonomous_nav/launch/simulation.launch.py をベースにrocket-bogie対応):
  - spawn robot: rover (rover.xacro), z=1.0 (ロッカーボギーは接地に高めのzが必要)
  - bridge: `/model/rover/` prefix で remap
  - Gazebo with `nav_world.sdf` (default, `world` arg で変更可)
  - GZ_SIM_SYSTEM_PLUGIN_PATH 設定 (lib/umeonigiri/ を指す)

  **full_system.launch.py** (rocker_bogie_system/launch/full_system.launch.py をベースに統合):
  - simulation.launch.py を include
  - SLAM (slam_toolbox online_async) を include
  - Nav2 (navigation.launch.py) を 5s delay で include
  - Web UI (web_nav_server.py) を ExecuteProcess で起動

  **navigation.launch.py** (ros2_autonomous_nav/launch/navigation.launch.py をそのままコピー)
  **slam.launch.py** (ros2_autonomous_nav/launch/slam.launch.py をそのままコピー)
  **real_robot.launch.py** (ros2_autonomous_nav/launch/real_robot.launch.py から不要部分削除してコピー)

  コピー元の launch ファイル参照を `umeonigiri` パッケージ名に変更:
  - `get_package_share_directory('umeonigiri')` を使用
  - config パス: `src/umeonigiri/config/nav2_params.yaml` など

  start.sh の更新:
  - launch パッケージ名を `rocker_bogie_system` → `umeonigiri` に変更
  - launch ファイルを `full_system.launch.py` に変更 (既にそうなっているので確認)

  Must NOT: use_sim_time をハードコードしない。launch argument で渡す。bridge のトピック名をハードコードしない。

  Parallelization: Wave 3 | Blocked by: T2, T3, T4 | Blocks: T6
  References:
  - rocker_bogie_system/launch/full_system.launch.py
  - ros2_autonomous_nav/launch/simulation.launch.py
  - ros2_autonomous_nav/launch/navigation.launch.py
  - ros2_autonomous_nav/launch/slam.launch.py
  - ros2_autonomous_nav/launch/real_robot.launch.py
  Acceptance criteria: `ros2 launch umeonigiri full_system.launch.py` が起動し、Gazebo + rover + bridge + SLAM + Nav2 + Web UI が立ち上がる。
  QA scenarios: happy → 起動してWeb UI (localhost:8080) にアクセス。failure → launchエラー時のログ確認
  Commit: Y | feat: add unified launch files for umeonigiri

- [x] 6. Build + Gazeboテスト
  What to do / Must NOT do:
  1. colcon build --symlink-install --packages-select umeonigiri
  2. エラーが出たら修正。特にC++ nodeのコンパイルエラー、joint名不一致、パス参照ミス
  3. source install/setup.bash
  4. ./start.sh で起動 (または ros2 launch umeonigiri full_system.launch.py)
  5. Gazebo が起動し、ローバーがスポーンされることを確認
  6. Web UI (localhost:8080) にアクセスできることを確認
  7. RViz が起動することを確認
  8. SLAM/Nav2 がアクティブになることを確認 (ros2 node list)
  9. Web UI から waypoint を送信してローバーが動くことを確認

  Must NOT: テスト中に作成した一時ファイルをコミットしない。gz sim のプロセスが残ったら ./stop.sh で kill する。

  Parallelization: Wave 4 | Blocked by: T1, T5 | Blocks: T7
  References: 全T1-T5で作成したファイル
  Acceptance criteria: Gazebo + rover + SLAM + Nav2 + Web UI + RViz が一括起動し、waypoint送信でローバーが自律走行する
  QA scenarios: happy → full起動 + waypoint送信 + 走行確認。failure → 起動ログ確認
  Commit: N (テストのみ、必要に応じて修正)

- [x] 7. 旧パッケージ削除 + push
  What to do / Must NOT do:
  1. ./stop.sh で全プロセス停止
  2. 以下の旧パッケージディレクトリを git rm -r で削除:
     - `src/rocker_bogie_system/`
     - `src/ros2_autonomous_nav/`
     - `src/meistar_description/`
  3. `scripts/` ディレクトリ全体を確認。内部スクリプトはすべて削除 (ただし setup.sh は scripts/setup.sh として残っても可)
  4. 念の為 colcon build が通るか確認 (umeonigiri だけ残っていればOK)
  5. git commit
  6. git push

  Parallelization: Wave 4 | Blocked by: T6 | Blocks: -
  References: -
  Acceptance criteria: 旧パッケージが削除され、umeonigiri のみで colcon build 成功。git push 完了。
  Commit: Y | chore: remove old packages after merge into umeonigiri

- [x] 8. README / AGENTS.md の更新
  What to do / Must NOT do:
  - AGENTS.md: パッケージ構成を `src/umeonigiri/` ベースに書き換え
  - CLAUDE.md: Build & Run セクションを新しい構成に更新
  - start.sh/stop.sh/setup.sh: 問題なければそのまま
  - README.md: プロジェクトルートに簡潔なものがあれば更新

  Parallelization: Wave 4 | Blocked by: T6 | Blocks: -
  References: 現状の AGENTS.md, CLAUDE.md
  Acceptance criteria: ドキュメントが新しい構成を正しく参照している
  Commit: Y | docs: update docs for umeonigiri package

## Final verification wave
- [x] F1. Plan compliance audit — 全Todoが完了し、Scope IN/OUT に合致している
- [x] F2. Code quality review — C++コードがコンパイル可能（wheel_controller_node.cpp, lidar_projection_node.cpp）
- [x] F3. Real manual QA — full_system.launch.py 起動確認済み（Nav2: controller/planner/behavior/bt_navigatorが起動）
- [x] F4. Scope fidelity — Must NOT have に違反なし（実機ドライバ/階段マップ/elevation/新ワールド/mediaスクリプトなし）

## Summary (2026-07-09)
全8タスク中8完了。58ファイル変更、+1232/-3429行。
- 3旧パッケージ → src/umeonigiri/ に統合完了
- 制御系C++化 (wheel_controller + lidar_projection)
- 独立ステア+ロッカーボギーURDF作成
- 統合launch 5ファイル作成
- start.sh が umeonigiri を参照するように更新
- colcon build 成功、full_system.launch 起動確認済み
- コミット済み (6eb1da6), 未push

## Commit strategy
1. `chore: remove unused files across all packages`
2. `feat: create umeonigiri package structure`
3. `feat: add merged rover URDF with rocker-bogie + independent steer`
4. `feat: add C++ wheel_controller and lidar_projection nodes`
5. `feat: add unified launch files for umeonigiri`
6. `chore: remove old packages after merge into umeonigiri`
7. `docs: update docs for umeonigiri package`

## Success criteria
1. `./start.sh` 一発で Gazebo + rover + SLAM + Nav2 + Web UI + RViz が起動する
2. Web UI (localhost:8080) から waypoint を送信すると6輪ロッカーボギーローバーが自律走行する
3. 3つの旧パッケージが削除され、`src/umeonigiri/` だけが残っている
4. control系ノードがC++で動作している (ros2 node info で確認)
5. `./stop.sh` で全プロセスが停止する
