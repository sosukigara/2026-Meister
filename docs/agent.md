# Project Context & Progress: Meistar ROS 2 Navigation

このドキュメントは、次回以降の作業を円滑に引き継ぐための現状サマリーです。

## 1. プロジェクト概要
*   **目的**: ROS 2を用いた自律移動ロボットの開発。SLAMによる地図作成とNav2による経路計画・走行を実現する。
*   **現在のフェーズ**: Gazeboシミュレーション環境での動作確認が完了し、実機への移行準備（設計・ドキュメント化）を開始した段階。

## 2. ワークスペースと主要ファイル
*   **メインディレクトリ**: `/home/so/Meistar`
*   **主要パッケージ**: `ros2-autonomous-nav`
*   **主要スクリプト**:
    *   `./scripts/start_mapping_nav.sh`: Gazebo + SLAM (slam_toolbox) + Nav2 を一括起動する。
    *   `./src/ros2_autonomous_nav/rviz.sh`: 設定済みのRViz2を起動する。
*   **ドキュメント**:
    *   `./docs/real_hardware_migration_guide.md`: 実機移行に向けたチェックリストと技術解説。
    *   `./docs/agent.md`: 本ドキュメント。

## 3. ロボットの構成・見た目 (URDF/Models)
ロボットの形状やセンサーの配置は以下のファイルで定義されています。

*   **メイン設計データ (Xacro)**:
    *   `/home/so/Meistar/src/meistar_description/urdf/robot.urdf.xacro`
    *   見た目（色・形）や物理特性のマスターデータです。
*   **ナビゲーション用展開データ (URDF)**:
    *   `/home/so/Meistar/src/ros2_autonomous_nav/urdf/my_robot_gpu.urdf`
    *   シミュレーション環境が直接参照しているファイルです。
*   **モデルデータ (Meshes)**:
    *   3Dモデル（STL等）を使用する場合は `/home/so/Meistar/src/meistar_description/meshes/` を作成して配置します。

---

## 4. 現在の状況 (Latest Status)
*   **シミュレーション**: Gazebo上でロボットがスポーンし、Lidarデータとオドメトリに基づいたマッピング・自律移動が正常に動作している。
*   **移行準備**: 実機に置き換える際に必要な「トピック（/scan, /odom, /cmd_vel）」と「TF（odom -> base_footprint）」の要件を整理済み。
*   **トラブルシューティング**: 実機で発生しがちな「LiDARの自己干渉（機体への映り込み）」への対策（最短距離制限 vs 角度制限）についてもドキュメント化済み。

## 4. 次回以降のタスク (Next Actions)
1.  **実機用パラメータの整備**:
    *   `config/real_nav2_params.yaml` 等を作成し、`use_sim_time: false` を設定する。
2.  **通信ブリッジの実装/起動**:
    *   STM32（モーター制御側）とのUDPブリッジを起動し、実機の `/odom` 生成と `/cmd_vel` の実機送信を確認する。
3.  **実機センサーの統合**:
    *   実機LiDAR（LakiBeam等）を起動し、`/scan` トピックが正しく出ているか確認する。
4.  **実機Launchの作成**:
    *   Gazeboを使わず、実機ハードウェアとNav2/SLAMを繋ぐための `real_mapping_nav.launch.py` を作成する。
5.  **現地チューニング**:
    *   実機の慣性やモーター特性に合わせた速度・加速度パラメータの追い込み。

---

## 6. URDFの編集ガイド (見た目と計算用データの変更)
ロボットのモデルを修正する際は、`/home/so/Meistar/original/src/meistar_description/urdf/robot.urdf.xacro` を以下の要領で編集します。

### A. 見た目を変える (`<visual>`)
*   **色を変える**: `<material>` タグの `rgba` (赤, 緑, 青, 透明度) を 0.0〜1.0 の範囲で変更します。
*   **サイズを変える**: `<geometry>` 内の `box size="横 幅 高"` や `cylinder radius="半径" length="長さ"` を変更します。
*   **3Dモデルを使う**: `<box>` などを消し、`<mesh filename="package://meistar_description/meshes/file.stl" scale="1 1 1"/>` に書き換えます。

### B. 計算用・物理データを変える (`<collision>`, `<inertial>`)
*   **当たり判定**: `<collision>` タグ内のサイズを編集します。基本的には `<visual>` と同じにします。
*   **重さと慣性**: `<inertial>` 内の `<mass value="質量(kg)"/>` を変更します。重心位置 (`origin`) や慣性モーメント (`inertia`) も実機に合わせて調整するとシミュレーションの挙動が正確になります。

### C. センサー位置と仕様を変える (`<joint>`, `<sensor>`)
*   **取り付け位置**: `<joint name="lidar_joint">` 内の `<origin xyz="x y z" rpy="roll pitch yaw"/>` を編集します。
    *   `xyz`: ロボット中心からの距離（メートル単位）。
    *   `rpy`: 取り付け角度（ラジアン単位。180度なら 3.14）。
*   **スキャン性能**: `<sensor>` タグ内の値を実機のカタログスペックに合わせます。
    *   `<update_rate>`: スキャン周期 (Hz)。
    *   `<samples>`: 1周のスキャン点数。
    *   `<range>`: 最小/最大検知距離 (`min`/`max`)。

**注意**: Xacroを編集した後は、パッケージを `colcon build` するか、展開後の `.urdf` ファイルに内容を反映させる必要があります。

## 5. 技術メモ (Agent Note)
*   **TF構成**: `map` -> `odom` -> `base_footprint` -> `lidar_frame` の繋がりを維持すること。
*   **自己位置推定**: 現在は `slam_toolbox` の `async` モードを使用。実機の計算リソースに応じて同期モードや `localization` モードへの切り替えを検討する可能性がある。
