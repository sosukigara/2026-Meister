# Project Context & Progress: Meistar ROS 2 Navigation

このドキュメントは、作業の進捗と「モジュール化された新構成」を記録した引き継ぎ用サマリーです。

## 1. プロジェクトの現状
*   **フェーズ**: シミュレーション環境の高度なリファクタリングが完了。
*   **状態**: 
    *   URDFはXacroマクロにより部品化済み。
    *   Launchファイルは役割（Sim/SLAM/Nav2/RSP）ごとに分割され、メンテナンス性が高い状態。
    *   SLAM (slam_toolbox) と Nav2 が安定して動作することを確認済み。

## 2. ディレクトリ構造と主要コンポーネント

### 📦 ソースコード (`src/`)
*   **`meistar_description/urdf/`**: ロボットの定義
    *   `robot.urdf.xacro`: メイン。以下のパーツを統合。
    *   `chassis.xacro`: ロボット本体の形状・色。
    *   `lidar.xacro`: センサー設定（Gazeboプラグイン含む）。
    *   `gazebo.xacro`: シミュレーション用プラグイン（速度制御・オドメトリ）。
*   **`ros2_autonomous_nav/launch/`**: 起動設定（分割済み）
    *   `mapping_nav.launch.py`: **一括起動用メイン**。
    *   `robot_state_publisher.launch.py`: Xacroの動的処理（静的URDF不要）。
    *   `simulation.launch.py`: Gazebo + Bridge（通信）。
    *   `slam.launch.py` / `navigation.launch.py`: それぞれのスタックを起動。

### 📄 ドキュメント・スクリプト
*   `./scripts/start_mapping_nav.sh`: **【最重要】** ビルドから起動までを行うエントリーポイント。
*   `./docs/real_hardware_migration_guide.md`: 実機移行への技術メモ。
*   `./README.md`: ワークスペース全体の概要。

## 3. 次回へのヒント（プロンプト節約）
*   **見た目の変更**: `chassis.xacro` を編集するだけで即座に反映されます。
*   **Bridgeの仕様**: Gazebo側は `/model/meistar_bot/` という接頭辞が付く設定になっています。トピックが届かない場合は `simulation.launch.py` の remappings を確認してください。
*   **時間の同期**: 全てのLaunchで `use_sim_time:=true` がデフォルトになっています。実機移行時はこれを一括で `false` に切り替える必要があります。

---
**Status**: 🚀 Ready for next steps (Hardware integration or Visual customization).

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
