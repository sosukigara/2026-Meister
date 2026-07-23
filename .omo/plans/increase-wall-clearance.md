# increase-wall-clearance - Work Plan

## TL;DR (For humans)
YAMLファイルのパラメータ3つを変更するだけで、ロボットが壁からより距離を取った経路を選ぶようになります。具体的には、`mapping_nav_params.yaml` の `cost_scaling_factor`（20.0→3.0）と `inflation_radius`（1.5→2.0）でコストマップの壁影響範囲を広げ、`BaseObstacle.scale`（0.02→5.0）でDWBの障害物回避を強化します。

**やりたくないこと**: 行動ロジックの変更、新機能追加、その他ノードの追加。

**作業量**: 1ファイル、4行の編集。再ビルドは不要（--symlink-install で config は symlink のため）。

## Scope
### In scope
- `mapping_nav_params.yaml` の3パラメータ（4箇所）の値変更
  - `local_costmap` inflation_layer: `cost_scaling_factor`, `inflation_radius`
  - `global_costmap` inflation_layer: `cost_scaling_factor`, `inflation_radius`  
  - `controller_server` → `FollowPath` → DWB: `BaseObstacle.scale`
- 変更後の動作確認（Nav2 goal 送信、壁からの距離を目視確認）

### Out of scope
- その他のナビゲーションパラメータの調整
- 新しいプラグイン/レイヤーの追加
- 起動ファイルの変更
- ソースコードの変更

## Verification strategy
1. システム起動後、全 lifecycle node が active [3] であることを確認
2. RViz で Nav2 goal を送信し、ロボットが実際に移動することを確認
3. 経路が以前より壁から離れていることを目視確認（RViz の経路表示）
4. 狭い通路も通れることを確認

## Execution strategy
1. `mapping_nav_params.yaml` の該当行を編集
2. `mapping_nav.launch.py` 経由でシステム起動（`colcon build` 不要 — symlink install のため）
3. RViz を起動
4. 複数の Nav2 goal を送信して挙動確認

## Todos

- [x] 1. `mapping_nav_params.yaml`: 4行のパラメータ変更を適用する
  - **References**: `/home/so/Meistar/src/ros2_autonomous_nav/config/mapping_nav_params.yaml`
  - **変更内容**:
    - Line 205: `cost_scaling_factor: 20.0` → `cost_scaling_factor: 3.0`（local_costmap inflation_layer）
    - Line 206: `inflation_radius: 1.5` → `inflation_radius: 2.0`（local_costmap inflation_layer）
    - Line 240: `cost_scaling_factor: 20.0` → `cost_scaling_factor: 3.0`（global_costmap inflation_layer）
    - Line 241: `inflation_radius: 1.5` → `inflation_radius: 2.0`（global_costmap inflation_layer）
    - Line 113: `BaseObstacle.scale: 0.02` → `BaseObstacle.scale: 5.0`（FollowPath DWB critic）
  - **Acceptance**: `grep -c "cost_scaling_factor: 3.0" config/mapping_nav_params.yaml` → 2, `grep -c "inflation_radius: 2.0"` → 2, `grep "BaseObstacle.scale: 5.0"` → 1
  - **QA (happy)**: 編集後、`colcon build --packages-select ros2_autonomous_nav --symlink-install` が成功すること
  - **QA (failure)**: 無効な YAML になっていないこと（`python3 -c "import yaml; yaml.safe_load(open('config/mapping_nav_params.yaml'))"` がエラーを返さない）
  - **Commit**: `config: increase wall clearance - cost_scaling_factor 20→3, inflation_radius 1.5→2, BaseObstacle.scale 0.02→5`

- [x] 2. システムを起動し、全 lifecycle node が active であることを確認する
  - **References**: `/home/so/Meistar/src/ros2_autonomous_nav/launch/mapping_nav.launch.py`
  - **手順**:
    1. `killall -9 gz-server rviz2 2>/dev/null; sleep 1`（既存プロセス停止）
    2. `unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH; source /opt/ros/jazzy/setup.bash && source /home/so/Meistar/install/local_setup.bash && ros2 launch ros2_autonomous_nav mapping_nav.launch.py &`（起動）
    3. 30秒待機（Nav2 15s + lifecycle 5s + 余裕）
    4. `ros2 lifecycle get /controller_server` → `active [3]`
    5. 同様に `/planner_server`, `/behavior_server`, `/bt_navigator`, `/velocity_smoother`, `/slam_toolbox` も確認
    6. `timeout 3 ros2 topic echo /tf --once 2>&1 | grep "frame_id: map"` → map→odom TF が確認できること
  - **Acceptance**: 全6ノードが `active [3]` を返し、map→odom TF が publish されている
  - **QA (happy)**: ノード一覧に全 Nav2 ノードと slam_toolbox が含まれる
  - **QA (failure)**: lifecycle_manager が "Managed nodes are active" とログに出力すること
  - **Commit**: （該当せず — 自動テスト用、コミット不要）

- [x] 3. RViz を起動し、Nav2 goal を送信して壁からの距離を確認する
  - **References**: rviz config at `/home/so/Meistar/src/ros2_autonomous_nav/config/nav2_rviz.rviz`
  - **手順**:
    1. RViz が起動していなければ起動：`rviz2 -d src/ros2_autonomous_nav/config/nav2_rviz.rviz &`
    2. ロボットをテレオペで初期位置から移動させる（必要に応じて）
    3. 経路計画が通る場所に goal を送信：`ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: -2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback`
    4. RViz で global plan を確認 — 壁から離れた経路になっていることを目視確認
    5. 狭い通路を通る goal もテスト（例：x: 0.0, y: -5.0）
  - **Acceptance**: goal が SUCCEEDED（error_code: 0）し、経路が壁から明らかに距離を取っている
  - **QA (happy)**: `distance_remaining` が減少し続け、最終的に 0 に近づく
  - **QA (failure)**: `error_code: 0` で SUCCEEDED すること
  - **Commit**: （該当せず — 検証ステップ）

## Final verification wave
- [ ] F1. plan compliance: 3つのパラメータが意図通り変更されていること
- [ ] F2. navigation test: ロボットが壁から距離を取った経路を通り、goal が SUCCEEDED すること
- [ ] F3. narrow passage test: 狭い通路も通れること（やむを得ない場合）

## Commit strategy
1 commit: `config: increase wall clearance - cost_scaling_factor 20→3, inflation_radius 1.5→2, BaseObstacle.scale 0.02→5`

## Success criteria
- マッピングモードで Nav2 goal が SUCCEEDED する
- 経路が以前より壁から離れている（RViz で確認）
- 狭い通路も通れる
- 全 lifecycle node が `active [3]` になる（既存の autostart 修正が維持されていること）
