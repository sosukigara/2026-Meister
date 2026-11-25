# アーム手先カメラの物体検出＋識別 30fps と SLAM 同時実行 — 実測記録

## 目的と判定

「手先カメラの ROS 画像トピック → NPU(RKNN) 検出／識別 → `/detections`」を
GViz GUI・RViz 無しのシミュレーションで 30fps 近傍で回し、同じセッションで
slam_toolbox を実時間比 0.8 以上で動かす構成。**全条件を実測で確認した**。

起動は 1 コマンド（worktree の `ros/` を cwd に、ROS と install を source 済み）:

```bash
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe   # P13: llvmpipe が必須
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 launch umeonigiri_autonomy arm_vision_slam.launch.py world:=maze sensor_profile:=lite
```

`gui` 引数はこの launch 内で常に `false`。RViz も Nav2 も Web UI も起動しない。
既存の `mapping_nav.launch.py` / `pc_vision.launch.py` の既定挙動は変更していない。

## 環境

| 項目 | 値 |
|---|---|
| 機体 | Orange Pi 5 Max / RK3588 / Ubuntu + ROS 2 Jazzy / Gazebo 8.15 |
| 描画 | llvmpipe（Mali/Panfrost の GL 3.3 では lidar が壊れる、P13） |
| モデル | `yolov8n-zoo.rknn`（Rockchip 最適化 yolov8n を INT8 変換、640x640 固定入力） |
| 推論 | `yolo_grasp_node`（C++ / librknnrt、int8 のままデコード） |
| 手先カメラ | 80x60 / gz `update_rate=45` sim Hz / rgb8 / `/arm_camera_image` → `/arm_camera/image_raw` |
| 物理 | `/tmp/meister_maze_step_0.01.sdf`（maze.sdf の `max_step_size` だけ 0.001→0.01） |
| base カメラ | 1 sim Hz（経路は残す。手先カメラに描画 CPU を集中させるため） |
| lidar | lite: 360 点 / 5 sim Hz |
| CPU | 8 コア。gz 3.60、NPU ノード 0.61、bridge 0.17、SLAM 0.07 コア（20 秒積分、[final_cpu.txt](logs/final_cpu.txt)） |

## 手先カメラの取り付け（URDF）

[sensor 定義](../../ros/src/umeonigiri_description/urdf/arm_camera.xacro) を
[robot.urdf.xacro](../../ros/src/umeonigiri_description/urdf/robot.urdf.xacro) から include している。

| 項目 | 値 |
|---|---|
| 親 → 子 | `base_link` → `arm_base_link` → `arm_upper_link` → `arm_forearm_link` → `arm_end_effector_link` → `arm_camera_link` |
| すべての関節 | `fixed`（ROS 側のアーム制御は未実装、実機の 4 軸アームを模した固定姿勢） |
| camera の親 | `arm_end_effector_link`（手先） |
| 位置 | 手先から `xyz="0.03 0 -0.01"` m |
| 向き | `rpy="0 -0.349 0"`（+X 前方、ピッチ -20° で卓上/床を見下ろす） |
| 解像度 / レート | 80x60 / 45 sim Hz（Xacro 既定は 320x240 / 40 Hz。起動引数で上書き） |
| gz / ROS トピック | `/arm_camera_image` → `/arm_camera/image_raw` |
| base カメラ | `/camera/image_raw` は従来どおり残す |

実機では ★要確認★ どのデバイス／マウントに相当するか（カメラの型番、USB バス、
アーム手先への固定具、焦点距離）は未確定。`/dev/video0` を手先カメラとして使う
想定だが、実機の base カメラと同時に繋ぐ場合はデバイス名の固定 (by-id) が必要。
数値はすべて上表のシミュレーション用の値であり、実機へ流用しないこと。

## 実測値（2026-11-25 / 20 秒連続計測）

| 条件 | 実測 | 判定 | 証拠 |
|---|---|---|---|
| `/arm_camera/image_raw` | **41.830 Hz**（20 秒 / window 848、末尾平均） | Measured（≥28 を満たす） | [final_camera_hz.txt](logs/final_camera_hz.txt) |
| `/detections` | **39.099 Hz**（20 秒 / window 873、末尾平均） | Measured（≥28 を満たす） | [final_detections_hz.txt](logs/final_detections_hz.txt) |
| `/clock` RTF | **0.905 / 0.925**（同一セッションで 20 秒 × 2 回） | Measured（≥0.8） | [final_measure_45.txt](logs/final_measure_45.txt) |
| `slam_toolbox` | `active [3]` | Measured | [final_slam_tf.txt](logs/final_slam_tf.txt) |
| `/scan` | 4.706 Hz（lite=5 sim Hz × RTF） | Measured | [final_scan_hz.txt](logs/final_scan_hz.txt) |
| `/map` | 1.000 Hz（`map_update_interval: 1.0`） | Measured | [final_map_hz.txt](logs/final_map_hz.txt) |
| TF | `map -> odom` が `tf2_echo` で取れる。`/tf_static` にアーム各 link | Measured | [final_slam_tf.txt](logs/final_slam_tf.txt)、[tf_static 抜粋](logs/map_tf_r8.txt) |
| ヘッドレス | `rviz2` なし。`gz sim -s -r ...` のみ（GUI プロセスなし） | Measured | [final_headless.txt](logs/final_headless.txt) |
| 二重配信 | `Publisher count: 1`（kill.sh → 再起動後も 1） | Measured | [final_slam_tf.txt](logs/final_slam_tf.txt) の直前の `topic info` |
| 識別 | 静止画像で `id='person' class_id='0'`、`id='bus' class_id='5'` | Measured | [static_detection_detail.txt](logs/static_detection_detail.txt) |
| ビルド / gtest | `./build.sh` 成功、`umeonigiri_vision` 82 tests 0 failures | Measured | 本ドキュメントの検証手順を再実行 |
| 既存 launch の回帰 | `mapping_nav.launch.py`（gui:=false/lite）で `/camera/image_raw` と `/arm_camera/image_raw` が各 1 publisher、`slam_toolbox` active、Web UI `/api/map` が HTTP 200 | Measured | [regression_check.txt](logs/regression_check.txt) |

`/detections` がこの回帰起動で無いのは、worktree に ONNX モデルが無いため既存の
`vision_launch()` が検出ノードを起動しない挙動（P19）そのものであり、回帰ではない。
NPU 実機ノードの V4L2 入力経路は実カメラが無いため ✗ Unverified（静止画像経路と
トピック経路は実測済み）。

`/detections` の `id` にクラス名、`results[0].hypothesis.class_id` に数値 ID を入れる。
Web UI は `id` を表示ラベルとして使うので、どちらかだけでは認識モニターが
`unknown` になる（[detection_msg.hpp](../../ros/src/umeonigiri_vision/include/umeonigiri_vision/detection_msg.hpp)）。

## 画像入力と無駄の除去

- gz bridge は `rgb8` を出す。[image_util.cpp](../../ros/src/umeonigiri_vision/src/image_util.cpp) の
  `BgrFromImageMsg` で BGR に変換する（rgb8/rgba8/mono8 等に対応、step のパディングも処理）。
- 購読はメンバで保持（`create_subscription` の戻りを捨てない、P12）。
- トピック入力では推論スレッドが毎フレーム `/detections` を出し、
  `/detection_image` と `/grasp_image` は**購読者がいるときだけ**描画する。
- V4L2 直接入力 (`/dev/video0`) は既定のまま残す。シミュレーションでは
  `use_topic_input:=true` と `image_topic:=/arm_camera/image_raw` を明示して選ぶ。
- `detection_fps` ゲートは汎用機能として残しテストもあるが、この構成では 0（無効）。
  推論が ~25 ms のとき 30 Hz ゲートが 1 フレームおきに捨てて 18.6 Hz に落ちたため。

## 試した代替手段と失敗

| 試行 | 結果 | 判定 |
|---|---|---|
| 320x240 / 40 sim Hz / 1ms step | camera 24.5 Hz、detections 19.1 Hz、RTF 0.521 | ✗ 不採用（RTF とレートが共に不足） |
| 160x120 / 40 sim Hz / 1ms step | camera 29.6 Hz、RTF 0.631、detections 18.6 Hz（ゲートが律速） | ✗ 不採用 |
| `gz sim -z 250` で物理を間引く | camera 9.6 Hz に低下（センサ更新まで遅くなる） | ✗ 不採用 |
| 30 Hz publish gate を有効 |推論 ~25 ms に対し 1 帧おきに破棄 → detections 18.6 Hz | ✗ 不採用 |
| 80x60 / 50 sim Hz / 4ms step | camera 41.9、detections 38.4 Hz、RTF 0.789 | ✗ RTF が 0.8 未満 |
| 80x60 / 50 sim Hz / 8ms step | RTF 0.800 ぎりぎり、再現性が不安定 | ✗ 不採用 |
| **80x60 / 45 sim Hz / 10ms step** | **camera 41.8、detections 39.1 Hz、RTF 0.905 / 0.925** | **Confirmed（採用）** |

## 未検証・注意

- maze ワールドの手先カメラ映像は暗く、実行中の `/detections` は空配列だった。
  識別結果そのものは静止画像（bus.jpg）で確認済み。シミュレーション内の
  物体が手先カメラに正しく写るかは ★要確認★（照明／物体配置の問題）。
- INT8 量子化でクラス確信度が 0.5 に張り付く既知の問題は残る（tools/grasp/README.md）。
  明るい画像での再キャリブは ★要確認★。
- 実機の手先カメラ性能・マウント・デバイス名は ★要確認★（上表）。
- シミュレーションの CPU 値は `/clock` 配信量に比例するため実機見積りに流用しない（P13）。

## 検証手順（再現）

```bash
# ビルド & テスト（worktree 内）
PATH=/usr/bin:/bin:/usr/local/bin ./build.sh
cd ros && PATH=/usr/bin:/bin:/usr/local/bin bash -c \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   colcon test --packages-select umeonigiri_vision && colcon test-result --verbose'

# 計測（別シェル。launch はバックグラウンド）
cd /home/so/Meistar/.worktrees/arm-cam-30fps-slam
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=1 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 topic hz /arm_camera/image_raw     # 20 秒
ros2 topic hz /detections               # 20 秒
python3 docs/perf/measure_rtf.py --seconds 20
ros2 lifecycle get /slam_toolbox
ros2 topic hz /scan && ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
pgrep -a rviz2; pgrep -af 'gz sim'      # rviz2 なし / -s のみ
./kill.sh                               # 後始末
```
