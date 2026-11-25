# 手先カメラ＋NPU 検出の「非空」実測（ヘッドレス SLAM 同時実行）

## 目的と判定

前回の [arm-cam-30fps-slam.md](arm-cam-30fps-slam.md) で 39.1 Hz の `/detections` は
実測済みだったが、メッセージの `detections` は空配列だった。ここでは
**ヘッドレス SLAM を止めずに、クラス名付きの非空検出が 28 Hz 以上で継続する**
ことを実測で示す。

**判定: 全条件 Measured / Confirmed（下記）。画像の出所は (b) テスト用対象。**
実機の手先カメラ認識は ★要確認★ のまま（後述）。

## 画像の出所（正直な記録）

条件 2 の選択は **(b) テスト用にワールドへ追加した_detections可能なテクスチャ対象** である。

- 入力は Gazebo の手先カメラ `/arm_camera/image_raw` そのもの（80x60 / rgb8）で、
  ROS 画像を NPU ノードが購読している。画像リプレイ (c) は使っていない。
- ただしカメラが映しているのは自然な施工现场ではなく、
  `vision_target:=true` のときだけ `/tmp` の SDF コピーへ差し込む
  **テスト用の平面** [vision_target.sdf](../../ros/src/umeonigiri_autonomy/worlds/vision_target.sdf) である。
  平面には [detection_target.jpg](../../ros/src/umeonigiri_autonomy/worlds/detection_target.jpg)
  （[docs/perf/bus.jpg](bus.jpg) のコピー）をテクスチャとして貼り、
  点光源で照らしている。
- したがって **「実カメラが現場の物体を認識した」という主張ではない**。
  これは NPU トピック入力 → 検出 → 識別 → 配信の経路と、SLAM との同時実行を
  再現可能に計測するための対象である。
- 実機の手先カメラ（型番／マウント／`/dev/video0` の割当て／焦点距離）および
  実機映像での検出性能は ★要確認★（前回ドキュメントのまま変更なし）。
  NPU の V4L2 直接入力経路も実カメラが無いため ✗ Unverified。

## 1 コマンド起動

```bash
cd /home/so/Meistar/.worktrees/arm-cam-live-detections   # worktree 内
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe   # llvmpipe 必須 (P13)
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 launch umeonigiri_autonomy arm_vision_slam.launch.py \
    world:=maze sensor_profile:=lite vision_target:=true
```

`vision_target` は既定 `false`。省略すれば前回と同じ maze／同じ挙動
（`/tmp/meister_maze_step_0.01.sdf`、検出は空配列）になる。
RTF は実測で変動するので 2 回以上取ること。

## 構成（前回を壊していない）

| 項目 | 値 | 根拠 |
|---|---|---|
| 手先カメラ | 80x60 / 45 sim Hz / rgb8 | launch 既定（前回と同じ、引数で上書き可） |
| 物理ステップ | 0.01 s | `physics_step` 既定（前回と同じ） |
| gz | `-s -r`（GUI なし） | `arm_vision_slam.launch.py` は常に `gui:=false` |
| RViz / Nav2 / Web UI | 起動しない | 同じ |
| lidar / base カメラ | lite 360点/5 sim Hz / 1 sim Hz | 同じ |
| NPU 入力 | `use_topic_input:=true`, `image_topic:=/arm_camera/image_raw` | 同じ |
| `detection_fps` | 0（ゲート無効） | 30 Hz ゲートは推論 ~25 ms と位相がぶつかり 18.6 Hz に落ちた実績 |
| 新規引数 | `vision_target:=true`（既定 false、maze 専用） | [arm_vision_slam.launch.py](../../ros/src/umeonigiri_autonomy/launch/arm_vision_slam.launch.py) / [simulation.launch.py](../../ros/src/umeonigiri_autonomy/launch/simulation.launch.py) |

設定の置き場所は 1 箇所だけ: SDF の挿入は [simulation.launch.py](../../ros/src/umeonigiri_autonomy/launch/simulation.launch.py)
の `inject_vision_target()` が担い、対象の形とテクスチャ名は
[vision_target.sdf](../../ros/src/umeonigiri_autonomy/worlds/vision_target.sdf) の 1 ファイルにある。
`maze.sdf` 自体は未変更。

## 実測値（2026-11-25 / 20 秒連続、複数回）

| 条件 | 実測 | 判定 | 証拠 |
|---|---|---|---|
| `/arm_camera/image_raw` | **38.901 Hz**（20 秒 / window 631、末尾平均。別回 39.121） | Measured（≥28） | [live_camera_hz.txt](logs/live_camera_hz.txt) |
| `/detections` | **36.944 Hz**（20 秒 / window 674、末尾平均。別回 37.718） | Measured（≥28） | [live_detections_hz.txt](logs/live_detections_hz.txt) |
| 非空の継続 | **667 / 667 メッセージが非空（100%）**、20.014 秒、33.327 Hz、検出総数 1334 | Measured | [live_detections_20s.txt](logs/live_detections_20s.txt) |
| クラス名 / 数値 ID | `id='person'`, `results[0].hypothesis.class_id='0'`（score 0.500 / 0.473）を実メッセージで確認 | Confirmed（生メッセージ抜粋 3 件入り） | [live_detections_20s.txt](logs/live_detections_20s.txt) |
| RTF | **0.863 / 0.849 / 0.869**（20 秒 × 3）。別セッションで 0.853 / 0.835 も計測 | Measured（すべて ≥0.8） | [live_rtf_final.txt](logs/live_rtf_final.txt)、[live_rtf.txt](logs/live_rtf.txt) |
| `slam_toolbox` | `active [3]` | Measured | [live_slam_tf_headless.txt](logs/live_slam_tf_headless.txt) |
| `/scan` | 4.257 Hz | Measured | 同上 |
| `/map` | 1.000 Hz | Measured | 同上 |
| TF `map -> odom` | `tf2_echo` で取得（並進/回転が出力） | Measured | 同上 |
| `/detections` publisher | 1（kill.sh → 再起動後も 1） | Measured | 同上 |
| ヘッドレス | `rviz2` なし、`gz sim -s -r /tmp/meister_maze_step_0.01_vision.sdf` のみ | Measured | 同上、[live_final_launch.log](logs/live_final_launch.log) |
| カメラ映像 | テクスチャ対象が実際に映っている（80x60 の生フレーム） | Measured | [live_attempt4_camera.png](logs/live_attempt4_camera.png) |
| ビルド / test | `./build.sh` 成功、`umeonigiri_vision` **82 tests 0 failures**、新規 pytest 3 件含む全体 103 tests 0 failures | Measured | 下記「ビルド・テスト」 |
| 回帰 | 既定 launch と `mapping_nav.launch.py` の挙動・base カメラ・Web UI に変化なし | Measured | [live_regression_default.txt](logs/live_regression_default.txt)、[live_regression_mapping_nav.txt](logs/live_regression_mapping_nav.txt) |

`id` にクラス名、`results[0].hypothesis.class_id` に数値 ID を入れる契約は
[detection_msg.hpp](../../ros/src/umeonigiri_vision/include/umeonigiri_vision/detection_msg.hpp)
のとおり。Web UI は `id` を表示ラベルとして使うため、どちらか片方だけでは
認識モニターが `unknown` になる。

## 試行と失敗（記録）

| # | 試行 | 結果 | 判定 |
|---|---|---|---|
| 1 | フラグメントに `<?xml ...?>` を含めて挿入 | gz が `XML_ERROR_PARSING_DECLARATION` で起動失敗 | ✗ 不採用（[log](logs/live_attempt1_xml_declaration_error.log)） |
| 2 | テクスチャ平面だけを配置（無照明） | カメラに暗く映り、検出 0 | ✗ 不採用（[log](logs/live_attempt2_target_unlit.log)、[画像](logs/live_attempt2_camera.png)） |
| 3 | `emissive=0.55` で明るくする | 白く飛んでテクスチャが消え、検出 0 | ✗ 不採用（[log](logs/live_attempt3_emissive_washed.log)、[画像](logs/live_attempt3_camera.png)） |
| 4 | 平面前方に点光源 + PBR `albedo_map` | 画像が認識可能になり検出 2 件/フレーム、RTF 0.835–0.869 | **Confirmed（採用）** |
| 5 | 点光源を廃し `emissive_map` で自発光 | 検出は出たが RTF 0.778 / 0.861（1 回が 0.8 未満） | ✗ 不採用（[log](logs/live_attempt5_selflit.log)、[画像](logs/live_exp5_selflit_camera.png)） |

検出されるのは主に bus.jpg 中の person（2 件）。低解像度 80x60 では bus 自体は
検出されなかった。これは計測対象の性質であり、モデルが bus を認識できない
という意味ではない（静止画像 640x640 では bus も検出済み、
[static_detection_detail.txt](logs/static_detection_detail.txt)）。

INT8 量子化で score が 0.5 に張り付く既知の問題は残る（tools/grasp/README.md）。

## ビルド・テスト（worktree 内で実施）

```bash
cd /home/so/Meistar/.worktrees/arm-cam-live-detections
PATH=/usr/bin:/bin:/usr/local/bin ./build.sh            # 6 packages finished, exit 0
cd ros && PATH=/usr/bin:/bin:/usr/local/bin bash -c \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   colcon test --packages-select umeonigiri_vision umeonigiri_autonomy && \
   colcon test-result --verbose'
# => 103 tests, 0 errors, 0 failures, 0 skipped
#    umeonigiri_vision 単独 => 82 tests, 0 failures
```

新規コードへのテストとして
[test_vision_target.py](../../ros/src/umeonigiri_autonomy/test/test_vision_target.py)
（3 件）を追加した: フラグメントが妥当な SDF であること、挿入が既定ワールドを
変更しないこと、未解決プレースホルダを拒否すること。

## 再現手順（計測）

```bash
# 起動は上記「1 コマンド起動」。別シェルで source してから:
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
cd /home/so/Meistar/.worktrees/arm-cam-live-detections
timeout 21 ros2 topic hz /arm_camera/image_raw
timeout 21 ros2 topic hz /detections
python3 docs/perf/capture_detections_live.py --seconds 20 \
    --out docs/perf/logs/live_detections_20s.txt --excerpts 3
python3 docs/perf/capture_arm_image.py --out /tmp/arm.png
for i in 1 2 3; do python3 docs/perf/measure_rtf.py --seconds 20; done
ros2 lifecycle get /slam_toolbox
timeout 11 ros2 topic hz /scan && timeout 11 ros2 topic hz /map
timeout 6 ros2 run tf2_ros tf2_echo map odom
pgrep -a rviz2; pgrep -af 'gz sim'
./kill.sh
```
