# 自動走行の安定化 ＆ 30fps 高精度・安定検出の同時成立（Round 2）

## 目的と現状判定（2026-11-26 / Round 2）

前段の「手先カメラ NPU 検出 37.6Hz（非空 person）＋ヘッドレス SLAM」は
[arm-cam-live-detections.md](arm-cam-live-detections.md) のとおり達成済み。
ここでは **自動走行しながら 30fps 近傍の検出が途切れず、クラス／枠が安定し、
ゴールが 6/6 成功すること** を目標にする。

**Round 2 判定: 主要 7 条件は Measured / 1 条件は未達（下記）。完了とは言わない。**

| 条件 | 判定 | 根拠 |
|---|---|---|
| 1 コマンド headless 起動 | **Measured**（回帰なし） | [stable_drive_launch_r19.log](logs/stable_drive_launch_r19.log)、[regression_mapping_nav_r19.txt](logs/regression_mapping_nav_r19.txt) |
| 2 自動走行の安定性 | **Measured**（6/6, max 0.180 m, 急変 0 回） | [stability_r19.log](logs/stability_r19.log)、[drive_bag_r19_summary.json](logs/drive_bag_r19_summary.json) |
| 3 走行中の 30fps 連続性 | **Measured**（36.4/38.4 Hz、146.8 s 無空白。ただし起動直後に 1 回 0.344 s） | [drive_bag_r19_summary.json](logs/drive_bag_r19_summary.json) |
| 4 高精度 | 一部 **Measured** / **✗ 未達**（視野内 25.1 s のみで 60 s 条件を満たせない） | 下記「高精度の定義」 |
| 5 Resource / RTF | **Measured**（中央値 0.854、gz 4.55 / vision 0.81 コア） | [cpu_r19.txt](logs/cpu_r19.txt)、[drive_bag_r19_summary.json](logs/drive_bag_r19_summary.json) |
| 6 ビルド・テスト | **Measured**（build exit 0、117 tests 0 failures） | 下記 |
| 7 証拠・ドキュメント | このファイル＋`docs/perf/logs/` | 全数値に生ログを紐付け |

## 1 コマンド起動（既定挙動は無変更）

```bash
cd /home/so/Meistar/.worktrees/stable-drive-accurate-vision
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe   # llvmpipe 必須 (P13)
source /opt/ros/jazzy/setup.bash && source ros/install/setup.bash
ros2 launch umeonigiri_autonomy stable_drive_vision.launch.py world:=maze
```

- Gazebo は `-s -r`（サーバのみ）。`rviz2` は無く、`gz sim` は 1 プロセスのみ。
- SLAM + Nav2（controller / planner / behavior / bt_navigator / velocity_smoother）+
  手先カメラ + NPU 検出 + Web UI（http://127.0.0.1:8088）が同時に動く。
- NPU 設定は [vision_node_common.py](../../ros/src/umeonigiri_autonomy/launch/vision_node_common.py)
  の 1 箇所、`detection_fps=0`。走行用の Nav2 上書きは
  [stable_drive_nav_overlay.yaml](../../ros/src/umeonigiri_autonomy/config/stable_drive_nav_overlay.yaml)
  の 1 箇所だけ。既定の `mapping_nav_params.yaml` は未変更。
- 既存 `mapping_nav.launch.py` は smoke で lifecycle active、`/camera/image_raw` /
  `/scan` / `/cmd_vel` を維持、`/cmd_vel_nav` は出さない（＝既定経路のまま）:
  [regression_mapping_nav_r19.txt](logs/regression_mapping_nav_r19.txt)。
  `arm_vision_slam.launch.py` の契約テストも継続してパス。

## 実測: 自動走行（2026-11-26 / r19、148.5 s）

ROS 側の Python 購読 recorder は取りこぼすため、計測は `ros2 bag record -s sqlite3`
に切り替えた。全メッセージを保存するため、レートも配信空白も後から再現できる。

| 項目 | 実測 | 判定 | 証拠 |
|---|---|---|---|
| 6 ゴール連続 | **6/6 成功** | Measured | [stability_r19.log](logs/stability_r19.log) |
| 最終誤差 | max **0.180 m** / mean 0.149 m（全ゴール ≤0.25 m） | Measured | [stability_r19.json](logs/stability_r19.json) |
| 所要時間 | 9.0〜23.1 s / ゴール（≤180 s） | Measured | 同上 |
| lifecycle | slam / controller / planner / bt_navigator / smoother すべて `active [3]` | Measured | [health_r19.txt](logs/health_r19.txt) |
| プロセス再起動 | `process has died` / `exit code` **0 件** | Measured | 同上 |
| TF | `map -> odom` が有効（tf2_echo の init 警告は CLI 側の時刻問題で変換値は健全） | Measured | [health_r16.txt](logs/health_r16.txt) |
| 走行距離 | odom 経路長 **26.18 m** | Measured | [drive_bag_r19_summary.json](logs/drive_bag_r19_summary.json) |
| `/cmd_vel` | **26.908 Hz**、linear 急変 **0 回**、angular 急変 **0 回**、max Δlin 0.025 / Δang 0.0225 | Measured（≥10 Hz, ≤1 回） | 同上 |
| `/arm_camera/image_raw` | **38.438 Hz** | Measured（≥28） | 同上 |
| `/detections` | **36.413 Hz** | Measured（≥28） | 同上 |
| 配信空白 | >200 ms は **1 回 (0.344 s, t=0.51–0.85 s)**。その後 **146.76 s 連続無空白** | **Measured**（60 s 連続条件は満たす。ただしセッション全体で 0 回ではない） | 同上 |
| RTF 20 s × 7 | **0.777 / 0.822 / 0.857 / 0.854 / 0.868 / 0.854 / 0.876**、中央値 **0.854** | Measured（≥0.8） | 同上 |
| CPU | gz **4.551 コア**、yolo_grasp **0.81 コア**（142 s 平均、llvmpipe） | Measured | [cpu_r19.txt](logs/cpu_r19.txt) |

### `/cmd_vel` 急変が 14→0 になった理由（原因と修正）

r16 では angular 急変 14 回（max 1.065 rad/s）だった。原因は `/cmd_vel` に
**controller_server の生出力と velocity_smoother の平滑出力が両方流れていた**こと。
さらに Jazzy の velocity_smoother は `cmd_vel` を購読し `cmd_vel_smoothed` を
発行するため、設定ファイルの `velocity_topic` / `smoothed_velocity_topic` は
効いていなかった（ros2 param list で未設定を確認）。

修正は [navigation.launch.py](../../ros/src/umeonigiri_autonomy/launch/navigation.launch.py)
の `use_smoother_path` フラグ（既定 false）で、ON のときだけ
`controller: cmd_vel -> cmd_vel_nav`、`smoother: cmd_vel_nav -> cmd_vel` に
remap する。上書き加速度も [stable_drive_nav_overlay.yaml](../../ros/src/umeonigiri_autonomy/config/stable_drive_nav_overlay.yaml)
で angular 3.2 → 0.9 rad/s² に下げた。r16 (14 回) → r17 (15 回、remap 前) →
**r19 (0 回)** が実測の before/after。

## 高精度の定義（Known-object 回帰。COCO mAP ではない）

画像の出所は前段と同じ `vision_target:=true` の bus.jpg テクスチャ平面で、
**実カメラの現場認識ではない**。ここでの「高精度」は
**テクスチャ対象に対する既知クラス・Known-object 回帰**であって COCO mAP ではない。
モデル出力を正解に使う循環参照は避け、ラベル付き静止画像の GT 枠は画像を見て
手作業で付けている ([measure_static_accuracy.py](../../docs/perf/measure_static_accuracy.py))。

| 項目 | 実測 | 判定 | 証拠 |
|---|---|---|---|
| ラベル正解率（走行中） | person **1746** / 非空 1753 = **99.60 %**（他 car 5 / bus 1 / toilet 1） | Measured（≥95） | [drive_bag_r19_summary.json](logs/drive_bag_r19_summary.json) |
| 検出継続率（視野内） | **870 / 870 = 100 %** | Measured（≥95） | 同上 |
| 視野内 time span | **25.12 s**（t=0.04–25.16 s）。走行セッションの残りは対象が視野外 | **✗ 60 s 条件未達** | 同上 |
| 視野外 false positive（同一セッション） | **4 / 4505 = 0.089 %** | Measured（<1） | 同上 |
| 視野外 false positive（対象なし別セッション） | **0 / 755 = 0.00 %**（`vision_target:=false`, 21.7 s） | Measured（<1） | [no_target_summary_r19.json](logs/no_target_summary_r19.json) |
| bbox 中心 stddev | x **3.632 px** / y 0.036 px（80x60 入力、1746 検出） | Measured（≤5） | [drive_bag_r19_summary.json](logs/drive_bag_r19_summary.json) |
| 静止画像 class 正解率 | **4 / 4 = 100 %**（person ×3, bus ×1） | Measured | [static_accuracy.json](logs/static_accuracy.json) |
| 静止画像 IoU（手作業 GT） | mean **0.9177**（個別 0.855 / 0.900 / 0.988 / 0.928） | Measured | 同上 |
| 静止画像 FP | 1 件（person, score 0.3922）— 隠さず記録 | Measured | 同上 |

- INT8 量子化で score が 0.5 に張り付く既知問題は残る（tools/grasp/README.md）。
- 枠揺れは [box_center_filter.hpp](../../ros/src/umeonigiri_vision/include/umeonigiri_vision/box_center_filter.hpp)
  の EMA（α=0.25、12 px 以上でリセット）で抑制した。r16 の std_x 6.998 px →
  r17 5.109 px（α=0.35）→ **r19 3.632 px（α=0.25）**。
- 視野内判定は画像ヒストグラム（bus.jpg との Bhattacharyya 距離 <0.65）。
  しきい値 0.35 は既知ポジティブ画像で 0.546 だったため不適切と判明し較正した
  （壁は 0.834〜0.902）。検出出力は oracle に使っていない。

### なぜ 60 s 視野内が達成できないか（具体的報告）

計測対象は世界に固定した平面で、手先カメラ視野は 60°。6 ゴールで機体が
26 m を移動するため、対象は t≈25 s で視野外へ出る。これを避けるため
Round 2 で「同じ bus.jpg 平面を機体に固定する」試行を 6 回行ったが、
いずれも次の理由で断念した（例: [r12](logs/stable_drive_launch_r12.log) /
[r14](logs/stable_drive_launch_r14.log)）:

- 平面を視野に入れる高さ (base_link 0.28) では **gpu_lidar が visual を前方障害として拾い**、
  `no valid path found` / `detected collision ahead` で 6/6 → 0/6 に回帰。
- 平面を下げると lidar は回避できるが検出が 0 になる。
- lidar を平面より上へ上げると高さ 0.6 m が maze 壁の上端 0.6 m と一致し、
  `/scan` が全 `inf` になって SLAM 地図が 0x0 になった。
- 横へずらすと lidar は回避できるが、平面が画像右端で欠けて検出 0 になった。

このため **視野内 60 s の bbox 安定性は現状の fixture では ✗ Unverified**。
25 s の実測では std_x 3.632 px であり、条件の数値自体は満たしている。
「対象が視野内にある frames の検出継続率」は 25 s / 870 frames で 100 %。

## 試行と失敗（Round 2）

| # | 試行 | 結果 | 判定 / 次_actions |
|---|---|---|---|
| 1 | rosbag2 で生記録に切替 | 前段の 0.246–0.428 s 空白は recorder 側の取りこぼしと判明。実 bag でも 0.344 s が 1 回だけ残る | Measured（空白ゼロは未達、60 s 連続は達成） |
| 2 | 機体固定テクスチャ平面（6 通りの high/lateral/pitch） | 検出できても lidar が障害物化、または lidar 回避なら検出 0 | ✗ 不採用（コードは revert 済み） |
| 3 | bbox EMA α=0.35 | std_x 6.998 → 5.109 px（Still >5） | ✗ 不十分 |
| 4 | bbox EMA α=0.25 | std_x **3.632 px** | **Confirmed（採用）** |
| 5 | velocity_smoother の加速度だけ下げる | 急変 15 回のまま（生出力が別トピックから /cmd_vel へ出ていた） | ✗ 原因違い |
| 6 | controller → smoother → /cmd_vel に remap | 急変 **0 回**、Hz 12→26.9 | **Confirmed（採用）** |
| 7 | 可視判定しきい値 0.35 | 既知ポジティブが 0.546 で false 判定 | ✗ 0.65 に較正 |

## ビルド・テスト（worktree 内で実施）

```bash
cd /home/so/Meistar/.worktrees/stable-drive-accurate-vision
PATH=/usr/bin:/bin:/usr/local/bin ./build.sh                 # 6 packages finished, exit 0
cd ros && bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  colcon test --packages-select umeonigiri_vision umeonigiri_autonomy && \
  colcon test-result --verbose'
# => 117 tests, 0 errors, 0 failures, 0 skipped
```

新規コードへのテスト: [test_stable_drive_launch.py](../../ros/src/umeonigiri_autonomy/test/test_stable_drive_launch.py)
（回帰・CLI）、[test_static_accuracy.py](../../ros/src/umeonigiri_autonomy/test/test_static_accuracy.py)
（yaw 対応・上書きの分離・GT 独立性）、
[test_box_center_filter.cpp](../../ros/src/umeonigiri_vision/test/test_box_center_filter.cpp)
（EMA の揺れ低減・別クラス追跡・リセット）を追加。

## 後始末

`ROS_DOMAIN_ID=5 ./kill.sh` で全 ROS / gz プロセスを停止（r19 セッション後に実施済み）。
push はしない。main への merge は全条件達成後に行う（R13）。

## 次のラウンドでやること

1. 視野内 60 s を満たす fixture を再検討する。候補: 平面を robot に固定する際に
   `gpu_lidar` の可視レイヤを分ける方法、または静止した対象を複数配置して
   6 ゴールの各前期間で視野内時間を足す方法。数値を緩めずに 60 s を狙う。
2. 起動直後の 0.344 s 空白（t=0.51–0.85 s）の原因を詰める。走行開始前なので
   NPU ノードの初期化／DDS discovery が疑わしい。publish 開始を lifecycle 同期にする。
3. RTF の最初の窓 0.777 が依然ばらつく。gz の CPU 4.55 コアを下げるため
   base カメラ 1 Hz を 0 にする選択肢を、Web UI 契約を壊さない範囲で試す。
4. 最終条件達成後に `git diff main...HEAD` レビュー → main  checkout へ `git merge --no-ff`。
