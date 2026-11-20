# 実行・テスト・検証コマンド — Meister

> pio 系は `firmware/main_board/` を cwd にする。ros2 系は `ros/` を cwd にする（R6）。詳細な罠は [docs/pitfalls.md](pitfalls.md)。

## 1. 前提

- PlatformIO CLI（`pio`）が使えること。
- ROS 2 Jazzy が source 済みであること。
- 実機接続時は `upload_port` / `monitor_port` が `platformio.ini` で by-id 指定されている（別ボード誤書き込み防止）。

## 2. ビルド

```bash
# 全体ビルド（4 パッケージ並列）
./build.sh

# ファームウェア（両 env）
cd firmware/main_board
pio run -e esp32dev -e esp32dev_usbuart

# 条件コンパイルの分岐を静かに壊さないため、デバッグフラグでもビルド
PLATFORMIO_BUILD_FLAGS="-DMSTE_SERVO_DRIVER_DEBUG=1" pio run -e esp32dev

# 書き込み
cd firmware/main_board
pio run -e esp32dev -t upload
```

## 3. テスト

```bash
# ファームウェア プロトコル / バス codec（Unity、38 cases）
cd firmware/main_board
pio test -e native

# Python ブリッジ（22 cases）
cd ros/src/meister_serial_bridge
PYTHONPATH=. python3 -m pytest test/

# リポジトリルートから
python3 -m pytest ros/src/meister_serial_bridge/test/
```

## 4. 実機検証（comm_check）

実機 `comm_check` が 3 手順 PASS するまで commit しない。

```bash
# リポジトリルート
ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0
# 手順: uplink / downlink / err_flag

# FB_STATE 周期の実測
ros2 run meister_serial_bridge meister_hz_measure --port /dev/ttyUSB0 --expect-hz 100
```

## 5. 起動

```bash
# 実機モード (既定。Gazeboなし。LD-D500 + ESP32 が必要)
./launch.sh
# シミュレーションモード (Gazeboあり)
./launch.sh --sim [warehouse|maze|empty]
# 停止
./kill.sh
```

- 実機モードは `ros2_autonomous_nav/real_robot.launch.py` を起動する
  (robot_state_publisher + LD-D500 + SLAM + Nav2 + serial_bridge + Web UI + 画像認識)。
- ポートは環境変数で上書き可 (既定: ESP32 `/dev/ttyUSB0`、LD-D500 `/dev/ttyUSB1`。
  両方 `ttyUSB0` のままだと衝突する)。by-id / by-path 推奨。
  ```bash
  MEISTAR_SERIAL_PORT=/dev/ttyUSB0 MEISTAR_LIDAR_PORT=/dev/ttyUSB1 ./launch.sh
  ```
- シリアル権限がないと `/scan` も `/esp32/state` も流れない。
  `groups` に `dialout` がなければ `sudo usermod -a -G dialout $USER` 後に再ログイン。
- `launch.sh` は VPN 環境のマルチキャスト障害回避で `ROS_LOCALHOST_ONLY=1` を設定するが、**子プロセスのみ**。別ターミナルの `ros2` CLI には継承されない。
- 表示系は切れる。既定はどちらも有効（従来どおり）。
  ```bash
  # Gazebo の GUI と RViz を切る。実測で 1 コアの 66% と 34% を節約でき、
  # 同じ時間でシミュレーションが約 2 倍進む。地図と姿勢は Web UI で見る。
  MEISTER_GUI=0 MEISTER_RVIZ=0 ./launch.sh --sim maze
  ```
  ヘッドレスでもセンサ(`/scan`・`/camera/image_raw`)はサーバ側で動く。
- **シミュレーションは既定でソフトウェア描画（llvmpipe）を使う。**
  この機体の Mali/Panfrost は GL 3.3 しか出せず、Ogre2 のシェーダがリンクできずに
  Gazebo の lidar が壊れる（全点 0.1 m）。llvmpipe は GL 4.5 を出せる。
  実 GPU のある機体では `MEISTER_GL=hardware` で切る。詳細は
  [pitfalls P13](pitfalls.md)。
- センサの重さは `MEISTER_SIM_PROFILE` で選ぶ。シミュレーションの既定は `lite`。
  ```bash
  MEISTER_SIM_PROFILE=full ./launch.sh --sim maze   # 640点/10Hz・640x480/10Hz
  ```
  `lite` は 360 点/5 Hz・320x240/5 Hz。ソフトウェア描画では `lite` でないと
  実時間に届かない（maze で実測 RTF 0.89）。`warehouse` は外部メッシュが重く
  0.2 程度なので、**アルゴリズム検証は `maze` を使う**。

## 5.5 ナビゲーションの計測

```bash
# 30 run のベースライン計測（起動〜計測〜片付けをまとめて行う）
ros/src/ros2_autonomous_nav/scripts/measure.sh docs/bench/<date>-<name>.json 10 3 <map-id>

# 制御器を差し替えて比較（既定は RPP）
MEISTER_LAUNCH_ARGS="controller:=mppi" \
  ros/src/ros2_autonomous_nav/scripts/measure.sh docs/bench/<date>-mppi.json 10 1 mppi
```

- ラッパーが `DISPLAY`・`ROS_DOMAIN_ID` を設定し、Nav2 と `map→base_footprint` の
  両方が立ち上がるまで待ってから計測する（spawn 失敗時は起動し直す）。
- **launch と計測は同じプロセス内で行うこと。** 別プロセスからは DDS が
  見えないことがある（VPN 環境のマルチキャスト障害）。
- 結果の読み方と過去の計測は [docs/bench/](../bench/) を参照。

## 5.6 CPU 内訳の計測

```bash
# 起動したまま、区間 20 秒でプロセス別の CPU を積分する
python3 ros/src/ros2_autonomous_nav/scripts/cpu_split.py --seconds 20 --label "headless"
```

`/proc/<pid>/stat` の utime+stime の差分を使う。`top` の瞬間値は数秒で振れるため、
比較には区間積分を使う。**シミュレーションの値は `/clock` の配信量に比例して
膨らむ**ので、実機の見積りに流用しないこと（[bench/2026-11-20-cpu-analysis.md](../bench/2026-11-20-cpu-analysis.md) §1）。

## 6. 4 通りのビルド（条件コンパイル確認）

1. `pio run -e esp32dev`
2. `pio run -e esp32dev_usbuart`
3. `PLATFORMIO_BUILD_FLAGS="-DMSTE_SERVO_DRIVER_DEBUG=1" pio run -e esp32dev`
4. `pio test -e native`

## 6.5 worktree での作業と着地（R13）

```bash
# メインチェックアウト (/home/so/Meistar) で: 開始
git fetch origin
git worktree add -b <type>/<topic> .worktrees/<topic> origin/main

# worktree 内で: 編集・ビルド・テスト・commit
cd .worktrees/<topic>
git diff main...HEAD          # レビュー（R6）

# メインで: merge → push → 後片付け
cd /home/so/Meistar
git merge --no-ff <type>/<topic>
git push origin main
git worktree remove .worktrees/<topic> && git branch -d <type>/<topic>
```

`<type>` は `feat` / `fix` / `docs` / `chore`。feature ブランチ自体は push しない。

## 7. 機械チェック

```bash
# コンパイル / 波括弧の平衡 / 名前空間の開閉 / 識別子の多重定義 / 文字化け
bash .omo/session-work/check_header.sh
```

## 8. 設定変更時のチェックリスト

- `platformio.ini` の `MSTE_*` を変える → `meister_config.h` のフォールバックも変える
- プロトコルの軸数を変える → `meister_protocol.h` **と** `protocol.py` **と** 両方のテストの期望値を変える
- ピン番号を変える → `pio run -e esp32dev -e esp32dev_usbuart`（両 env）と実機 `comm_check`
- Python のみの変更 → `pytest`。C++ 変更 → `pio test -e native`
- 実機挙動を変更 → `comm_check` 3 手順

## 9. 画像認識の A/B（同一入力でモデルを比べる）

同じ録画を 2 つの設定に流して比べる。シーンが暗いと検出が出ず比較にならないので、
**明るく物体が写る状態で録る**こと。

```bash
# 1) 入力を録る（実機カメラ）
ros2 bag record -o /tmp/bag_cam /camera/image_raw   # 25 秒ほど

# 2) 各設定で /detections を採取（bag 再生 + 購読の記録）
ros/install/meister_vision/lib/meister_vision/detection_node --ros-args \
  -p image_topic:=/camera/image_raw -p rate:=30.0 -p inference_threads:=4 \
  -p model_path:=ros/install/meister_vision/share/meister_vision/models/yolov8n.onnx &
python3 ros/src/meister_vision/scripts/record_detections.py \\
  --seconds 26 --out /tmp/ab_640.json
ros2 bag play /tmp/bag_cam --rate 1.0

# 3) 320 でも同じことをして、フレームごとの検出数・クラス一致を比べる
```

前回の結果と「比較にならなかった理由」は
[bench/2026-11-20-cpu-analysis.md](../bench/2026-11-20-cpu-analysis.md) §5。

## 10. 画像認識の内訳を測る

`MEISTER_VISION_PROFILE=1` を付けると、1 フレームの前処理・推論・後処理の
平均が 20 フレームごとに stderr に出る。**入力は計算より速く流す**こと
（カメラは 1〜8 Hz で頭打ちになるため、bag 再生で 3〜5 倍速にする）。

```bash
MEISTER_VISION_PROFILE=1 \
ros/install/meister_vision/lib/meister_vision/detection_node --ros-args \
  -p image_topic:=/camera/image_raw -p rate:=30.0 -p inference_threads:=2 \
  -p model_path:=ros/install/meister_vision/share/meister_vision/models/yolov8n-320.onnx &

# 購読者がいないと推論しないので、記録器か rqt を同時に動かす
python3 ros/src/meister_vision/scripts/record_detections.py --seconds 30 --out /tmp/p.json &
ros2 bag play /tmp/bag_cam --rate 5.0
```

出力例（320 入力・A76 固定・2 スレッド）:
`[vision-profile] pre 0.9 ms  infer 57.0 ms  post 0.4 ms  (input 320)`

現状は**推論が 98%** なので、前処理・後処理の最適化では速くならない。

## 11. 画像認識を RViz で見る

```bash
DISPLAY=:0 ros/src/meister_vision/scripts/vision_view.sh
# DEVICE=/dev/video0 RATE=30 THREADS=2 MODEL=/path/to/model.onnx ... vision_view.sh
```

- 左のパネルに**検出枠つき**の `/detection_image`、タブに生の `/camera/image_raw`。
- スクリプトはカメラ・検出・`robot_state_publisher`（`camera_link` の TF）・RViz を
  まとめて起動する。TF が無いと `Global Status: Warn` になり 3D ビューが空になる。
- 検出ノードは購読者が 0 の間は推論しないので、RViz が立ち上がるまで
  `/detections` は流れない（異常ではない）。
- レート確認: `ros2 topic hz /detections`、`ros2 topic hz /detection_image`。

## 12. 画像認識のバックエンド切り替え

既定は NPU（RKNN）。CPU 版（ONNX Runtime）へ戻すときは `backend:=onnx`。

```bash
ros2 launch meister_vision pc_vision.launch.py                # NPU（既定）
ros2 launch meister_vision pc_vision.launch.py backend:=onnx  # CPU
```

- どちらも `./detections` / `./detection_image` / `./target` を出すので、
  Web UI とアーム制御は変更不要。
- NPU 版は自前でカメラを開く。**`use_pc_camera:=true` と併用しないこと**（排他）。
- RKNN モデルは `ros/src/meister_vision/models/yolov8n-zoo.rknn`。
  生成手順は [tools/rknn/README.md](../tools/rknn/README.md)。
- 実機モードの `launch.sh` は、モデルがあれば自動で NPU を選ぶ。
