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

## 6. 4 通りのビルド（条件コンパイル確認）

1. `pio run -e esp32dev`
2. `pio run -e esp32dev_usbuart`
3. `PLATFORMIO_BUILD_FLAGS="-DMSTE_SERVO_DRIVER_DEBUG=1" pio run -e esp32dev`
4. `pio test -e native`

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
