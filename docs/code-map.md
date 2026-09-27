# コードマップ

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-09-27 | 更新日: 2026-09-27
> このファイルは「このリポジトリのどのファイルが何をして、どのファイルに依存しているか」を 1 画面にまとめたもの。**設計判断は各 design ドキュメント、要件は features に書く**。ここは「地図」だけを置く。

---

## 1. 全体像

半分散型アーキテクチャ。**实时性が必要な制御は ESP32、認識・判断は PC (ROS 2)。**

```
┌──────────────────────── PC (Ubuntu / ROS 2 Jazzy) ────────────────────────┐
│  meister_web_nav   Web UI (HTTP + WebSocket)                                │
│  ros2_autonomous_nav  Nav2 / SLAM / 起動スクリプト                          │
│  meister_vision    YOLOv8n ONNX 物体検出                                    │
│  meister_serial_bridge  ⇅ UART ⇅ ESP32 ブリッジ  ← ★ PC と ESP32 の唯一の接点 │
└────────────────────────────────┬──────────────────────────────────────────┘
                                 │  USB-UART / USB ケーブル
                                 │  115200 bps・固定長バイナリ・XOR チェックサム
┌────────────────────────────────┴──────────────────────────────────────────┐
│  firmware/  (ESP32-WROOM-32, PlatformIO + Arduino)                          │
│  main.cpp        setup/loop・配線（現状: 606 行に全部入り = 要分割）         │
│  meister_protocol.*  プロトコル codec（Arduino 非依存・native で test 可能）  │
│  [未実装] バスサーボ HAL（PWM サーボ / DS サーボ / STS3215 の 3 系統）       │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 2. ROS 2 パッケージ（`src/`）

| パッケージ | 役割 | エントリポイント | 依存 |
|---|---|---|---|
| `meistar_description` | URDF / spawn / 全システム起動 | `launch/all_system.launch.py` `launch/spawn.launch.py` | — |
| `ros2_autonomous_nav` | Nav2 / SLAM / `real_robot.launch.py` | 各 `launch/*.launch.py` | meistar_description, meister_serial_bridge |
| `meister_vision` | YOLOv8n ONNX 検出 → `detections` | `meister_vision/detection_node.py` | — |
| `meister_web_nav` | Web UI（地図・現在地・経路） | `meister_web_nav/web_nav_server.py` | meistar_description |
| `meister_serial_bridge` | **`/cmd_vel` → UART フレーム、`/esp32/state` ← FB_STATE** | `serial_bridge_node.py` / `comm_check.py` / `feedback_hz_measure.py` | — |

### `meister_serial_bridge` 内部（PC 側と ESP32 側の接点。ここ最重要）

| ファイル | 役割 | 備考 |
|---|---|---|
| `protocol.py` | **フレーム codec（C++ 版とバイト単位で同じ）** | `encode_*` / `FrameParser`。Python 側で唯一のプロトコル定義。rclpy 非依存なので単体で import できる |
| `kinematics.py` | `Twist` → (舵角, 速度) の変換 | 純粋関数。`test_kinematics.py` で覆盖 |
| `serial_bridge_node.py` | ROS ノード本体。`/cmd_vel` 購読 → フレーム送信、`/esp32/state` 配信、`cmd_timeout` ウォッチドッグ | 実機排他的に UART を持つ |
| `comm_check.py` | ベンチ確認 CLI（`ros2 run … meister_comm_check`） | uplink / downlink / err_flag の 3 手順。ROS 非依存 |
| `feedback_hz_measure.py` | FB_STATE 周期の実測 CLI | フレーム数・エラーフラグ・欠落を検証。ROS 非依存 |

> **`protocol.py` は `firmware/include/meister_protocol.h` とバイト単位で一致していなければならない。**両者の軸数を変えるときは必ず両方直す（`test_protocol.py` / `test_protocol.cpp` が同じ入力で同じ出力を返すことを保証している）。

---

## 3. firmware（`firmware/`）

### 現状（2026-09-27）

| ファイル | 行数 | 役割 | 問題点 |
|---|---|---|---|
| `src/main.cpp` | 606 | **全部入り**: ピン定義 / LEDC ラッパ / モータ / サーボ / 受信 FSM / ディスパッチ / フィードバック / setup・loop | 9 セクションが 1 個の匿名 namespace に入っている。1 ファイルを変更すると全部にheeler影響 |
| `include/meister_protocol.h` | 197 | TypeId・軸数・値域・サイズ・Frame 構造・関数宣言 | 1 ファイルに全部入り |
| `src/meister_protocol.cpp` | 178 | エンコード / デコード / チェックサム | Arduino 非依存（良い） |
| `test/test_protocol/` | 249 | Unity テスト（`pio test -e native`） | — |

### ピン割当は二重定義になっている（要解消）

| 場所 | 内容 |
|---|---|
| `platformio.ini` の `build_flags` | `MSTE_MOTOR_PIN0..5` `MSTE_STEER_PIN0..5` `MSTE_ARM_PIN0..3` `MSTE_GRIPPER_PIN` `MSTE_UART_RX_PIN/TX_PIN` `MSTE_FEEDBACK_HZ` … |
| `firmware/src/main.cpp:33-135` | 同名マクロの `#ifndef` フォールバック（既定値を 2 箇所に書く） |

**どちらを変更しても片方が残る。** 設定の唯一の出所を作る（→ `include/meister_config.h`）のが重构の第一段階。

### 構造の計画（2026-09-27 合意、以下順で実施）

```
firmware/
├── include/
│   ├── meister_config.h        ★設定の唯一の出所（pin / サーボ ID / 軸数 / 範囲）
│   ├── meister_protocol.h       プロトコル codec（軸数は config から導出）
│   └── hal/
│       ├── ledc_pwm.h           LEDC ラッパ（arduino-esp32 2.x/3.x 両対応）
│       ├── servo.h              IServoChannel 抽象（1 実装 = 1 機構の 1 軸）
│       ├── pwm_servo.h          PWM サーボ（DS 150kg・ステアリング等は 50Hz LEDC）
│       ├── bus_protocol.h       STS/SCS パケット codec（Arduino 非依存・host test 可能）
│       ├── bus_servo.h          バスサーボ（STS3215 等）
│       └── motor.h              DC モータ（速度 → デューティ比 + 方向）
├── src/
│   ├── meister_protocol.cpp
│   ├── hal/{ledc_pwm,pwm_servo,bus_protocol,bus_servo,motor}.cpp
│   ├── base_chassis.{h,cpp}     足回り: モータ 6 + ステアリング 4 + ロッカー 4
│   ├── arm.{h,cpp}              アーム: 肩 1（PWM）+ バス 5
│   ├── command_dispatch.{h,cpp} フレーム → 各機構
│   ├── feedback.{h,cpp}         FB_STATE 送出
│   └── main.cpp                 setup/loop と配線だけ（50 行程度）
└── test/
    ├── test_protocol/           既存（そのまま）
    └── test_bus_protocol/       新規: バス codec（native）
```

**依存の向き**（必ずこの方向のみ）:

```
main.cpp → command_dispatch → base_chassis / arm → hal/* → Arduino
                ↓
        feedback
                          hal/bus_protocol ← Arduino 非依存（native で test）
```

`hal/bus_protocol` は Arduino を include しない。これだけが `pio test -e native` で
ホスト側テストできるために重要。

### アクチュエータ構成（BOM 画像より・**計画値。確定ではない**）

| 部位 | 種別 | 個数 | 制御方式 |
|---|---|---|---|
| 車輪駆動 | DC ギヤード + マグネットエンコーダ | 6 | PWM デューティ比 |
| ロッカー軸ボギー軸 | DS サーボ 150kg | 4 | **PWM**（ユーザー確認済み） |
| ステアリング | STS3215 12V | 4 | バス |
| アーム肩 | DS サーボ 150kg (`rds51150`) | 1 | **PWM**（ユーザー確認済み） |
| アーム肩（方位角）/ 先端（肘・手首）/ 開閉①② | STS3215 12V | 1 + 2 + 2 | バス |

> 現プロトコルは `steering 6ch` / `arm 4ch` / `gripper 1ch` で BOM と不一致。
> 軸数は **`meister_config.h` の定数から導出する**方針にして、実物のigt 確認後に
> 1 箇所の変更で済むようにする（`protocol.h` / `protocol.py` / テストは追従）。

---

## 4. テストの入口

| 対象 | コマンド | 現状 |
|---|---|---|
| firmware プロトコル | `cd firmware && pio test -e native` | 15 cases |
| bridge (Python) | `cd src/meister_serial_bridge && PYTHONPATH=. python3 -m pytest test/` | 22 cases |
| firmware ビルド | `cd firmware && pio run -e esp32dev -e esp32dev_usbuart` | 2 env |
| 全体ビルド | `./build.sh` | 5 packages |
| 通信の動作確認 | `ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0` | 3 手順 |
| FB_STATE 周期の実測 | `ros2 run meister_serial_bridge meister_hz_measure --port /dev/ttyUSB0 --expect-hz 100` | — |

---

## 5. 設定ファイル

| 場所 | 役割 |
|---|---|
| `firmware/platformio.ini` | ビルド env・`MSTE_*` マクロ（→ `meister_config.h` へ集約予定） |
| `src/meistar_description/config/*.yaml` | URDF / Nav2 パラメータ / ブリッジ（ros_gz 用。シリアル設定は**含まない**） |
| `build.sh` / `start_meister.sh` / `kill_ros.sh` | ビルドと起動。`start_meister.sh:44-48` は VPN 環境のマルチキャスト障害回避で `ROS_LOCALHOST_ONLY=1` を設定（**子プロセスのみ**。別ターミナルの `ros2` CLI には継承されない） |
| `docs/{features,functions,design}/NN-*.md` | 4 層ドキュメント（やりたいこと → features → functions → design） |

---

## 6. 設定変更時のチェックリスト

- `firmware/platformio.ini` の `MSTE_*` を変える → `src/main.cpp` のフォールバックも変える（**二重定義**。`meister_config.h` 導入後はこの 2 箇所の分割が不要になる）
- プロトコルの軸数を変える → `firmware/include/meister_protocol.h` **と** `src/meister_serial_bridge/meister_serial_bridge/protocol.py` **と** 両方のテストの期望値を変える
- ピン番号を変える → `pio run -e esp32dev -e esp32dev_usbuart`（両 env 都要）と実機 `comm_check`
- Python のみの変更 → `pytest`。C++ 変更 → `pio test -e native`
- 実機挙動を変更 → `comm_check` 3 手順（uplink / downlink / err_flag）
