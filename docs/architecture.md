# アーキテクチャ — Meister

> 対象リポジトリ: `sosukigara/2026-Meister`
> コードマップ（ファイル単位の地図）は [docs/code-map.md](code-map.md)。罠は [docs/pitfalls.md](pitfalls.md)、コマンドは [docs/commands.md](commands.md)。

## 1. 全体像（半分散）

リアルタイム性が必要な制御は ESP32、認識・判断は PC (ROS 2)。

```
┌──────────────────── PC (Ubuntu / ROS 2 Jazzy) ────────────────────┐
│  umeonigiri_webnav       Web UI (HTTP + WebSocket :8088)           │
│  umeonigiri_autonomy    Nav2 / SLAM / launch                       │
│  umeonigiri_vision         YOLOv8n ONNX 物体検出                      │
│  umeonigiri_serial_bridge  ⇅ UART ⇅ ESP32 ブリッジ（唯一の接点）       │
└──────────────────────────────┬────────────────────────────────────┘
                               │ USB-UART, 115200 bps
                               │ 固定長バイナリ + XOR チェックサム
┌──────────────────────────────┴────────────────────────────────────┐
│  firmware/main_board/  (ESP32-WROOM-32, PlatformIO + Arduino)            │
│  main.cpp  setup/loop と配線のみ                                   │
│  command_dispatch  受信 FSM → 各機構                               │
│  base_chassis / arm / feedback  機構と FB_STATE 送出               │
│  hal/*  LEDC / サーボ / モータ / プロトコル UART / バス codec      │
└───────────────────────────────────────────────────────────────────┘
```

- シミュレーション（Gazebo）と実機は**同じロボットの異なる段階**。シミュレーションは実機開発の前段。
- 実機の機構構成（6 輪・ステアリング前後 4 輪 = STS3225 12V のバスサーボ・駆動 LA8308 1:20・タイヤ径 100 mm・12 V 系）は `ros/config/meister_robot.yaml` の `robot:` 節が唯一の出所。
- `firmware/foc_rover/` は走行用・`firmware/foc_test/` は回転テスト用の別 PlatformIO プロジェクト（MKS ESP32 FOC V2.0）。`firmware/main_board/` とは独立。単軸ウィンチ用 `foc_winch/` は 2026-11-09 に削除（テスト用途は `foc_test/` に集約）。

## 2. レイヤーと依存方向（片方向のみ）

```
main.cpp → command_dispatch → base_chassis / arm → hal/* → Arduino
                              feedback
hal/bus_protocol ← Arduino 非依存（pio test -e native でテストできる）
```

- `hal/` は上位 (`arm/`, `base_chassis/`, `main.cpp`) を知らない（R8）。
- `hal/bus_protocol` だけが Arduino を include しない。これによりホスト側テスト可能。
- `meister_protocol.*` は Arduino 非依存。PC 側の CMake が同じソースを直接コンパイルするので、バイト単位の一致はビルドで保証される。

## 3. モジュール責務

| モジュール | 責務 | 備考 |
|---|---|---|
| `main.cpp` | setup/loop と配線のみ | 50 行程度。ロジックを置かない |
| `command_dispatch` | 受信 FSM（ヘッダ→種別→ペイロード→チェックサム）と機構への振り分け | 足回り・アームをメンバで保持 |
| `base_chassis` | 足回り（モータ 6 + ステアリング） | 静的インスタンスは作らない |
| `arm` | アーム 4 軸 + グリッパー | 同上 |
| `feedback` | FB_STATE の定周期送出 | 送信間隔は `config::kFeedbackIntervalMs` |
| `hal/ledc_pwm` | LEDC ラッパ（arduino-esp32 2.x/3.x 両対応） | API 差はここに閉じる |
| `hal/pwm_servo` | PWM サーボ（50 Hz LEDC） | DS 150kg 等 |
| `hal/motor` | DC モータ（速度→デューティ比+方向） | |
| `hal/bus_protocol` | STS/SCS パケット codec | Arduino 非依存、host test 可能 |
| `hal/bus_servo` | バスサーボ（STS3215 等） | |
| `meister_protocol` | TypeId・軸数・値域・サイズ・Frame 構造 | PC 側とバイト一致 |
| `meister_config` | 設定の唯一の出所 | 生成物 `generated_config.h` を include |
| `umeonigiri_serial_bridge/include/.../serial_io.hpp` | UART I/O 抽象と libserial 実装 | gtest からフェイクを注入できる |
| `umeonigiri_serial_bridge/include/.../bridge_core.hpp` | ROS 非依存の中核（ウォッチドッグ・再オープン・受信） | `Clock` を注入する |
| `umeonigiri_serial_bridge/src/frames.cpp` / `kinematics.cpp` | フレーム組み立てと Twist → (舵角, 速度) | 値域は firmware の生成 config を 1 箇所で使う |
| `umeonigiri_serial_bridge/src/odometry_publisher.cpp` | `/odom` と `odom -> base_footprint` TF | rclcpp 依存。コアは ROS 非依存のまま |
| `umeonigiri_serial_bridge/src/feedback_publisher.cpp` | `/esp32/state` と `/imu/data` | rclcpp 依存。コアは ROS 非依存のまま |

## 4. データフロー

### PC → ESP32（指令）
1. Nav2 が `/cmd_vel` (Twist) を配信
2. `serial_bridge_node` が購読 → `kinematics.cpp` で (舵角, 速度) に変換
3. `firmware` の `meister_protocol.cpp` を直接リンクした `frames.cpp` でフレーム encode → UART 送信（到着時、10〜20 Hz）
4. ESP32 `command_dispatch` が受信 FSM で decode → `base_chassis` / `arm` へ振り分け
5. ESP32 は `MSTE_CONTROL_HZ` (100 Hz) で最新指令を再適用

### ESP32 → PC（フィードバック）
1. ESP32 が `FB_STATE` を `MSTE_FEEDBACK_HZ` (100 Hz) で送信
2. `serial_bridge_node` が受信 → `/esp32/state` 配信

### STS バス（半二重）
- SYNC_WRITE で 1 パケットで複数台に届く（≈0.33 ms @ 9 台）
- 状態読み戻しは turnaround が直列に積もるため、ラウンドロビンで 2〜3 台/回（`MSTE_BUS_READ_CHUNK`）

## 5. 通信周期（2026-09-27 合意）

| 対象 | 周期 | 実装 | 根拠 |
|---|---|---|---|
| PC → ESP32 指令 | 到着時（10〜20 Hz） | `_on_cmd_vel` で即送信 | 同じ値の再送は情報量が増えない |
| ESP32 指令適用 | 100 Hz (10 ms) | `MSTE_CONTROL_HZ` | 制御周期。線路使用率 41%→15% |
| ESP32 → PC FB_STATE | 100 Hz | `MSTE_FEEDBACK_HZ`（実測決定） | `firmware/main_board/README.md`「通信周期の測定」 |
| STS 指令（SYNC_WRITE） | 100 Hz 可 | 1 パケットで 9 台 | 帯域に余裕 |
| STS 状態読み戻し | 全体刷新 30〜40 ms | `MSTE_BUS_READ_CHUNK` ラウンドロビン | turnaround が直列に積もる |

## 6. 設計パターン / 規約

- **HAL 分離**: ハードウェア依存は `hal/` に閉じ込め、上位は抽象 (`IServoChannel`) を使う。
- **設定の単一出所**: `meister_config.h`（コンパイル時 `-D` → フォールバックの 1 経路）。
- **生成物**: `ros/config/meister_robot.yaml` → `tools/gen_config.py` → `hal/generated_config.h`。機構は生成定数表を読む。
- **プロトコルの双方向一致**: PC と firmware は同じ `meister_protocol.*` をコンパイルする。軸数変更時は firmware の config と両テストを直す。
- **静的グローバル禁止**: `setup()` 内で 1 か所に aggregate 構築（R11）。翻訳単位をまたぐ静的初期化順は未規定。
- **未確定値のゲート**: `MSTE_ENABLE_BUS_SERVO=0` の間はバスを駆動しない。未取得を表す値は到達不能値（`0xFF`）を使い、`kRegisterMapVerified=false` の間は WRITE をゲート（R4）。
- **幾何未確定のフェイルクローズ**: `kinematics.cpp` の `kGeometryNotFilled` のように、幾何が埋まるまで `Solve()` は必ず失敗し指令を出さない（R4 延長）。

## 7. アクチュエータ構成（BOM 画像より・計画値、確定ではない）

| 部位 | 種別 | 個数 | 制御方式 |
|---|---|---|---|
| 車輪駆動 | DC ギヤード + マグネットエンコーダ | 6 | PWM デューティ比 |
| ロッカー軸・ボギー軸 | DS サーボ 150kg | 4 | PWM（ユーザー確認済み） |
| ステアリング | STS3215 12V | 4 | バス |
| アーム肩 | DS サーボ 150kg (`rds51150`) | 1 | PWM（ユーザー確認済み） |
| アーム肩（方位角）/ 先端（肘・手首）/ 開閉①② | STS3215 12V | 1 + 2 + 2 | バス |

> 現プロトコルは `steering 6ch` / `arm 4ch` / `gripper 1ch` で BOM と不一致。軸数は `meister_config.h` の定数から導出する方針（R7）。

## 8. 未確認値（推測で埋めていない）

| 項目 | 現在の値 | 状態 |
|---|---|---|
| `MSTE_BUS_RESP_TIMEOUT_US` | 3000 µs | データシート未取得 |
| `MSTE_BUS_INTER_FRAME_GAP_US` | 200 µs | データシート未取得 |
| `reg::eprom::*` / `reg::sram::*` アドレス | 各表の値 | 未照合 |
| `reg::kStepsPerRev` | 4096 | 未照合（360° 型かどうか） |

一次情報（Feetech 公式 Python SDK 1.0.0）で確定できたのは命令コード・フレーム構造・チェックサム `~(sum(tx[2..L-2])) & 0xFF`・エラービット・ID 範囲のみ。STS3215 のデータシートで照合すること。

## 9. 設定変更時のチェックリスト

- `platformio.ini` の `MSTE_*` を変える → `meister_config.h` のフォールバックも変える（`ros/src/` はマクロを直接参照しない）
- プロトコルの軸数を変える → firmware の `generated_config.h` と、firmware / PC 両方のテストの期待値を変える
- ピン番号を変える → `pio run -e esp32dev -e esp32dev_usbuart`（両 env）と実機 `comm_check`
- PC の C++ 変更 → `colcon test --packages-select umeonigiri_serial_bridge`。firmware の変更 → `pio test -e native`
- 実機挙動を変更 → `comm_check` 3 手順（uplink / downlink / err_flag）

詳細コマンドは [docs/commands.md](commands.md)。罠は [docs/pitfalls.md](pitfalls.md)。
