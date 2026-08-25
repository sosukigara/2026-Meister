# Meister ESP32 ファームウェア

ROS 2 (Jazzy) ロボット「Meister」の**実機側マイコン層**。
半分散型アーキテクチャ（PC = ナビ・画像認識 / ESP32 = リアルタイム PWM 制御）の
ESP32 側を PlatformIO + Arduino framework で実装する。

- プロトコル仕様: [docs/design/07-esp32-uart.md](../docs/design/07-esp32-uart.md)
- PC 側の ROS 2 シリアルブリッジ: `src/meister_serial_bridge/` として実装済み。モック ESP32 は別 Wave（対象外）

## ディレクトリ構成

```
firmware/
├── platformio.ini              # 環境定義（esp32dev / native）
├── include/
│   └── meister_protocol.h      # バイナリプロトコル定義（フレーム形式・種別・定数）
├── src/
│   ├── meister_protocol.cpp    # エンコード/デコード実装（Arduino 非依存）
│   └── main.cpp                # ESP32 本体（UART パース / LEDC PWM / サーボ HAL / フィードバック送信）
└── test/
    └── test_protocol/
        └── test_protocol.cpp   # プロトコル層のホスト側ユニットテスト（Unity）
```

## プロトコル仕様（バイトレイアウト）

固定長バイナリフレーム + XOR チェックサム（全フレーム共通、リトルエンディアン）。

```
[0]      ヘッダ      0xA5（同期バイト）
[1]      種別        TypeId（1 バイト）
[2..L-2] ペイロード  種別ごとに固定長
[L-1]    チェックサム  ペイロード末尾までの全バイトの XOR
```

- フレーム長 `L = 2 + PayloadSize(種別) + 1`（種別から一意に決まる）
- チェックサム検証: `ComputeChecksum(フレーム, L) == 0`（チェックサム含めて XOR すると 0）

### PC → ESP32 コマンド（下り）

| 種別 | ID | ペイロード | フレーム長 |
|---|---|---|---|
| CMD_MOTOR_VELOCITY | 0x01 | `int16 vel[6]` 各輪速度 **-1000..+1000**（千分率） | 15 |
| CMD_STEERING_ANGLE | 0x02 | `int16 ang[6]` 舵角 **-900..+900**（0.1° 単位） | 15 |
| CMD_ARM_ANGLE | 0x03 | `int16 ang[4]` 関節角 **0..1800**（0.1° 単位） | 11 |
| CMD_GRIPPER | 0x04 | `uint8 cmd` **0=閉 / 1=開 / 2=停止** | 4 |

### ESP32 → PC フィードバック（上り）

| 種別 | ID | ペイロード | フレーム長 |
|---|---|---|---|
| FB_STATE | 0x81 | `int16 enc[6]` + `uint8 state` + `uint8 error_flags` | 17 |
| FB_ERROR | 0x82 | `uint8 error_code` | 4 |

詳細は `include/meister_protocol.h` 冒頭のコメントを参照。

## ビルド

```bash
cd firmware
pio run -e esp32dev
# 生成物: .pio/build/esp32dev/firmware.bin
```

`firmware.bin` を ESP32 へ書き込む:

```bash
pio run -e esp32dev -t upload
pio device monitor   # 115200 bps
```

### 設定の変更（platformio.ini の build_flags）

| マクロ | 既定値 | 説明 |
|---|---|---|
| `MSTE_UART_BAUD` | 115200 | PC との通信ボーレート |
| `MSTE_UART_RX_PIN` / `MSTE_UART_TX_PIN` | 16 / 17 | プロトコル用 UART（Serial2）ピン |
| `MSTE_MOTOR_PIN0..5` | 25,26,27,14,13,2 | 駆動モータ PWM ピン |
| `MSTE_MOTOR_DIR0..5` | -1 | モータ方向ピン（-1 = 未接続 → PWM のみ） |
| `MSTE_STEER_PIN0..5` | 4,5,15,18,19,21 | ステアリングサーボ PWM ピン |
| `MSTE_ARM_PIN0..3` | 22,23,32,33 | アームサーボ PWM ピン |
| `MSTE_GRIPPER_PIN` | 12 | グリッパーサーボ PWM ピン |
| `MSTE_SERVO_DRIVER_DEBUG` | （未定義） | 定義するとサーボをコンソール出力へ切替（ハード未接続時） |

> 既定のピン配置は**未配線のプレースホルダ**。実機配線が決まり次第ここで変更する。

## テスト

プロトコル層（`meister_protocol.*`）は Arduino 非依存のため、ホスト側でテストできる。

```bash
cd firmware
pio test -e native
```

`pio test -e native` は ESP32 不要で、エンコード/デコードのラウンドトリップ・
チェックサム検証・不正フレーム（ヘッダ/チェックサム/未知種別/途中切れ）の拒否を検証する。

## サーボ制御の設計

- サーボは `IServoChannel` 抽象（`begin` / `setAngleTenths` / `neutral`）で分離。
  - `PwmServoChannel`: LEDC PWM（50Hz、1000–2000µs）による実サーボ駆動（既定）
  - `ConsoleServoChannel`: デバッグコンソール出力（`MSTE_SERVO_DRIVER_DEBUG` で切替）
- モータは速度指令（±1000）→ 符号で方向 / 絶対値でデューティ比にマッピング。
- サーボライブラリは採用せず自前の LEDC 駆動で実装（依存を増やさない方針）。
  必要になれば `lib_deps` に ServoESP32 / ESP32Servo を追加して差し替え可能。

## 既知の制限（次 Wave 以降）

- PC 側 ROS 2 シリアルブリッジ / モック ESP32 は未実装
- エンコーダ値・エラー状態はプレースホルダ（実センサ未接続）
- ピン配置・サーボ/モータ仕様は設計ドキュメント確定後に要調整
