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

## プロトコル UART の切り替え

プロトコル通信に使う UART はビルド時に選ぶ。

| 環境 | UART | 用途 |
|---|---|---|
| `esp32dev` | `Serial2` = GPIO16/17（既定） | ロボット本体。PC とは外部 USB-UART で配線する |
| `esp32dev_usbuart` | `Serial` = UART0（USB） | ベンチ確認。**配線不要**、PC へ USB ケーブル 1 本だけで繋がる |

`esp32dev_usbuart` は UART0 をコンソールと共有するため、コンソール出力（`MSTE_LOG`）
をビルド時に無効化している。UART にはバイナリしか流れないため、`Serial` からデバッグ
出力を読むことはできない。

> `upload_port` / `monitor_port` は ESP32 の `/dev/serial/by-id` パスに固定してある。
> ポート未指定の `pio run -t upload` で同時接続中の別ボード（ユーザー申告 M5Stack、
> 実測 ESP32-P4 = `/dev/ttyACM0`）へ誤って書き込むことを防ぐため。別の ESP32 本体に書き込む場合は
> `platformio.ini` の `[common]` の該当行を変更する。

## ビルド

```bash
cd firmware
pio run -e esp32dev -e esp32dev_usbuart
```

ESP32 へ書き込む:

```bash
pio run -e esp32dev_usbuart -t upload   # ベンチ確認用（USB 経由）
pio run -e esp32dev -t upload           # 実機用（GPIO16/17 経由）
```

### 設定の変更（platformio.ini の build_flags）

| マクロ | 既定値 | 説明 |
|---|---|---|
| `MSTE_UART_BAUD` | 115200 | PC との通信ボーレート |
| `MSTE_UART_RX_PIN` / `MSTE_UART_TX_PIN` | 16 / 17 | プロトコル用 UART（Serial2）ピン |
| `MSTE_UART_BACKEND_USB0` | 0 | 1 でプロトコル UART を USB/UART0 に切り替える（`esp32dev_usbuart` が設定） |
| `MSTE_FEEDBACK_HZ` | 100 | FB_STATE の送信周期 [Hz]。**1000 の約数のみ**（`static_assert`）。下記「通信周期の測定」参照 |
| `MSTE_MOTOR_PIN0..5` | 25,26,27,14,13,2 | 駆動モータ PWM ピン |
| `MSTE_MOTOR_DIR0..5` | -1 | モータ方向ピン（-1 = 未接続 → PWM のみ） |
| `MSTE_STEER_PIN0..5` | 4,5,15,18,19,21 | ステアリングサーボ PWM ピン |
| `MSTE_ARM_PIN0..3` | 22,23,32,33 | アームサーボ PWM ピン |
| `MSTE_GRIPPER_PIN` | 12 | グリッパーサーボ PWM ピン |
| `MSTE_SERVO_DRIVER_DEBUG` | （未定義） | 定義するとサーボをコンソール出力へ切替（ハード未接続時） |

> 既定のピン配置は**未配線のプレースホルダ**。実機配線が決まり次第ここで変更する。

## 通信確認（ベンチ）

配線なしで PC ⇄ ESP32 の双方向通信を検証する。モータ・サーボは動かさない
（送る指令は全 0 とグリッパー停止のみ）。

```bash
cd firmware && pio run -e esp32dev_usbuart -t upload
cd ../src/meister_serial_bridge
python3 -m meister_serial_bridge.comm_check --port /dev/ttyUSB0
```

```
port: /dev/ttyUSB0  (/dev/serial/by-id/usb-Silicon_Labs_CP2102N_...-port0)  baud: 115200
  [OK] uplink    enc(placeholder)=[-988, -851, -714, -577, -440, -303] state=0x00 error=0x00
  [OK] downlink  CMD_MOTOR_VELOCITY, CMD_STEERING_ANGLE, CMD_ARM_ANGLE, CMD_GRIPPER -> ... error=0x00
  [OK] err_flag  corrupt frame -> error_flags |= 0x08
PASS: PC <-> ESP32 通信を確認しました。
```

- `uplink` — FB_STATE をチェックサム検証付きで受け取れるか（ESP32 → PC）
- `downlink` — 4 種の有効コマンドを送ったあとの FB_STATE にプロトコルエラーフラグが
  立たないこと。フラグが立つ = フレーム長が ESP32 側と一致していない（PC → ESP32）
- `err_flag` — 意図的に壊したフレームでフラグが立つこと。RX 経路が死んでいれば
  フラグが立たないため失敗する。`downlink` の判定基準が ESP32 側で実際に機能している
  ことの裏取り sekaligus、下行が因果的に動いている証明になる

`enc(placeholder)` は実エンコーダではなく firmware の時間由来ダミー値
（`sendFeedbackIfDue`）である。エンコーダの検証には使えない。

`comm_check` は既定で計測開始前に ESP32 を再起動する（`--no-reset` で無効化）。
受信エラーは再起動でしか消えないため、手順 2 の前提になっている。

ROS 2 経由の動作は `ros2 launch meister_serial_bridge serial_bridge.launch.py` で
`/esp32/state` を購読して確認する。

## 通信周期の測定

`MSTE_FEEDBACK_HZ` の根拠となった実測値。ESP32-D0WD-V3（MAC `14:2b:2f:ee:ba:c4`）/
 CP2102N / 115200bps に `esp32dev_usbuart` を書き込み、5 秒間の FB_STATE 到着レートを
 計測した（同時に 10 Hz で零指令を流し、双方向が健全な条件）。

再現手順（`platformio.ini` の `MSTE_FEEDBACK_HZ` を書き換えてビルド・書き込み、測り直す）:

```bash
cd firmware
pio run -e esp32dev_usbuart -t upload
cd ../src/meister_serial_bridge
python3 -m meister_serial_bridge.feedback_hz_measure \
    --port /dev/ttyUSB0 --duration 5 --with-commands --expect-hz 100
```

| 指定 Hz | 実測 Hz | 平均間隔 [ms] | p95 [ms] | 最大 [ms] | プロトコルエラー | 欠落 |
|---|---|---|---|---|---|---|
| 50 | 50.0 | 20.00 | 20.50 | 20.71 | 0 | 0 |
| **100** | **100.0** | **10.00** | **10.68** | **11.27** | **0** | **0** |
| 125 | 125.0 | 8.00 | 8.61 | 9.37 | 0 | 0 |
| 200 | 200.0 | 5.00 | 5.37 | 6.37 | 0 | 0 |
| 250 | 250.0 | 4.00 | 4.38 | 5.54 | 0 | 0 |
| 500 | 500.3 | 2.00 | 7.57 | 8.99 | 0 | 0 |
| 1000 | 686.9 | 1.46 | 5.68 | 6.92 | 0 | 2 |

読み取り方:

- **250 Hz まで**は指定値に完全追従し、`p95` が周期の 1.07 倍程度に収まる。欠落 0。
- **500 Hz** は平均 2.00 ms と周期自体は正しいが、`p95` が 7.57 ms と周期の **3.8 倍**まで
  膨らむ。送信バッファ（128 バイト ≒ 7 フレーム）が埋まってからまとめて出るためで、
  制御ループ 用には「周期が読めない」状態になる。まだ欠落は 0。
- **1000 Hz** で初めてフレーム欠落（2 フレーム）が現れ、実測は 686.9 Hz で頭打ちになる。
  理論天井 11520 B/s ÷ 17 B = 677.6 フレーム/s とほぼ一致するので、これは线路容量の限界。
- `MSTE_FEEDBACK_HZ` は **1000 の約数のみ**指定できる（`static_assert` でコンパイル時に
  弾く）。1000 の約数でない値を指定すると `1000 / HZ` の切り捨てで「設定値より高い Hz で
  動く」状態を無警告で生むため。

**推奨値は 100 Hz**。理由:

- 線路使用率 1700 B/s ÷ 11520 B/s = **14.8%**。指令送信と将来のフィードバック種別追加に
  十分な余裕が残る
- 周期 10 ms は 20 Hz 程度のナビゲーションループに対して十分新しいエンコーダ情報を与えられる
- タイミングが安定した上限 250 Hz の 40% の負荷で、`p95` は周期の 1.07 倍に収まる

> ベンチ測定（`esp32dev_usbuart`）は TX/RX が UART0 を共有するため、送信が飽和すると
> `loop()` が `write` に占有されて RX 処理が後回しになる。実運用の `esp32dev` は
> Serial2 経由の独立线路（full duplex）でこの取り合いが起きないため、**上の値は
> 実運用に対して保守的**である。

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
