# 設計: ESP32 ↔ PC UART 通信

> 対象機能: [features/07-esp32-uart.md](../features/07-esp32-uart.md) | 作成日: 2026-08-12 | ステータス: ベース実装済み（PC側ブリッジ/モックは未実装）
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収) [G3](../やりたいこと.md#g3-手動操縦も可能)

## 設計メモ

- **アーキテクチャ**: 半分散型。リアルタイム制御（PWM 生成・サーボバス制御）は ESP32、高度な判断（ナビ・認識・把持計画）は PC の ROS 2。
- **cmd_vel → アクチュエータ変換**: PC 側ブリッジ `meister_serial_bridge`（`/cmd_vel` → 自転車モデルで舵角 `atan(wheelbase·wz/vx)` と駆動千分率 `vx/vmax` に変換し、全 6 チャネルへ同一値を送信）。アッカーマン系のためその場旋回は不可（|vx|≈0 で |wz|>0 のときは最大舵角 + 前進 0%）。実機 nav は `use_rotate_to_heading: false` 推奨。
- **プロトコル推奨**: カスタムバイナリ（固定長フレーム + チェックサム）が決定的で堅牢。テキスト（JSON/改行区切り）はデバッグに有利だが非決定的。
- **ボーレート目安**: 115200 bps（最低ライン）。必要に応じて 921600 bps まで拡張。
- **配線**: PC ↔ ESP32 は USB-UART 変換 or ESP32 内蔵 USB。GND 共通必須。
- **電源分離**: サーボ・モータ駆動系とロジック系の電源分離を検討（ノイズ対策）。

## 実装済みプロトコル仕様

ESP32 ファームウェアのベース実装（[firmware/](../../firmware/)）に合わせ、以下が確定・実装済み。
詳細な定数定義は `firmware/include/meister_protocol.h` を参照。

- **フレームフォーマット**: 固定長バイナリ、リトルエンディアン。
  - `[0]` ヘッダ `0xA5`（同期バイト）
  - `[1]` 種別 `TypeId`（1 バイト）
  - `[2..L-2]` ペイロード（種別ごとに固定長）
  - `[L-1]` チェックサム（ペイロード末尾までの全バイトの XOR）
- **チェックサム検証**: フレーム全体（チェックサム含む）を XOR して 0 になれば正常。
- **フレーム長**: `L = 2 + PayloadSize(種別) + 1`（種別から一意に決まる固定長）
- **ボーレート**: 115200 bps（`MSTE_UART_BAUD` で変更可、既定は Serial2 = GPIO16/17）

### コマンド（PC → ESP32、下り）

| 種別 | ID | ペイロード | フレーム長 |
|---|---|---|---|
| CMD_MOTOR_VELOCITY | 0x01 | `int16 vel[6]` 各輪速度 -1000..+1000（千分率） | 15 |
| CMD_STEERING_ANGLE | 0x02 | `int16 ang[6]` 舵角 -900..+900（0.1° 単位） | 15 |
| CMD_ARM_ANGLE | 0x03 | `int16 ang[4]` 関節角 0..1800（0.1° 単位） | 11 |
| CMD_GRIPPER | 0x04 | `uint8 cmd` 0=閉 / 1=開 / 2=停止 | 4 |

### フィードバック（ESP32 → PC、上り）

| 種別 | ID | ペイロード | フレーム長 |
|---|---|---|---|
| FB_STATE | 0x81 | `int16 enc[6]` + `uint8 state` + `uint8 error_flags` | 17 |
| FB_ERROR | 0x82 | `uint8 error_code` | 4 |

### 実装状況

- ESP32 ファームウェアのベース実装済み（PlatformIO、`pio run` でコンパイル検証済み）
- 受信パース（ヘッダ/種別/ペイロード/チェックサムのステートマシン）、LEDC PWM によるモータ 6ch・サーボ（ステアリング 6ch / アーム 4ch / グリッパー 1ch）制御、10 Hz の FB_STATE 定期送信を実装
- PC 側ブリッジ実装済み（`meister_serial_bridge`、/cmd_vel → 舵角+PWM 変換）。モック ESP32 は未実装（後続）

## 設計図化対象

- [x] フレームフォーマット定義（固定長バイナリ: ヘッダ / 種別 / ペイロード / チェックサム）… 実装済み（`firmware/include/meister_protocol.h`）
- [x] コマンド/フィードバックの種別一覧（モータ速度・舵角・サーボ角度・エンコーダ・状態・エラー）… 実装済み
- [ ] シーケンス図（PC からの指令送信 → ESP32 応答 → フィードバック受信）
- [ ] エラー処理設計（フレーム破損・タイムアウト・ESP32 側の異常通知）

## 関連ドキュメント

- 機能要件: [features/07-esp32-uart.md](../features/07-esp32-uart.md)
- 機能分解: [functions/07-esp32-uart.md](../functions/07-esp32-uart.md)
- 関連設計: [08-rocker-bogie.md](08-rocker-bogie.md)（モータ/サーボの制御先） / [09-object-grasping.md](09-object-grasping.md)（アームサーボの制御先） / [10-auto-control.md](10-auto-control.md)（cmd_vel の受信経路）
