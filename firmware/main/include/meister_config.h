/*
 * meister_config.h — 設定の唯一の出所
 *
 * ピン番号・サーボ ID・軸数・値域・バックエンド選択はすべてこのファイルに
 * 集約する。以前は platformio.ini の build_flags と main.cpp の
 * #ifndef フォールバックの 2 箇所に書かれていて、片方だけ直すと
 * 不整合が起きていた。
 *
 * 優先順位:
 *   1. コンパイル時 -D（MSTE_*）— platformio.ini から与えられる
 *   2. このファイルのフォールバック（-D されていない場合）
 *
 * ★未確定な値には「★要確認★」と書いている。確定したらここだけ直す。
 * 実配線・実机型が確定していないものは value の選択で済むように
 * 「フラグ 1 個」で切り替えられるようにしてある。
 */
#pragma once

#include <stdint.h>

#include "hal/generated_config.h"
#include "meister_protocol.h"

// ===========================================================================
// 1. プロトコル UART（PC ⇄ ESP32）
// ===========================================================================

/// プロトコル用ボーレート [bps]
#ifndef MSTE_UART_BAUD
#define MSTE_UART_BAUD 115200
#endif

/// プロトコル UART の GPIO（Serial2 = UART2）。MSTE_UART_BACKEND_USB0=1 では使わない
#ifndef MSTE_UART_RX_PIN
#define MSTE_UART_RX_PIN 16
#endif
#ifndef MSTE_UART_TX_PIN
#define MSTE_UART_TX_PIN 17
#endif

/// 0: プロトコルを Serial2(GPIO16/17) に使う（実機。UART0 はバスに回せる）
/// 1: プロトコルを UART0(USB) に使う（ベンチ。バスは無効化される）
#ifndef MSTE_UART_BACKEND_USB0
#define MSTE_UART_BACKEND_USB0 0
#endif

/// 1 のときはコンソール出力を無効化する（UART0 をコンソールと共有するため）
#if MSTE_UART_BACKEND_USB0
#define MSTE_PROTO_PORT_NAME "usb0"
#define MSTE_LOG(...) ((void)0)
#else
#define MSTE_PROTO_PORT_NAME "serial2"
#define MSTE_LOG(...) Serial.printf(__VA_ARGS__)
#endif

// ===========================================================================
// 2. FB_STATE 送信周期
// ===========================================================================

/// FB_STATE の送信周期 [Hz]
#ifndef MSTE_FEEDBACK_HZ
#define MSTE_FEEDBACK_HZ 100
#endif

/// 送信間隔は 1000 / MSTE_FEEDBACK_HZ（整数ミリ秒）で決まる。1000 の約数でない値を
/// 指定すると切り捨てで「設定値より高い Hz で動く」状態を無警告で生むため弾く。
static_assert(1000 % MSTE_FEEDBACK_HZ == 0,
              "MSTE_FEEDBACK_HZ must divide 1000 (1000 % HZ == 0); "
              "e.g. 50/100/125/200/250/500 -- 1000/HZ is integer milliseconds");

// ===========================================================================
// 2b. IMU（GY-BNO055）方位送信
// ===========================================================================

/// IMU 機能の有効化。0 の間は I2C を初期化しない（未接続時のハング防止）。
#ifndef MSTE_ENABLE_IMU
#define MSTE_ENABLE_IMU 1
#endif

/// BNO055 の I2C 配線。バス無効中は空いている 25/26 を使う。
/// バス（UART1 = TX25/RX26）を有効化したらこの 2 本を再割当すること。
#ifndef MSTE_IMU_SDA_PIN
#define MSTE_IMU_SDA_PIN 25
#endif
#ifndef MSTE_IMU_SCL_PIN
#define MSTE_IMU_SCL_PIN 26
#endif

/// BNO055 の I2C アドレス。ADDR=GND で 0x28、VDD で 0x29。
/// ★要確認★ 実物の ADDR ピン結線と照合すること。
#ifndef MSTE_IMU_ADDR
#define MSTE_IMU_ADDR 0x28
#endif

/// FB_IMU の送信周期 [Hz]。FB_STATE と同じ理由で 1000 の約数のみ。
#ifndef MSTE_IMU_HZ
#define MSTE_IMU_HZ 20
#endif
static_assert(1000 % MSTE_IMU_HZ == 0,
              "MSTE_IMU_HZ must divide 1000");

// ===========================================================================
// 3. STS/SCS シリアルバスサーボ
// ===========================================================================

/// バス機能の有効化。実機接続が確定するまで 0（無効）のままにする。
/// 1 にすると UART を初期化してバスを駆動し始める。★要確認★
#ifndef MSTE_ENABLE_BUS_SERVO
#define MSTE_ENABLE_BUS_SERVO 0
#endif

/// バス用 UART ボーレート [bps]。STS/SCS の既定は 1 Mbps。★要確認★
#ifndef MSTE_BUS_BAUD
#define MSTE_BUS_BAUD 1000000
#endif

/// バス用 UART の選択。0: Serial1(GPIO UART) / 1: UART0(USB)
///   - Serial1 … バスドライバに TTL/UART 端子がある場合。方向はドライバが持つ
///   - UART0   … ESP32 の USB ポートをバスドライバの USB-C に挿す場合
///                （★WROOM-32 は USB デバイスなので、両端 USB-C の直結は
///                   列挙が成立しない。成立するのは TTL 端子経由で UART1 を使う方）
#ifndef MSTE_BUS_UART0
#define MSTE_BUS_UART0 0
#endif

/// 指令の適用周期 [Hz]。PC→ESP32 のコマンドは受信ごとに即時適用し、
/// ESP32 はこの周期で最新の指令を周期ごとに再適用する（線路上は 10〜20 Hz で十分）。
/// STS バスの SYNC_WRITE は 1 パケットで 9 台に届くので 100 Hz でも問題ない。
#ifndef MSTE_CONTROL_HZ
#define MSTE_CONTROL_HZ 100
#endif
static_assert(1000 % MSTE_CONTROL_HZ == 0,
              "MSTE_CONTROL_HZ must divide 1000");

/// バス上の状態読み戻しが 1 指令あたりに読ませる台数。
/// 半二重バスはスレーブごとに応答の turnaround（0.5〜2 ms）が入り、
/// 9 台をまとめて読むと 25〜40 ms かかる。よって全台の更新は遅く、
/// 1 回の応答で 2〜3 台分を更新するラウンドロビンにする。
/// 実測で turnaround を詰め、これで足りなければ増やす。
#ifndef MSTE_BUS_READ_CHUNK
#define MSTE_BUS_READ_CHUNK 2
#endif

/// スレーブの応答待ちタイムアウト [us]。
/// ★要確認★ STS3215 データシートの応答時間項目と照合すること。
/// ここでは 1 Mbps での最長パケット換算（250 B ≒ 2500 us）に turnaround の
/// 余裕を足した保守値。过小だと応答を取りこぼし、過大だと待たされる。
#ifndef MSTE_BUS_RESP_TIMEOUT_US
#define MSTE_BUS_RESP_TIMEOUT_US 3000
#endif

/// 1 パケット送信後に次のパケットを出すまでの間隔 [us]。
/// ★要確認★ データシートの「フレーム間隔」項目。バスが過負荷で落ちないための値。
#ifndef MSTE_BUS_INTER_FRAME_GAP_US
#define MSTE_BUS_INTER_FRAME_GAP_US 200
#endif

/// 制御ループの周期。指令の適用と 1 回のバス応答をここに割り当てる。

/// 半二重の TX/RX 切替ピン。アダプタ側が面倒を見るなら -1（不使用）
#ifndef MSTE_BUS_DIR_PIN
#define MSTE_BUS_DIR_PIN -1
#endif

#if MSTE_BUS_UART0
#define MSTE_BUS_PORT_NAME "uart0"
#else
#define MSTE_BUS_PORT_NAME "serial1"
#endif

// ===========================================================================
// 4. アクチュエータ構成
//
// ★要確認★ 以下は 2026-09-27 時点の BOM 画像に基づく**計画値**。
// 実物の確認で変わる前提で、値はここ 1 ファイルに閉じ込めている。
// プロトコルの軸数もここから導出するため、構成が変わっても
// 1 箇所の修正で済む。
// ===========================================================================

namespace meister {
namespace config {

// ---- 通信（ログ表示用。ポートの選択と開始処理は上記 ProtoPort / ProtoPortBegin）----

constexpr const char* kProtoPortName = MSTE_PROTO_PORT_NAME;
/// FB_STATE の送信間隔 [ms]

// ---- IMU（GY-BNO055）。ピンと周期の唯一の出所は上の MSTE_IMU_* ----
constexpr bool kImuEnabled = MSTE_ENABLE_IMU != 0;
constexpr uint8_t kImuSdaPin = MSTE_IMU_SDA_PIN;
constexpr uint8_t kImuSclPin = MSTE_IMU_SCL_PIN;
constexpr uint8_t kImuAddr = MSTE_IMU_ADDR;
constexpr uint32_t kImuIntervalMs = 1000u / MSTE_IMU_HZ;

// ---- 軸数 ----
// kNumDriveMotors / kNumArmServos / kNumSteeringServos はプロトコルの定義なので
// firmware/main/include/meister_protocol.h が持つ。ここでは重複定義しない。

// ステアリング台数は Phase 2 で proto::kNumSteeringServos を 6→4 に
// 揃えた時点で初めてここに定義する。

/// ロッカー軸ボギー軸（DS サーボ 150kg / PWM）

/// アーム肩（DS サーボ 150kg / PWM）

/// アーム肘・手首・肩方位角（STS3215・バス）
constexpr uint8_t kNumArmBus = 3;
/// グリッパー開閉①②（STS3215・バス）
constexpr uint8_t kNumGripper = 2;

// ---- 値域 ----
// kMinVelocity / kMaxVelocity / kMinSteering / kMaxSteering / kMinArmAngle /
// kMaxArmAngle もプロトコルの定義なのでここでは持たない。
// 機構固有の範囲だけをここに置く。

// ---- バスサーボの ID 割当（★要確認★ 実機の ID 設定と一致させる）----
// 1..4 = ステアリング、5..9 = アーム
constexpr uint8_t kSteeringFirstId = 1;
constexpr uint8_t kArmFirstId      = 5;

constexpr uint8_t SteeringId(uint8_t index) {
  return kSteeringFirstId + index;
}
constexpr uint8_t ArmId(uint8_t index) {
  return kArmFirstId + index;
}

// ---- ピン割当（ESP32-WROOM-32）
//
// 使える GPIO: 4,5,13,14,18,19,21,22,23,25,26,27,32,33 + 16,17（プロトコル）
// 除外: 0/2/12/15（ストラップ）、6-11（フラッシュ）、34-39（入力のみ）、1/3（コンソール）
//
// ★要確認★ バスを UART1 にする場合（既定）と UART0 にする場合で
// ピン使用が変わる。バスを UART0 にするなら UART1 の 3 本が空く。
// ----
/// DC モータの PWM 駆動は廃止（駆動は foc drive 基板）。
/// 空いた 25/26 は IMU の I2C（SDA/SCL）に使う。

/// バス UART の GPIO（MSTE_BUS_UART0=0 のときだけ使う）
#ifndef MSTE_BUS_TX_PIN
#define MSTE_BUS_TX_PIN 25
#endif
#ifndef MSTE_BUS_RX_PIN
#define MSTE_BUS_RX_PIN 26
#endif

/// IMU の I2C（既定 SDA25/SCL26）はバス UART1（TX25/RX26）と排他。
/// 両方 1 にしたらビルド時に落とす（無言の配線衝突を作らない）。
#if MSTE_ENABLE_IMU && MSTE_ENABLE_BUS_SERVO && !MSTE_BUS_UART0
#error "MSTE_IMU_* pins (25/26) conflict with bus UART1 (25/26). Reassign IMU pins or use MSTE_BUS_UART0=1."
#endif

/// PWM サーボのピン。DS サーボ 150kg ×5（ロッカー 4 + 肩 1）
/// ★要確認★ 実配線。緑灯時に 50Hz で駆動する。
#ifndef MSTE_PWM_SERVO_PIN0
#define MSTE_PWM_SERVO_PIN0 4
#define MSTE_PWM_SERVO_PIN1 5
#define MSTE_PWM_SERVO_PIN2 23
#define MSTE_PWM_SERVO_PIN3 32
#define MSTE_PWM_SERVO_PIN4 33
#endif

/// ステアリング PWM ピン（6ch）。いまの配線は上のバス構成ではなく PWM で駆動している
#ifndef MSTE_STEER_PIN0
#define MSTE_STEER_PIN0 4
#define MSTE_STEER_PIN1 5
#define MSTE_STEER_PIN2 15
#define MSTE_STEER_PIN3 18
#define MSTE_STEER_PIN4 19
#define MSTE_STEER_PIN5 21
#endif

/// アーム PWM ピン（4ch）
#ifndef MSTE_ARM_PIN0
#define MSTE_ARM_PIN0 22
#define MSTE_ARM_PIN1 23
#define MSTE_ARM_PIN2 32
#define MSTE_ARM_PIN3 33
#endif

/// グリッパー PWM ピン（1ch）。GPIO12 はストラップピン。★要確認★
#ifndef MSTE_GRIPPER_PIN
#define MSTE_GRIPPER_PIN 12
#endif

/// サーボ PWM の分解能 [bit]（DC モータ PWM 廃止のため周波数定義は持たない）

// ---- PWM パラメータ ----

// ---- いまの配線で機構が読む PWM ピン表 ----
// ステアリングをバスへ移す計画が進んだら、この表ごと置き換える。


/// LEDC チャネル数の上限。ESP32-WROOM-32 は 16。
/// ステア 6 + アーム 4 + グリッパー 1 = 11 で収まる（DC モータ PWM 廃止）。

/// 使用する LEDC チャネル数（静的チェック用）

}  // namespace config
}  // namespace meister
