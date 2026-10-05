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
#ifndef MSTE_MOTOR_PIN0
#define MSTE_MOTOR_PIN0 13
#define MSTE_MOTOR_PIN1 14
#define MSTE_MOTOR_PIN2 18
#define MSTE_MOTOR_PIN3 19
#define MSTE_MOTOR_PIN4 21
#define MSTE_MOTOR_PIN5 22
#endif

/// モータの向きピン。-1 = 未接続（PWM のみ）
#ifndef MSTE_MOTOR_DIR0
#define MSTE_MOTOR_DIR0 -1
#define MSTE_MOTOR_DIR1 -1
#define MSTE_MOTOR_DIR2 -1
#define MSTE_MOTOR_DIR3 -1
#define MSTE_MOTOR_DIR4 -1
#define MSTE_MOTOR_DIR5 -1
#endif

/// バス UART の GPIO（MSTE_BUS_UART0=0 のときだけ使う）
#ifndef MSTE_BUS_TX_PIN
#define MSTE_BUS_TX_PIN 25
#endif
#ifndef MSTE_BUS_RX_PIN
#define MSTE_BUS_RX_PIN 26
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

/// モータ PWM の周波数 [Hz] と分解能 [bit]
#ifndef MSTE_MOTOR_PWM_FREQ
#define MSTE_MOTOR_PWM_FREQ 1000
#endif
#ifndef MSTE_PWM_RESOLUTION
#define MSTE_PWM_RESOLUTION 8
#endif

// ---- PWM パラメータ ----

// ---- いまの配線で機構が読む PWM ピン表 ----
// ステアリングをバスへ移す計画が進んだら、この表ごと置き換える。


/// LEDC チャネル数の上限。ESP32-WROOM-32 は 16。
/// モータ 6 + PWM サーボ 5 = 11 で収まる。DIR ピンを使うようになると逼迫する。

/// 使用する LEDC チャネル数（静的チェック用）

}  // namespace config
}  // namespace meister
