/*
 * main.cpp — Meister ESP32 ファームウェア
 *
 * 半分散型アーキテクチャの ESP32 側を実装する。
 *   - PC (ROS 2) からのバイナリフレームを UART (Serial2) で受信・パース
 *   - 駆動モータ 6ch を LEDC PWM で駆動（速度指令 → デューティ比 + 方向）
 *   - ステアリング 6ch / アーム 4ch / グリッパー 1ch のサーボ制御
 *     （IServoChannel 抽象 → LEDC PWM 実装 / デバッグコンソール実装）
 *   - 定周期でフィードバックフレーム（エンコーダ/状態/エラー）を送信
 *
 * ピン配置・ボーレート・サーボドライバは platformio.ini の build_flags
 * （MSTE_* マクロ）で変更できる。
 *
 * 注意: このファイルは Arduino 依存のため、`#ifdef ARDUINO` でガードして
 * いる。native テスト環境ではコンパイル対象外となり、プロトコル層
 * （meister_protocol.*）だけがホスト側でテストされる。
 */

#ifdef ARDUINO

#include <Arduino.h>

#include "meister_protocol.h"

namespace {
using namespace meister;

// ===========================================================================
// 1. ピン・パラメータ定義（build_flags で上書き可能）
// ===========================================================================

// ---- UART（プロトコル通信）----
#ifndef MSTE_UART_BAUD
#define MSTE_UART_BAUD 115200
#endif
#ifndef MSTE_UART_RX_PIN
#define MSTE_UART_RX_PIN 16
#endif
#ifndef MSTE_UART_TX_PIN
#define MSTE_UART_TX_PIN 17
#endif

// ---- 駆動モータ PWM ピン（6ch）----
#ifndef MSTE_MOTOR_PIN0
#define MSTE_MOTOR_PIN0 25
#endif
#ifndef MSTE_MOTOR_PIN1
#define MSTE_MOTOR_PIN1 26
#endif
#ifndef MSTE_MOTOR_PIN2
#define MSTE_MOTOR_PIN2 27
#endif
#ifndef MSTE_MOTOR_PIN3
#define MSTE_MOTOR_PIN3 14
#endif
#ifndef MSTE_MOTOR_PIN4
#define MSTE_MOTOR_PIN4 13
#endif
#ifndef MSTE_MOTOR_PIN5
#define MSTE_MOTOR_PIN5 2
#endif

// ---- 駆動モータ 方向ピン（-1 = 未接続 → PWM のみ）----
#ifndef MSTE_MOTOR_DIR0
#define MSTE_MOTOR_DIR0 -1
#endif
#ifndef MSTE_MOTOR_DIR1
#define MSTE_MOTOR_DIR1 -1
#endif
#ifndef MSTE_MOTOR_DIR2
#define MSTE_MOTOR_DIR2 -1
#endif
#ifndef MSTE_MOTOR_DIR3
#define MSTE_MOTOR_DIR3 -1
#endif
#ifndef MSTE_MOTOR_DIR4
#define MSTE_MOTOR_DIR4 -1
#endif
#ifndef MSTE_MOTOR_DIR5
#define MSTE_MOTOR_DIR5 -1
#endif

// ---- ステアリングサーボ PWM ピン（6ch）----
#ifndef MSTE_STEER_PIN0
#define MSTE_STEER_PIN0 4
#endif
#ifndef MSTE_STEER_PIN1
#define MSTE_STEER_PIN1 5
#endif
#ifndef MSTE_STEER_PIN2
#define MSTE_STEER_PIN2 15
#endif
#ifndef MSTE_STEER_PIN3
#define MSTE_STEER_PIN3 18
#endif
#ifndef MSTE_STEER_PIN4
#define MSTE_STEER_PIN4 19
#endif
#ifndef MSTE_STEER_PIN5
#define MSTE_STEER_PIN5 21
#endif

// ---- アームサーボ PWM ピン（4ch）----
#ifndef MSTE_ARM_PIN0
#define MSTE_ARM_PIN0 22
#endif
#ifndef MSTE_ARM_PIN1
#define MSTE_ARM_PIN1 23
#endif
#ifndef MSTE_ARM_PIN2
#define MSTE_ARM_PIN2 32
#endif
#ifndef MSTE_ARM_PIN3
#define MSTE_ARM_PIN3 33
#endif

// ---- グリッパーサーボ PWM ピン（1ch）----
#ifndef MSTE_GRIPPER_PIN
#define MSTE_GRIPPER_PIN 12
#endif

// ---- PWM パラメータ ----
#ifndef MSTE_MOTOR_PWM_FREQ
#define MSTE_MOTOR_PWM_FREQ 1000  // 駆動モータ PWM 周波数 [Hz]
#endif
#ifndef MSTE_PWM_RESOLUTION
#define MSTE_PWM_RESOLUTION 8  // 駆動モータ PWM 分解能 [bit]
#endif

constexpr uint32_t kMotorFreqHz = MSTE_MOTOR_PWM_FREQ;
constexpr uint8_t kMotorResolutionBits = MSTE_PWM_RESOLUTION;

constexpr uint32_t kServoFreqHz = 50;   // サーボ PWM 周波数（50Hz = 20ms 周期）
constexpr uint8_t kServoResolutionBits = 16;

// ===========================================================================
// 2. LEDC ラッパ（arduino-esp32 2.x / 3.x 両対応）
// ===========================================================================
// 3.x は pin ベース API（ledcAttach / ledcWrite）、2.x はチャネルベース API。
namespace ledc {

#if ESP_ARDUINO_VERSION_MAJOR < 3
int8_t sChannelOfPin[40];  // pin → LEDC チャネル（-1 = 未使用）
uint8_t sNextChannel = 0;
#endif

bool attach(uint8_t pin, uint32_t freq, uint8_t resolutionBits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ::ledcAttach(pin, freq, resolutionBits);
#else
  if (pin >= 40 || sNextChannel >= 16) {
    return false;
  }
  const uint8_t ch = sNextChannel++;
  if (!::ledcSetup(ch, freq, resolutionBits)) {
    return false;
  }
  if (!::ledcAttachPin(pin, ch)) {
    return false;
  }
  sChannelOfPin[pin] = static_cast<int8_t>(ch);
  return true;
#endif
}

void write(uint8_t pin, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ::ledcWrite(pin, duty);
#else
  if (pin < 40 && sChannelOfPin[pin] >= 0) {
    ::ledcWrite(static_cast<uint8_t>(sChannelOfPin[pin]), duty);
  }
#endif
}

}  // namespace ledc

// ===========================================================================
// 3. 駆動モータ制御（速度指令 → PWM デューティ比 + 方向）
// ===========================================================================
class MotorChannel {
 public:
  MotorChannel(uint8_t pwmPin, int8_t dirPin)
      : pwmPin_(pwmPin), dirPin_(dirPin) {}

  bool begin() {
    if (pwmPin_ != kNoPin && !ledc::attach(pwmPin_, kMotorFreqHz, kMotorResolutionBits)) {
      return false;
    }
    if (dirPin_ >= 0) {
      pinMode(dirPin_, OUTPUT);
      digitalWrite(dirPin_, LOW);
    }
    return true;
  }

  /// vel: -1000..+1000（千分率）。符号 → 方向、絶対値 → デューティ比
  void setVelocity(int16_t vel) {
    if (vel > proto::kMaxVelocity) {
      vel = proto::kMaxVelocity;
    }
    if (vel < proto::kMinVelocity) {
      vel = proto::kMinVelocity;
    }
    const bool forward = vel >= 0;
    const uint16_t absVel = forward ? static_cast<uint16_t>(vel)
                                    : static_cast<uint16_t>(-vel);
    const uint32_t maxDuty = (1u << kMotorResolutionBits) - 1u;
    const uint32_t duty = absVel * maxDuty / static_cast<uint32_t>(proto::kMaxVelocity);
    if (dirPin_ >= 0) {
      digitalWrite(dirPin_, forward ? HIGH : LOW);
    }
    ledc::write(pwmPin_, duty);
  }

  void stop() { setVelocity(0); }

 private:
  static constexpr int8_t kNoPin = -1;
  uint8_t pwmPin_;
  int8_t dirPin_;
};

// ===========================================================================
// 4. サーボ制御（HAL 抽象 → LEDC PWM 実装 / デバッグコンソール実装）
// ===========================================================================
class IServoChannel {
 public:
  virtual ~IServoChannel() = default;
  virtual bool begin() = 0;
  /// angleTenths: 0.1° 単位（ステアリング -900..900 / アーム 0..1800）
  virtual bool setAngleTenths(int16_t angleTenths) = 0;
  virtual void neutral() = 0;
};

/// LEDC PWM によるサーボ駆動（50Hz、パルス幅 minUs..maxUs）
class PwmServoChannel : public IServoChannel {
 public:
  PwmServoChannel(uint8_t pin, int16_t rangeMinTenths, int16_t rangeMaxTenths,
                  uint16_t minUs = 1000, uint16_t maxUs = 2000)
      : pin_(pin),
        rangeMinTenths_(rangeMinTenths),
        rangeMaxTenths_(rangeMaxTenths),
        minUs_(minUs),
        maxUs_(maxUs) {}

  bool begin() override {
    attached_ = ledc::attach(pin_, kServoFreqHz, kServoResolutionBits);
    if (attached_) {
      neutral();
    }
    return attached_;
  }

  bool setAngleTenths(int16_t tenths) override {
    if (!attached_) {
      return false;
    }
    if (tenths < rangeMinTenths_) {
      tenths = rangeMinTenths_;
    }
    if (tenths > rangeMaxTenths_) {
      tenths = rangeMaxTenths_;
    }
    // 角度 → パルス幅 [us] → デューティ比
    const int32_t span = static_cast<int32_t>(rangeMaxTenths_) - rangeMinTenths_;
    const uint32_t us = minUs_ + static_cast<uint32_t>(tenths - rangeMinTenths_) *
                                     (static_cast<uint32_t>(maxUs_) - minUs_) /
                                     static_cast<uint32_t>(span);
    ledc::write(pin_, PulseUsToDuty(us));
    return true;
  }

  void neutral() override {
    const int16_t mid = static_cast<int16_t>(rangeMinTenths_ +
                                             (static_cast<int32_t>(rangeMaxTenths_) - rangeMinTenths_) / 2);
    setAngleTenths(mid);
  }

 private:
  static uint32_t PulseUsToDuty(uint32_t us) {
    return us * (1u << kServoResolutionBits) * kServoFreqHz / 1000000u;
  }
  uint8_t pin_;
  int16_t rangeMinTenths_;
  int16_t rangeMaxTenths_;
  uint16_t minUs_;
  uint16_t maxUs_;
  bool attached_ = false;
};

/// デバッグ用: 実サーボの代わりにコンソールへ角度を表示する
class ConsoleServoChannel : public IServoChannel {
 public:
  ConsoleServoChannel(uint8_t pin, int16_t rangeMinTenths, int16_t rangeMaxTenths)
      : rangeMinTenths_(rangeMinTenths), rangeMaxTenths_(rangeMaxTenths) {
    snprintf(name_, sizeof(name_), "servo%u", pin);
  }

  bool begin() override { return true; }

  bool setAngleTenths(int16_t tenths) override {
    if (tenths < rangeMinTenths_) {
      tenths = rangeMinTenths_;
    }
    if (tenths > rangeMaxTenths_) {
      tenths = rangeMaxTenths_;
    }
    Serial.printf("[servo] %-12s angle = %+7.1f deg\n", name_, tenths / 10.0f);
    return true;
  }

  void neutral() override {
    const int16_t mid = static_cast<int16_t>(rangeMinTenths_ +
                                             (static_cast<int32_t>(rangeMaxTenths_) - rangeMinTenths_) / 2);
    setAngleTenths(mid);
  }

 private:
  char name_[16];
  int16_t rangeMinTenths_;
  int16_t rangeMaxTenths_;
};

// ドライバ選択: MSTE_SERVO_DRIVER_DEBUG 定義時はコンソール出力に切替
#if defined(MSTE_SERVO_DRIVER_DEBUG)
using SteerServoImpl = ConsoleServoChannel;
using ArmServoImpl = ConsoleServoChannel;
using GripperServoImpl = ConsoleServoChannel;
#else
using SteerServoImpl = PwmServoChannel;
using ArmServoImpl = PwmServoChannel;
using GripperServoImpl = PwmServoChannel;
#endif

/// グリッパー: 開/閉の 2 ポジション制御（サーボチャネルを内包）
class Gripper {
 public:
  explicit Gripper(IServoChannel& servo) : servo_(servo) {}

  void setCommand(proto::GripperCommand cmd) {
    switch (cmd) {
      case proto::kGripperOpen:
        servo_.setAngleTenths(kOpenTenths);  // 開
        break;
      case proto::kGripperClose:
        servo_.setAngleTenths(kCloseTenths);  // 閉
        break;
      case proto::kGripperStop:
      default:
        break;  // 現在値を保持
    }
  }

 private:
  static constexpr int16_t kOpenTenths = 100;   // 10.0°（開）— 実機に合わせて調整
  static constexpr int16_t kCloseTenths = 0;    // 0.0°（閉）
  IServoChannel& servo_;
};

// ===========================================================================
// 5. インスタンス生成
// ===========================================================================
static MotorChannel motors[proto::kNumDriveMotors] = {
    MotorChannel(MSTE_MOTOR_PIN0, MSTE_MOTOR_DIR0),
    MotorChannel(MSTE_MOTOR_PIN1, MSTE_MOTOR_DIR1),
    MotorChannel(MSTE_MOTOR_PIN2, MSTE_MOTOR_DIR2),
    MotorChannel(MSTE_MOTOR_PIN3, MSTE_MOTOR_DIR3),
    MotorChannel(MSTE_MOTOR_PIN4, MSTE_MOTOR_DIR4),
    MotorChannel(MSTE_MOTOR_PIN5, MSTE_MOTOR_DIR5),
};

static SteerServoImpl steerServos[proto::kNumSteeringServos] = {
    SteerServoImpl(MSTE_STEER_PIN0, proto::kMinSteering, proto::kMaxSteering),
    SteerServoImpl(MSTE_STEER_PIN1, proto::kMinSteering, proto::kMaxSteering),
    SteerServoImpl(MSTE_STEER_PIN2, proto::kMinSteering, proto::kMaxSteering),
    SteerServoImpl(MSTE_STEER_PIN3, proto::kMinSteering, proto::kMaxSteering),
    SteerServoImpl(MSTE_STEER_PIN4, proto::kMinSteering, proto::kMaxSteering),
    SteerServoImpl(MSTE_STEER_PIN5, proto::kMinSteering, proto::kMaxSteering),
};

static ArmServoImpl armServos[proto::kNumArmServos] = {
    ArmServoImpl(MSTE_ARM_PIN0, proto::kMinArmAngle, proto::kMaxArmAngle),
    ArmServoImpl(MSTE_ARM_PIN1, proto::kMinArmAngle, proto::kMaxArmAngle),
    ArmServoImpl(MSTE_ARM_PIN2, proto::kMinArmAngle, proto::kMaxArmAngle),
    ArmServoImpl(MSTE_ARM_PIN3, proto::kMinArmAngle, proto::kMaxArmAngle),
};

static GripperServoImpl gripperServoChannel(MSTE_GRIPPER_PIN,
                                            proto::kMinArmAngle, proto::kMaxArmAngle);
static Gripper gripper(gripperServoChannel);

// ===========================================================================
// 6. UART 受信パース（ステートマシン）
// ===========================================================================
namespace rx {
enum class State : uint8_t {
  kWaitHeader,  // ヘッダ 0xA5 を待機
  kWaitType,    // 種別を待機
  kPayload,     // ペイロード収集
  kChecksum,    // チェックサム受信 → 検証
};

State state = State::kWaitHeader;
uint8_t buf[proto::kMaxFrameSize];
size_t idx = 0;          // 書込中インデックス
size_t payloadLeft = 0;  // ペイロード残バイト数
}  // namespace rx

static volatile uint32_t sRxErrorCount = 0;  // フレーム破損回数

void onFrameComplete(const uint8_t* data, size_t len);

/// 受信バイト 1 個をステートマシンで処理する
void onRxByte(uint8_t b) {
  using namespace proto;
  switch (rx::state) {
    case rx::State::kWaitHeader:
      if (b == kHeaderByte) {
        rx::buf[0] = b;
        rx::state = rx::State::kWaitType;
      }
      break;

    case rx::State::kWaitType: {
      const TypeId type = static_cast<TypeId>(b);
      if (PayloadSize(type) == 0) {
        rx::state = rx::State::kWaitHeader;  // 未知種別 → 再同期
        return;
      }
      rx::buf[1] = b;
      rx::payloadLeft = PayloadSize(type);
      rx::idx = 2;
      rx::state = (rx::payloadLeft > 0) ? rx::State::kPayload : rx::State::kChecksum;
      break;
    }

    case rx::State::kPayload:
      if (rx::idx < proto::kMaxFrameSize) {
        rx::buf[rx::idx++] = b;
      }
      if (--rx::payloadLeft == 0) {
        rx::state = rx::State::kChecksum;
      }
      break;

    case rx::State::kChecksum:
      if (rx::idx < proto::kMaxFrameSize) {
        rx::buf[rx::idx++] = b;
      }
      onFrameComplete(rx::buf, rx::idx);
      rx::state = rx::State::kWaitHeader;
      break;
  }
}

// ===========================================================================
// 7. フレーム処理（検証 → 指令ディスパッチ）
// ===========================================================================
void onFrameComplete(const uint8_t* data, size_t len) {
  using namespace proto;
  Frame frame;
  const ParseResult result = ParseFrame(data, len, &frame);
  if (result != ParseResult::kOk) {
    ++sRxErrorCount;
    Serial.printf("[rx] invalid frame: %s (count=%lu)\n", ParseResultName(result),
                  static_cast<unsigned long>(sRxErrorCount));
    return;
  }

  switch (frame.type) {
    case kCmdMotorVelocity:
      for (size_t i = 0; i < kNumDriveMotors; ++i) {
        motors[i].setVelocity(FrameGetInt16(frame, i * 2));
      }
      break;

    case kCmdSteeringAngle:
      for (size_t i = 0; i < kNumSteeringServos; ++i) {
        steerServos[i].setAngleTenths(FrameGetInt16(frame, i * 2));
      }
      break;

    case kCmdArmAngle:
      for (size_t i = 0; i < kNumArmServos; ++i) {
        armServos[i].setAngleTenths(FrameGetInt16(frame, i * 2));
      }
      break;

    case kCmdGripper:
      gripper.setCommand(static_cast<GripperCommand>(FrameGetU8(frame, 0)));
      break;

    default:  // FB_STATE / FB_ERROR など上り方向は受信側では無視
      break;
  }
}

// ===========================================================================
// 8. フィードバック送信（定周期）
// ===========================================================================
void sendFeedbackIfDue() {
  constexpr uint32_t kIntervalMs = 100;  // 10 Hz
  static uint32_t lastSendMs = 0;
  const uint32_t now = millis();
  if (now - lastSendMs < kIntervalMs) {
    return;
  }
  lastSendMs = now;

  // エンコーダ値はプレースホルダ（将来は実エンコーダから読む）
  int16_t encoders[proto::kNumDriveMotors];
  for (size_t i = 0; i < proto::kNumDriveMotors; ++i) {
    encoders[i] = static_cast<int16_t>(((now / 100) * 3 + i * 137) % 2000 - 1000);
  }

  uint8_t buf[proto::kMaxFrameSize];
  uint8_t errorFlags = proto::kFbStateNormal;
  if (sRxErrorCount > 0) {
    errorFlags |= proto::kFbErrorProtocol;
  }
  const size_t n = proto::EncodeState(buf, sizeof(buf), encoders, 0, errorFlags);
  Serial2.write(buf, n);
}

}  // namespace

// ===========================================================================
// 9. setup / loop（Arduino フレームワークから参照されるためグローバル定義）
// ===========================================================================
void setup() {
  Serial.begin(115200);  // コンソール（UART0）

  // プロトコル用 UART（既定: Serial2 = GPIO16/17）
  Serial2.begin(MSTE_UART_BAUD, SERIAL_8N1, MSTE_UART_RX_PIN, MSTE_UART_TX_PIN);

  for (auto& m : motors) {
    m.begin();
  }
  for (auto& s : steerServos) {
    s.begin();
  }
  for (auto& s : armServos) {
    s.begin();
  }
  gripperServoChannel.begin();

  Serial.println("[meister-esp] boot OK");
  Serial.printf("[meister-esp] uart=%d baud=%lu\n", 2, static_cast<unsigned long>(MSTE_UART_BAUD));
}

void loop() {
  // 受信バイトをすべて処理（ノンブロッキング）
  while (Serial2.available() > 0) {
    onRxByte(static_cast<uint8_t>(Serial2.read()));
  }
  sendFeedbackIfDue();
}

#endif  // ARDUINO
