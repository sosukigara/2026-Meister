/*
 * 機構のインスタンスは CommandDispatch のメンバとして 1 か所で構築する。
 * 静的初期化順（翻訳単位間の順序）に依存しないため（R11）。
 * Arduino 依存のため #ifdef ARDUINO でガードし、native テストでは
 * プロトコル層（meister_protocol.*）だけをホスト側でテストする。
 */

#ifdef ARDUINO

#include <Arduino.h>

#include "command_dispatch.h"
#include "feedback.h"
#include "hal/ledc_pwm.h"
#include "hal/proto_uart.h"
#include "imu_reporter.h"
#include "meister_config.h"
#include "meister_protocol.h"

namespace {

meister::CommandDispatch gDispatch;
meister::Feedback gFeedback;
meister::ImuReporter gImu;

}  // namespace

void setup() {
  meister::hal::proto_uart::begin();
  gDispatch.begin();
  gImu.begin();

  MSTE_LOG("[meister-esp] boot OK\n");
  MSTE_LOG("[ledc] attached=%u failed=%u\n",
           meister::hal::ledc::attached_count(),
           meister::hal::ledc::failed_count());
  MSTE_LOG("[meister-esp] uart=%s baud=%lu\n", meister::config::kProtoPortName,
           static_cast<unsigned long>(meister::config::kProtoBaud));
}

void loop() {
  // 受信バイトをすべて処理（ノンブロッキング）
  while (meister::hal::proto_uart::available() > 0) {
    gDispatch.feedRxByte(static_cast<uint8_t>(meister::hal::proto_uart::read()));
  }
  gImu.tick();
  // IMU 死活は FB_STATE の sensor ビットに畳む（上り種別を増やさない）。
  const uint8_t errors = static_cast<uint8_t>(
      gDispatch.rxErrorFlags() | (gImu.ok() ? 0 : meister::proto::kFbErrorSensor));
  gFeedback.tick(errors);
}

#endif  // ARDUINO
