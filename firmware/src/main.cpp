/*
 * main.cpp — ファームウェアの配線とメインループ
 *
 * 処理は下の各層に分けている。依存の向きは片方向のみ。
 *   main → command_dispatch → 機構（base_chassis / arm）→ hal
 * 機構のインスタンスは CommandDispatch のメンバとして 1 か所で構築され、
 * 静的初期化順（翻訳単位間の順序）に依存しない。
 *
 * 注意: このファイルは Arduino 依存のため、`#ifdef ARDUINO` でガードして
 * いる。native テスト環境ではコンパイル対象外となり、プロトコル層
 * （meister_protocol.*）だけがホスト側でテストされる。
 */

#ifdef ARDUINO

#include <Arduino.h>

#include "command_dispatch.h"
#include "feedback.h"
#include "hal/ledc_pwm.h"
#include "hal/proto_uart.h"
#include "meister_config.h"

namespace {

meister::CommandDispatch gDispatch;
meister::Feedback gFeedback;

}  // namespace

void setup() {
  meister::hal::proto_uart::begin();
  gDispatch.begin();

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
  gFeedback.tick(gDispatch.rxErrorFlags());
}

#endif  // ARDUINO
