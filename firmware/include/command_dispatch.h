/*
 * command_dispatch.h — 受信フレーム → 各機構への指令振り分け
 *
 * 1 バイトずつ溜めるステートマシン（ヘッダ → 種別 → ペイロード →
 * チェックサム）をここで完結させ、フレームが揃ったら機構へ配る。
 * 機構はメンバで持つので、静的初期化順に依存しない。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "arm.h"
#include "base_chassis.h"
#include "meister_protocol.h"

namespace meister {

class CommandDispatch {
 public:
  void begin();

  /// 受信バイト 1 個をステートマシンで処理する
  void feedRxByte(uint8_t byte);

  /// 受信側で見たエラー状態（フィードバックのエラーフラグに使う）
  uint8_t rxErrorFlags() const;

 private:
  void onFrameComplete(const uint8_t* data, size_t len);

  enum class RxState : uint8_t {
    kWaitHeader,  // ヘッダ 0xA5 を待機
    kWaitType,    // 種別を待機
    kPayload,     // ペイロード収集
    kChecksum,    // チェックサム受信 → 検証
  };

  BaseChassis chassis_;
  Arm arm_;

  RxState rxState_ = RxState::kWaitHeader;
  uint8_t rxBuf_[proto::kMaxFrameSize] = {};
  size_t rxIndex_ = 0;        // 書込中インデックス
  size_t rxPayloadLeft_ = 0;  // ペイロード残バイト数
  volatile uint32_t rxErrorCount_ = 0;  // フレーム破損回数
};

}  // namespace meister
