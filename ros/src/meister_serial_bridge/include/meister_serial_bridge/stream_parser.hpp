#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "meister_protocol.h"

namespace meister_serial_bridge {

/// 受信ストリームから固定長フレームを取り出す増分パーサ。
///
/// firmware の ParseFrame は「フレームが揃ったバッファ」しか受けない。UART は
/// 任意の境界で割れるので firmware 側 main.cpp の受信状態機械と同じ手順で
/// バッファを持ち、再同期とチェックサム不一致の破棄はここでやる。
///
/// 最適化:
///   旧実装は非ヘッダバイトや破損フレームのたびに `buf_.erase(buf_.begin())` を呼び出し、
///   最悪 O(N^2) のメモリシフトを起こしていた。
///   本実装では読み取りカーソル `head_` を用いて O(1) で走査し、バッファ消費完了時または
///   閾値到達時のみ一括コンパクションを行う。
class StreamFrameParser {
 public:
  StreamFrameParser() { buf_.reserve(512); }

  /// data を投入し、完結したフレームを out に積む。out は追記される。
  void Feed(const uint8_t* data, std::size_t len, std::vector<meister::proto::Frame>* out);

  /// 保持している未消費バイト数。テストと診断用。
  std::size_t buffered() const noexcept { return buf_.size() - head_; }

  void Clear() noexcept {
    buf_.clear();
    head_ = 0;
  }

 private:
  void CompactIfNeeded() noexcept;

  std::vector<uint8_t> buf_;
  std::size_t head_ = 0;
};

}  // namespace meister_serial_bridge
