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
class StreamFrameParser {
 public:
  /// data を投入し、完結したフレームを out に積む。out は追記される。
  void Feed(const uint8_t* data, std::size_t len, std::vector<meister::proto::Frame>* out);

  /// 保持している未消費バイト数。テストと診断用。
  std::size_t buffered() const { return buf_.size(); }

  void Clear() { buf_.clear(); }

 private:
  std::vector<uint8_t> buf_;
};

}  // namespace meister_serial_bridge
