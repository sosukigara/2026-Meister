#include "meister_serial_bridge/stream_parser.hpp"

#include <cstring>

namespace meister_serial_bridge {

void StreamFrameParser::CompactIfNeeded() noexcept {
  if (head_ == buf_.size()) {
    buf_.clear();
    head_ = 0;
  } else if (head_ >= 256 && head_ * 2 >= buf_.size()) {
    const std::size_t rem = buf_.size() - head_;
    std::memmove(buf_.data(), buf_.data() + head_, rem);
    buf_.resize(rem);
    head_ = 0;
  }
}

void StreamFrameParser::Feed(const uint8_t* data, std::size_t len,
                             std::vector<meister::proto::Frame>* out) {
  namespace proto = meister::proto;
  if (data && len > 0) {
    buf_.insert(buf_.end(), data, data + len);
  }

  while (buf_.size() > head_) {
    // 1. ヘッダバイト 0xA5 まで O(1) カーソル移動
    if (buf_[head_] != proto::kHeaderByte) {
      ++head_;
      continue;
    }

    const std::size_t avail = buf_.size() - head_;
    if (avail < 2) {
      break;
    }

    const auto type = static_cast<proto::TypeId>(buf_[head_ + 1]);
    if (proto::PayloadSize(type) == 0) {
      // 未知種別。ヘッダ 1 バイトだけ進めて次のヘッダを探し直す。
      ++head_;
      continue;
    }

    const std::size_t frame_size = proto::FrameSize(type);
    if (avail < frame_size) {
      break;
    }

    proto::Frame frame;
    const proto::ParseResult result =
        proto::ParseFrame(buf_.data() + head_, frame_size, &frame);
    if (result == proto::ParseResult::kOk) {
      head_ += frame_size;
      out->push_back(frame);
    } else {
      // チェックサム不一致時も全フレーム分進める（旧実装互換）
      head_ += frame_size;
    }
  }

  CompactIfNeeded();
}

}  // namespace meister_serial_bridge
