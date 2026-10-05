#include "meister_serial_bridge/stream_parser.hpp"

namespace meister_serial_bridge {

void StreamFrameParser::Feed(const uint8_t* data, std::size_t len,
                             std::vector<meister::proto::Frame>* out) {
  namespace proto = meister::proto;
  buf_.insert(buf_.end(), data, data + len);

  while (true) {
    while (!buf_.empty() && buf_[0] != proto::kHeaderByte) {
      buf_.erase(buf_.begin());
    }
    if (buf_.size() < 2) {
      return;
    }
    const auto type = static_cast<proto::TypeId>(buf_[1]);
    if (proto::PayloadSize(type) == 0) {
      // 未知種別。ヘッダ 1 バイトだけ捨てて次のヘッダを探し直す。
      buf_.erase(buf_.begin());
      continue;
    }
    const std::size_t frame_size = proto::FrameSize(type);
    if (buf_.size() < frame_size) {
      return;
    }
    proto::Frame frame;
    const proto::ParseResult result = proto::ParseFrame(buf_.data(), frame_size, &frame);
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    if (result == proto::ParseResult::kOk) {
      out->push_back(frame);
    }
  }
}

}  // namespace meister_serial_bridge
