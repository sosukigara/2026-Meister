#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "meister_serial_bridge/serial_io.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace meister_serial_bridge {
namespace test {

namespace proto = meister::proto;

/// テストダブルは呼出側のスタック上のオブジェクト。BridgeCore も RunCommCheck
/// も所有しないので、解放しない deleter 付きで受け取る。
inline std::shared_ptr<SerialIo> NonOwning(SerialIo* io) {
  return std::shared_ptr<SerialIo>(io, [](SerialIo*) {});
}

struct FakeEsp32Options {
  bool reply = true;
  bool reject_valid = false;
  bool notice_corrupt = true;
  bool dtr_stuck = false;
  /// pyserial は open 直後に DTR を assert する。libserial は解放する
  /// （serial_link.cpp の LibSerialIo と同じ挙動）。実際の実装に合わせる。
  bool dtr_asserted_at_open = true;
};

/// ESP32 ファームウェアの最小のふるまいを再現するテストダブル。
///
/// firmware/src/main.cpp の受信・送信の挙動（有効なフレームは受理してエラー
/// 無しの FB_STATE を返す、壊れたフレームはエラーフラグを立てる、フラグは
/// 再起動まで保持）と、IO0 (DTR) が Low のままだとブートローダで止まるという
/// 実際の挙動を反映している。
class FakeEsp32 final : public SerialIo {
 public:
  explicit FakeEsp32(FakeEsp32Options opts = {})
      : reply_(opts.reply),
        reject_valid_(opts.reject_valid),
        notice_corrupt_(opts.notice_corrupt),
        dtr_stuck_(opts.dtr_stuck),
        dtr_(opts.dtr_asserted_at_open) {}

  std::size_t Read(uint8_t* buf, std::size_t max, int) override {
    if (!AppRunning()) {
      return 0;
    }
    if (reply_ && rx_.empty()) {
      // FB_STATE は指令への応答ではなく周期送信（実機と同じ）。
      AppendState();
    }
    const std::size_t n = std::min(max, rx_.size());
    std::copy(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(n), buf);
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(n));
    return n;
  }

  void Write(const uint8_t* data, std::size_t len) override {
    if (!AppRunning()) {
      return;
    }
    written_.insert(written_.end(), data, data + len);

    std::vector<proto::Frame> frames;
    parser_.Feed(data, len, &frames);
    std::size_t consumed = 0;
    for (const auto& f : frames) {
      consumed += f.frameSize;
    }
    if (consumed != len) {
      // 破棄されたバイトがある = 壊れたフレームがあった。
      if (notice_corrupt_) {
        error_flags_ |= proto::kFbErrorProtocol;
      }
    } else if (!frames.empty() && reject_valid_) {
      error_flags_ |= proto::kFbErrorProtocol;
    }
    if (reply_) {
      AppendState();
    }
  }

  void Flush() override {}

  void ResetInputBuffer() override { rx_.clear(); }

  void SetDtr(bool high) override {
    if (!dtr_stuck_) {
      dtr_ = high;
    }
  }

  void SetRts(bool) override {}

  /// テスト側から受信列を流し込む（実機では UART から来るのと同じ枠）。
  void InjectRx(const uint8_t* data, std::size_t len) {
    rx_.insert(rx_.end(), data, data + len);
  }

  uint8_t error_flags() const { return error_flags_; }
  const std::vector<uint8_t>& written() const { return written_; }

 private:
  bool AppRunning() {
    if (app_running_) {
      return true;
    }
    if (dtr_) {
      return false;
    }
    app_running_ = true;
    return true;
  }

  void AppendState() {
    int16_t encoders[proto::kNumDriveMotors] = {};
    uint8_t buf[proto::kMaxFrameSize];
    const auto n = proto::EncodeState(buf, sizeof(buf), encoders, 0, error_flags_);
    rx_.insert(rx_.end(), buf, buf + n);
  }

  bool reply_;
  bool reject_valid_;
  bool notice_corrupt_;
  bool dtr_stuck_;
  bool dtr_;
  bool app_running_ = false;
  uint8_t error_flags_ = 0;
  std::vector<uint8_t> rx_;
  std::vector<uint8_t> written_;
  StreamFrameParser parser_;
};

}  // namespace test
}  // namespace meister_serial_bridge
