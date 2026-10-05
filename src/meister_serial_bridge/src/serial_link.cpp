#include "meister_serial_bridge/serial_io.hpp"

#include <algorithm>
#include <stdexcept>

#include <libserial/SerialPort.h>

namespace meister_serial_bridge {
namespace {

/// libserial 実装。pyserial の serial_for_url に相当する URL 対応はしない。
class LibSerialIo final : public SerialIo {
 public:
  LibSerialIo(const std::string& path, int baud) {
    port_.Open(path);
    port_.SetBaudRate(static_cast<LibSerial::BaudRate>(baud));
    // pyserial も open 直後に DTR を assert する。EN 線の制御は呼出側に任せる。
    port_.SetDTR(false);
    port_.SetRTS(false);
  }

  std::size_t Read(uint8_t* buf, std::size_t max, int timeout_ms) override {
    if (max == 0) {
      return 0;
    }
    LibSerial::DataBuffer dbuf;
    if (timeout_ms > 0) {
      // 1 バイトまで待つ。揃わなければ dataBuffer は空のまま戻る。
      port_.Read(dbuf, 1, static_cast<std::size_t>(timeout_ms));
    }
    // numberOfBytes=0, msTimeout=0 は「溜まっている分をすぐ返す」。
    port_.Read(dbuf, 0, 0);
    const std::size_t n = std::min(max, dbuf.size());
    std::copy(dbuf.begin(), dbuf.begin() + static_cast<std::ptrdiff_t>(n), buf);
    return n;
  }

  void Write(const uint8_t* data, std::size_t len) override {
    port_.Write(LibSerial::DataBuffer(data, data + len));
  }

  void Flush() override { port_.FlushOutputBuffer(); }

  void ResetInputBuffer() override { port_.FlushInputBuffer(); }

  void SetDtr(bool high) override { port_.SetDTR(high); }

  void SetRts(bool high) override { port_.SetRTS(high); }

 private:
  LibSerial::SerialPort port_;
};

}  // namespace

std::unique_ptr<SerialIo> OpenSerialPort(const std::string& path, int baud) {
  try {
    return std::make_unique<LibSerialIo>(path, baud);
  } catch (const LibSerial::OpenFailed& e) {
    throw std::runtime_error("cannot open " + path + ": " + e.what());
  }
}

}  // namespace meister_serial_bridge
