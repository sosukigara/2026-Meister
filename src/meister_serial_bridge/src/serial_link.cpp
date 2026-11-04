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
    // libserial の BaudRate は termios の speed_t 定数 (B115200 等) であり、
    // 生の整数 (115200) ではない。int から直接キャストすると "Invalid baud rate" になる。
    LibSerial::BaudRate br = LibSerial::BaudRate::BAUD_DEFAULT;
    switch (baud) {
      case 9600: br = LibSerial::BaudRate::BAUD_9600; break;
      case 19200: br = LibSerial::BaudRate::BAUD_19200; break;
      case 38400: br = LibSerial::BaudRate::BAUD_38400; break;
      case 57600: br = LibSerial::BaudRate::BAUD_57600; break;
      case 115200: br = LibSerial::BaudRate::BAUD_115200; break;
      case 230400: br = LibSerial::BaudRate::BAUD_230400; break;
      case 460800: br = LibSerial::BaudRate::BAUD_460800; break;
      case 921600: br = LibSerial::BaudRate::BAUD_921600; break;
      default: break;
    }
    port_.SetBaudRate(br);
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
      // 1 バイトまで待つ。揃わなければ ReadTimeout が投げられるので握りつぶす。
      try {
        port_.Read(dbuf, 1, static_cast<std::size_t>(timeout_ms));
      } catch (const LibSerial::ReadTimeout&) {
        // タイムアウト → まだ何も来ていない
      }
    }
    // numberOfBytes=0, msTimeout=0 は「溜まっている分をすぐ返す」。
    try {
      port_.Read(dbuf, 0, 0);
    } catch (const LibSerial::ReadTimeout&) {
    }
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
