#include "umeonigiri_serial_bridge/serial_io.hpp"

#include <algorithm>
#include <stdexcept>

#include <libserial/SerialPort.h>

namespace umeonigiri_serial_bridge {
namespace {

/// libserial の BaudRate は termios の speed_t 定数 (B115200 等) であり、
/// 生の整数 (115200) ではない。対応表をここ 1 か所に閉じる。
bool ToLibSerialBaud(int baud, LibSerial::BaudRate* out) {
  switch (baud) {
    case 9600: *out = LibSerial::BaudRate::BAUD_9600; return true;
    case 19200: *out = LibSerial::BaudRate::BAUD_19200; return true;
    case 38400: *out = LibSerial::BaudRate::BAUD_38400; return true;
    case 57600: *out = LibSerial::BaudRate::BAUD_57600; return true;
    case 115200: *out = LibSerial::BaudRate::BAUD_115200; return true;
    case 230400: *out = LibSerial::BaudRate::BAUD_230400; return true;
    case 460800: *out = LibSerial::BaudRate::BAUD_460800; return true;
    case 921600: *out = LibSerial::BaudRate::BAUD_921600; return true;
    default: return false;
  }
}

/// libserial 実装。pyserial の serial_for_url に相当する URL 対応はしない。
class LibSerialIo final : public SerialIo {
 public:
  LibSerialIo(const std::string& path, int baud, bool control_modem_lines) {
    LibSerial::BaudRate br = LibSerial::BaudRate::BAUD_DEFAULT;
    if (!ToLibSerialBaud(baud, &br)) {
      throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud));
    }
    port_.Open(path);
    port_.SetBaudRate(br);
    // pyserial も open 直後に DTR を assert する。EN 線の制御は呼出側に任せる。
    if (control_modem_lines) {
      port_.SetDTR(false);
      port_.SetRTS(false);
    }
  }

  std::size_t Read(uint8_t* buf, std::size_t max, int timeout_ms) override {
    if (buf == nullptr || max == 0) {
      return 0;
    }
    LibSerial::DataBuffer dbuf;
    if (timeout_ms > 0) {
      // 1 バイトまで待つ。揃わなければ ReadTimeout が投げられるので握りつぶす。
      try {
        port_.Read(dbuf, 1, static_cast<std::size_t>(timeout_ms));
      } catch (const LibSerial::ReadTimeout&) {
        // タイムアウト → まだ何も来ていない
        return 0;
      }
    }
    // libserial の Read(numberOfBytes=0, msTimeout=0) は何も読まずに即 return する。
    // 到着済みの分は GetNumberOfBytesAvailable() で取り、要求バイト数を明示して読む。
    const int available = port_.GetNumberOfBytesAvailable();
    const std::size_t room = max - dbuf.size();
    const std::size_t want =
        std::min(room, static_cast<std::size_t>(std::max(available, 0)));
    if (want > 0) {
      // libserial の Read は渡した DataBuffer を先頭で clear する。同じバッファへ
      // 追記すると 1 バイト目が消えるため、別バッファで読んで連結する。
      LibSerial::DataBuffer rest;
      try {
        // timeout 0 は「要求バイトが揃うまでブロック」なので、競合時の無限待ちを
        // 避けるため 1 ms にする。到着済みなら即返る。
        port_.Read(rest, want, 1);
      } catch (const LibSerial::ReadTimeout&) {
        // 途中まで読めた分だけ返す
      }
      dbuf.insert(dbuf.end(), rest.begin(), rest.end());
    }
    const std::size_t n = std::min(max, dbuf.size());
    std::copy_n(dbuf.begin(), n, buf);
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

std::unique_ptr<SerialIo> OpenSerialPort(const std::string& path, int baud,
                                         bool control_modem_lines) {
  try {
    return std::make_unique<LibSerialIo>(path, baud, control_modem_lines);
  } catch (const LibSerial::OpenFailed& e) {
    throw std::runtime_error("cannot open " + path + ": " + e.what());
  } catch (const std::invalid_argument& e) {
    throw std::runtime_error("cannot configure " + path + ": " + e.what());
  }
}

}  // namespace umeonigiri_serial_bridge
