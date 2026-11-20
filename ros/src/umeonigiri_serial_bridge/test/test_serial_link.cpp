#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <unistd.h>

#include "umeonigiri_serial_bridge/serial_io.hpp"

namespace msb = umeonigiri_serial_bridge;

namespace {

/// テスト専用の pty。実 UART を使わずに libserial 実装の読み書きを検証する。
///
/// master だけを開いた状態では slave の tty が消える環境があるため、
/// slave も O_NOCTTY で保持する。libserial 側が別 fd で開くのは従来どおり。
class PtyPair {
 public:
  PtyPair() = default;
  ~PtyPair() {
    if (slave_fd_ >= 0) {
      ::close(slave_fd_);
    }
    if (master_fd_ >= 0) {
      ::close(master_fd_);
    }
  }
  PtyPair(const PtyPair&) = delete;
  PtyPair& operator=(const PtyPair&) = delete;

  bool Open() {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd_ < 0) {
      return false;
    }
    if (grantpt(master_fd_) != 0 || unlockpt(master_fd_) != 0) {
      return false;
    }
    const char* name = ptsname(master_fd_);
    if (name == nullptr) {
      return false;
    }
    slave_name_ = name;
    slave_fd_ = ::open(slave_name_.c_str(), O_RDWR | O_NOCTTY);
    return slave_fd_ >= 0;
  }

  int master() const { return master_fd_; }
  const std::string& slave() const { return slave_name_; }

 private:
  int master_fd_ = -1;
  int slave_fd_ = -1;
  std::string slave_name_;
};

/// pty の書き込みが slave 側の受信バッファに載るのを待つ。
void WaitForUart() { usleep(50 * 1000); }

}  // namespace

TEST(SerialLink, ReadReturnsEveryBufferedByte) {
  PtyPair pty;
  ASSERT_TRUE(pty.Open());
  auto io = msb::OpenSerialPort(pty.slave(), msb::kDefaultBaud, /*control_modem_lines=*/false);
  ASSERT_TRUE(io != nullptr);

  std::array<uint8_t, 17> tx{};
  for (std::size_t i = 0; i < tx.size(); ++i) {
    tx[i] = static_cast<uint8_t>(i + 1);
  }
  ASSERT_EQ(::write(pty.master(), tx.data(), tx.size()),
            static_cast<ssize_t>(tx.size()));
  WaitForUart();

  std::array<uint8_t, 64> rx{};
  EXPECT_EQ(io->Read(rx.data(), rx.size(), 100), tx.size());
  EXPECT_EQ(std::memcmp(rx.data(), tx.data(), tx.size()), 0);
}

TEST(SerialLink, ReadWithoutTimeoutDrainsBufferedBytes) {
  PtyPair pty;
  ASSERT_TRUE(pty.Open());
  auto io = msb::OpenSerialPort(pty.slave(), msb::kDefaultBaud, /*control_modem_lines=*/false);
  ASSERT_TRUE(io != nullptr);

  const std::array<uint8_t, 4> tx{{0xA5, 0x01, 0x02, 0x03}};
  ASSERT_EQ(::write(pty.master(), tx.data(), tx.size()),
            static_cast<ssize_t>(tx.size()));
  WaitForUart();

  std::array<uint8_t, 64> rx{};
  EXPECT_EQ(io->Read(rx.data(), rx.size(), 0), tx.size());
  EXPECT_EQ(std::memcmp(rx.data(), tx.data(), tx.size()), 0);
}

TEST(SerialLink, ReadReturnsZeroWhenIdle) {
  PtyPair pty;
  ASSERT_TRUE(pty.Open());
  auto io = msb::OpenSerialPort(pty.slave(), msb::kDefaultBaud, /*control_modem_lines=*/false);
  ASSERT_TRUE(io != nullptr);

  std::array<uint8_t, 64> rx{};
  EXPECT_EQ(io->Read(rx.data(), rx.size(), 20), 0u);
}

TEST(SerialLink, RejectsUnsupportedBaud) {
  PtyPair pty;
  ASSERT_TRUE(pty.Open());
  EXPECT_THROW(msb::OpenSerialPort(pty.slave(), 12345), std::exception);
}

// pty は DTR/RTS を持たない。既定（modem control あり）で黙って開くと、
// 実機で配線が効かないときに起動だけ成功して原因が見えなくなる。
TEST(SerialLink, DoesNotSwallowModemControlFailureByDefault) {
  PtyPair pty;
  ASSERT_TRUE(pty.Open());
  EXPECT_THROW(msb::OpenSerialPort(pty.slave(), msb::kDefaultBaud), std::exception);
}
