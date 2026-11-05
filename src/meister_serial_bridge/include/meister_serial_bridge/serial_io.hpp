#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "meister_protocol.h"

namespace meister_serial_bridge {

inline constexpr char kDefaultPort[] = "/dev/ttyUSB0";
inline constexpr int kDefaultBaud = 115200;

/// ビットストリーム I/O の抽象。実装は libserial とテストのフェイク。
///
/// pyserial の read(size) は size バイトがそろうか timeout するまでブロックする。
/// 1 フレーム分 (17 バイト) ごとに呼ぶと受信が数フレーム束ねになり、
/// FB_STATE の到着間隔が乱れる（実測 p95 39.7 ms の遅延がそれ）。
/// 従って available() を見てから「残ってる分だけ」読む形に固定している。
class SerialIo {
 public:
  virtual ~SerialIo() = default;

  /// 受信済みのバイトを最大 max 個読む。採れなければ 0。
  ///
  /// timeout_ms > 0 のときは、まず 1 バイト揃うまでその時間だけ待ち、
  /// 揃ったぶんも貯まっていた分もまとめて返す。0 のときは待たない。
  ///
  /// 「溜まっている分だけ」を返すのは必須要件で、pyserial の read(size) を
  /// そのまま使うと size バイト揃うか timeout するまでブロックするため、
  /// 1 フレーム (17 バイト) ごとに呼ぶと受信が数フレーム束ねになり、
  /// FB_STATE の到着間隔が乱れる（実測 p95 39.7 ms の遅延がそれ）。
  virtual std::size_t Read(uint8_t* buf, std::size_t max, int timeout_ms) = 0;

  virtual void Write(const uint8_t* data, std::size_t len) = 0;
  virtual void Flush() = 0;
  virtual void ResetInputBuffer() = 0;

  /// DTR = IO0。assert のままだとブートローダで止まるので必ず解放する。
  virtual void SetDtr(bool high) = 0;

  /// RTS = EN。1 でリセット、0 で通常起動。
  virtual void SetRts(bool high) = 0;
};

/// libserial でポートを開く。開けないときは std::runtime_error。
std::unique_ptr<SerialIo> OpenSerialPort(const std::string& path, int baud);

/// FB_STATE 1 フレームのデコード結果。
struct Feedback {
  std::array<int16_t, meister::proto::kNumDriveMotors> encoders{};
  uint8_t state = 0;
  uint8_t error_flags = 0;

  bool HasProtocolError() const {
    return (error_flags & meister::proto::kFbErrorProtocol) != 0;
  }

  /// 人が読む 1 行。encoders は実測ではない（firmware の時間由来ダミー値）。
  std::string Describe() const;
};

/// FB_STATE フレームを Feedback に落とす。種別が違えば空の Feedback。
Feedback DecodeStateFrame(const meister::proto::Frame& frame);

/// FB_IMU 1 フレームのデコード結果。yaw は BNO055 heading 生値（0.1° 単位）。
struct ImuReport {
  int16_t yaw_tenths = 0;
  uint8_t calib = 0;  // CALIB_STAT 生値（sys/gyro/acc/mag 各 2bit）
};

/// FB_IMU フレームを ImuReport に落とす。種別が違えば空の ImuReport。
ImuReport DecodeImuFrame(const meister::proto::Frame& frame);

/// yaw のみから作る単位クォータニオン（x=y=0）。
struct YawQuat {
  double z = 0.0;
  double w = 1.0;
};

/// yaw_tenths（0.1° 単位 0..3600）を yaw のみのクォータニオンに変換する。
/// ★要確認★ BNO055 heading は時計回り正、ROS は反時計回り正。実機を回して
/// 符号が逆ならここ 1 か所で反転させる（bench 確認前はそのまま通す）。
YawQuat YawTenthsToQuat(int16_t yaw_tenths);

/// /dev/serial/by-id の実名を返す。接続が複数あるときに誤接続を見分けるため。
/// 見つからなければ path をそのまま返す。
std::string DescribePort(const std::string& path);

}  // namespace meister_serial_bridge
