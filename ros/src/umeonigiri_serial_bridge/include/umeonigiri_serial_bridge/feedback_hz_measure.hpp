#pragma once

#include "umeonigiri_serial_bridge/comm_check.hpp"
#include "umeonigiri_serial_bridge/serial_io.hpp"

namespace umeonigiri_serial_bridge {

inline constexpr double kDefaultDurationS = 5.0;

/// 計測中に流す零指令のレート（双方向健全性の確認用）。
inline constexpr double kCommandHz = 10.0;

struct Measurement {
  int frames = 0;
  long nbytes = 0;
  /// 復元できた全種別フレームのバイト数（FB_STATE / FB_IMU / FB_ERROR）。
  long decoded_bytes = 0;
  /// 受信末尾に残った未完成フレームのバイト数。欠落判定から除く。
  long buffered_bytes = 0;
  double measured_hz = 0.0;
  double mean_ms = 0.0;
  double p95_ms = 0.0;
  double max_ms = 0.0;
  int protocol_errors = 0;
  int commands_sent = 0;

  /// どのフレームにも復元できなかったバイト数を、FB_STATE（最大長）単位で
  /// 数える近似。FB_IMU など短いフレームを欠落と誤認しないための修正。
  int LostFrames() const {
    const long unaccounted = nbytes - decoded_bytes - buffered_bytes;
    if (unaccounted <= 0) {
      return 0;
    }
    return static_cast<int>(unaccounted /
                            static_cast<long>(meister::proto::kMaxFrameSize));
  }
};

/// FB_STATE の到着レートと間隔を測る。読み取りだけ。
Measurement MeasureFeedbackHz(const std::string& port, int baud, double duration_s,
                               bool with_commands, bool reset, const SerialOpener& opener);

}  // namespace umeonigiri_serial_bridge
