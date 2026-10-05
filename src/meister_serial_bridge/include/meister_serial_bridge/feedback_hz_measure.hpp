#pragma once

#include "meister_serial_bridge/comm_check.hpp"
#include "meister_serial_bridge/serial_io.hpp"

namespace meister_serial_bridge {

inline constexpr double kDefaultDurationS = 5.0;

/// 計測中に流す零指令のレート（双方向健全性の確認用）。
inline constexpr double kCommandHz = 10.0;

struct Measurement {
  int frames = 0;
  long nbytes = 0;
  double duration_s = 0.0;
  double measured_hz = 0.0;
  double mean_ms = 0.0;
  double p95_ms = 0.0;
  double max_ms = 0.0;
  int protocol_errors = 0;
  int commands_sent = 0;

  /// 受信バイト数から期待されるフレーム数が減った分。欠落か破損の検出用。
  int LostFrames() const {
    const long expected = nbytes / static_cast<long>(meister::proto::kMaxFrameSize);
    return expected > frames ? static_cast<int>(expected - frames) : 0;
  }
};

/// FB_STATE の到着レートと間隔を測る。読み取りだけ。
Measurement MeasureFeedbackHz(const std::string& port, int baud, double duration_s,
                               bool with_commands, bool reset, const SerialOpener& opener);

}  // namespace meister_serial_bridge
