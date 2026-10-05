#include "meister_serial_bridge/feedback_hz_measure.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

#include "meister_serial_bridge/frames.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace meister_serial_bridge {
namespace {

namespace proto = meister::proto;

constexpr int kPollMs = 2;

double NowSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

Measurement MeasureFeedbackHz(const std::string& port, int baud, double duration_s,
                               bool with_commands, bool reset, const SerialOpener& opener) {
  const std::shared_ptr<SerialIo> link = opener(port, baud);
  if (!link) {
    throw CommError("serial open failed: " + port);
  }

  // DTR = IO0。assert のままだとブートローダで止まる。
  link->SetDtr(false);
  if (reset) {
    link->SetRts(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    link->SetRts(false);
    // 起動待ち（ROM バナーと初期化の安定化）
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  link->ResetInputBuffer();

  StreamFrameParser parser;
  std::vector<double> stamps;
  std::vector<proto::Frame> frames;
  uint8_t buf[256];
  long nbytes = 0;
  int protocol_errors = 0;
  int ncmd = 0;
  double next_cmd = 0.0;

  const auto zero = NeutralCommandFrames();
  const double t0 = NowSeconds();
  while (true) {
    const double now = NowSeconds();
    if (now - t0 >= duration_s) {
      break;
    }
    if (with_commands && now >= next_cmd) {
      for (const auto& frame : zero) {
        link->Write(frame.data(), frame.size());
      }
      ++ncmd;
      next_cmd = now + 1.0 / kCommandHz;
    }
    const std::size_t n = link->Read(buf, sizeof(buf), kPollMs);
    if (n > 0) {
      nbytes += static_cast<long>(n);
      frames.clear();
      parser.Feed(buf, n, &frames);
      for (const auto& frame : frames) {
        if (frame.type != proto::kFbState) {
          continue;
        }
        stamps.push_back(NowSeconds());
        const std::size_t enc_end = proto::kNumDriveMotors * sizeof(int16_t);
        if (proto::FrameGetU8(frame, enc_end + 1) & proto::kFbErrorProtocol) {
          ++protocol_errors;
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  const double window = std::max(1e-9, NowSeconds() - t0);

  std::vector<double> gaps;
  for (std::size_t i = 1; i < stamps.size(); ++i) {
    gaps.push_back((stamps[i] - stamps[i - 1]) * 1e3);
  }
  std::sort(gaps.begin(), gaps.end());
  const double span = stamps.size() > 1 ? (stamps.back() - stamps.front()) : 0.0;

  Measurement m;
  m.frames = static_cast<int>(stamps.size());
  m.nbytes = nbytes;
  m.duration_s = window;
  m.measured_hz = span > 0.0 ? static_cast<double>(stamps.size() - 1) / span : 0.0;
  m.mean_ms = gaps.empty() ? 0.0
                           : std::accumulate(gaps.begin(), gaps.end(), 0.0) /
                                 static_cast<double>(gaps.size());
  m.p95_ms = gaps.empty() ? 0.0 : gaps[static_cast<std::size_t>(gaps.size() * 0.95)];
  m.max_ms = gaps.empty() ? 0.0 : gaps.back();
  m.protocol_errors = protocol_errors;
  m.commands_sent = ncmd;
  return m;
}

}  // namespace meister_serial_bridge
