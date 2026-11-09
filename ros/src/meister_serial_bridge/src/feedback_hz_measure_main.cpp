#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "meister_serial_bridge/feedback_hz_measure.hpp"
#include "meister_serial_bridge/serial_io.hpp"

namespace {

void PrintUsage() {
  std::printf(
      "usage: meister_hz_measure [--port PATH] [--baud N] [--duration SEC]\n"
      "                         [--with-commands] [--no-reset] [--expect-hz HZ]\n"
      "\n"
      "FB_STATE の送信周期を実測する。読み取りのみ、ROS 不要。\n"
      "  --with-commands  %g Hz で零指令を流し、双方向健全性を確認する\n"
      "  --expect-hz      設定値。指定すると実測値との差を表示する\n",
      meister_serial_bridge::kCommandHz);
}

}  // namespace

int main(int argc, char** argv) {
  using namespace meister_serial_bridge;

  std::string port = kDefaultPort;
  int baud = kDefaultBaud;
  double duration = kDefaultDurationS;
  bool with_commands = false;
  bool reset = true;
  bool expect_given = false;
  double expect_hz = 0.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ERROR: %s には値が必要です\n", name);
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--port") {
      port = next("--port");
    } else if (arg == "--baud") {
      baud = std::stoi(next("--baud"));
    } else if (arg == "--duration") {
      duration = std::stod(next("--duration"));
    } else if (arg == "--with-commands") {
      with_commands = true;
    } else if (arg == "--no-reset") {
      reset = false;
    } else if (arg == "--expect-hz") {
      expect_hz = std::stod(next("--expect-hz"));
      expect_given = true;
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "ERROR: 不明な引数 %s\n", arg.c_str());
      PrintUsage();
      return 2;
    }
  }

  std::printf("port: %s  (%s)  baud: %d\n", port.c_str(), DescribePort(port).c_str(), baud);
  Measurement m;
  try {
    m = MeasureFeedbackHz(port, baud, duration, with_commands, reset,
                          [](const std::string& p, int b) { return std::shared_ptr<SerialIo>(OpenSerialPort(p, b)); });
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }

  std::printf("frames          : %d  (%ld bytes, commands=%d)\n", m.frames, m.nbytes,
              m.commands_sent);
  std::printf("measured Hz     : %.1f", m.measured_hz);
  if (expect_given) {
    std::printf("  (expect %g, delta %+.1f)\n", expect_hz, m.measured_hz - expect_hz);
  } else {
    std::printf("\n");
  }
  std::printf("inter-arrival ms: mean=%.2f p95=%.2f max=%.2f\n", m.mean_ms, m.p95_ms,
              m.max_ms);
  std::printf("protocol errors : %d\n", m.protocol_errors);
  std::printf("lost frames     : %d\n", m.LostFrames());
  if (m.protocol_errors != 0 || m.LostFrames() != 0) {
    std::printf("NG: フレームが欠落または破損しています。この設定では採用しないこと。\n");
    return 1;
  }
  return 0;
}
