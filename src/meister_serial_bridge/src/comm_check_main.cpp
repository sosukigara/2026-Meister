#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "meister_serial_bridge/comm_check.hpp"
#include "meister_serial_bridge/serial_io.hpp"

namespace {

void PrintUsage() {
  std::printf(
      "usage: meister_comm_check [--port PATH] [--baud N] [--timeout SEC] [--no-reset]\n"
      "\n"
      "ESP32 とのプロトコル通信を検証する。モータとサーボは動かさない。\n"
      "  uplink    FB_STATE を受け取れるか\n"
      "  downlink  中立コマンド 4 種が通るか（フレーム長の検証）\n"
      "  err_flag  壊したフレームでエラーフラグが立つか（判定基準の裏取り）\n");
}

}  // namespace

int main(int argc, char** argv) {
  using namespace meister_serial_bridge;

  std::string port = kDefaultPort;
  int baud = kDefaultBaud;
  double timeout = kDefaultTimeoutS;
  bool reset = true;

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
    } else if (arg == "--timeout") {
      timeout = std::stod(next("--timeout"));
    } else if (arg == "--no-reset") {
      reset = false;
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
  CheckResult result;
  try {
    result = RunCommCheck(port, baud, timeout, reset,
                          [](const std::string& p, int b) { return std::shared_ptr<SerialIo>(OpenSerialPort(p, b)); });
  } catch (const CommError& e) {
    std::fprintf(stderr, "FAIL: %s\n", e.what());
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }

  for (const auto& step : result.steps) {
    std::printf("  [%s] %-9s %s\n", step.ok ? "OK" : "NG", step.name.c_str(),
                step.detail.c_str());
  }
  std::printf("PASS: PC と ESP32 の通信を確認しました。\n");
  return 0;
}
