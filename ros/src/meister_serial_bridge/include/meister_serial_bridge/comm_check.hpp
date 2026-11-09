#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "meister_serial_bridge/serial_io.hpp"

namespace meister_serial_bridge {

inline constexpr double kDefaultTimeoutS = 2.0;

/// 通信確認専用。中立と停止の値だけを流す（モータとサーボを動かさない）。
std::vector<std::vector<uint8_t>> NeutralCommandFrames();

struct CheckStep {
  std::string name;
  bool ok = false;
  std::string detail;
};

struct CheckResult {
  std::string port;
  std::vector<CheckStep> steps;
  bool ok() const { return !steps.empty() && AllStepsOk(); }
  bool AllStepsOk() const;
};

/// 検証が成立しなかった（使用者向けの理由）。
class CommError : public std::runtime_error {
 public:
  explicit CommError(const std::string& what) : std::runtime_error(what) {}
};

/// ポートを開く関数。テストではフェイクを注入する。
///
/// shared_ptr なのは、テストダブルが呼出側のスタック上にあり、返す側が所有
/// できないため。所有しない deleter 付きで受け取る。
using SerialOpener = std::function<std::shared_ptr<SerialIo>(const std::string&, int)>;

/// uplink / downlink / err_flag の 3 手順を実行する。
///
/// 手順 2 は ESP32 側の受信エラーが再起動でしか消えないことに依存する。
/// 既にフラグが立っている場合は --reset で再起動してから実行する。
CheckResult RunCommCheck(const std::string& port, int baud, double timeout_s, bool reset,
                         const SerialOpener& opener);

}  // namespace meister_serial_bridge
