#include "meister_serial_bridge/comm_check.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#include "meister_serial_bridge/frames.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace meister_serial_bridge {
namespace {

namespace proto = meister::proto;

constexpr auto kPollInterval = std::chrono::microseconds(2000);
constexpr int kPollMs = 2;

double NowSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void WriteAll(SerialIo& link, const std::vector<std::vector<uint8_t>>& frames) {
  for (const auto& frame : frames) {
    link.Write(frame.data(), frame.size());
  }
  link.Flush();
}

/// EN 線 (RTS) をパルスして ESP32 を再起動する。
///
/// 標準オートプログラム回路はエミッタ接地（active-LOW）なので RTS=1 で
/// EN=Low になってリセットし、RTS=0 で通常起動する。IO0 (DTR) が Low のままだと
/// ROM ダウンロードモードで落ちるので、トグルする前に DTR を解放する（呼出側が行う）。
void ResetEsp32(SerialIo& link) {
  link.SetRts(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  link.SetRts(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

/// deadline まで FB_STATE を最大 want 個読む。他種別は無視する。
std::vector<Feedback> CollectFeedback(SerialIo& link, StreamFrameParser& parser,
                                       double deadline, std::size_t want) {
  std::vector<Feedback> found;
  uint8_t buf[256];
  std::vector<proto::Frame> frames;
  while (NowSeconds() < deadline && found.size() < want) {
    const std::size_t n = link.Read(buf, sizeof(buf), kPollMs);
    if (n == 0) {
      std::this_thread::sleep_for(kPollInterval);
      continue;
    }
    frames.clear();
    parser.Feed(buf, n, &frames);
    for (const auto& frame : frames) {
      if (frame.type != proto::kFbState) {
        continue;
      }
      found.push_back(DecodeStateFrame(frame));
    }
  }
  return found;
}

bool WaitProtocolError(SerialIo& link, StreamFrameParser& parser, double deadline) {
  for (const auto& fb : CollectFeedback(link, parser, deadline, 32)) {
    if (fb.HasProtocolError()) {
      return true;
    }
  }
  return false;
}

std::string CommandNames() {
  return "CMD_MOTOR_VELOCITY, CMD_STEERING_ANGLE, CMD_ARM_ANGLE, CMD_GRIPPER";
}

}  // namespace

std::vector<std::vector<uint8_t>> NeutralCommandFrames() {
  std::vector<std::vector<uint8_t>> out;
  uint8_t buf[proto::kMaxFrameSize];
  auto add = [&out, &buf](std::size_t n) { out.emplace_back(buf, buf + n); };

  add(EncodeVelocity(buf, sizeof(buf), Uniform<VelocityArray>(0)));
  add(EncodeSteering(buf, sizeof(buf), Uniform<SteeringArray>(0)));
  add(EncodeArm(buf, sizeof(buf), Uniform<ArmArray>(0)));
  add(EncodeGripper(buf, sizeof(buf), proto::kGripperStop));
  return out;
}

bool CheckResult::AllStepsOk() const {
  for (const auto& step : steps) {
    if (!step.ok) {
      return false;
    }
  }
  return true;
}

CheckResult RunCommCheck(const std::string& port, int baud, double timeout_s, bool reset,
                         const SerialOpener& opener) {
  CheckResult result;
  result.port = port;

  const std::shared_ptr<SerialIo> link = opener(port, baud);
  if (!link) {
    throw CommError("serial open failed: " + port);
  }

  // DTR = IO0。assert のままだとブートローダで止まるので必ず解放してから触る。
  link->SetDtr(false);
  if (reset) {
    ResetEsp32(*link);
  }
  link->ResetInputBuffer();

  StreamFrameParser parser;

  // --- 1. 上り (ESP32 -> PC) ---
  const auto uplink = CollectFeedback(*link, parser, NowSeconds() + timeout_s, 1);
  if (uplink.empty()) {
    throw CommError("FB_STATE を受信できませんでした (" + DescribePort(port) + " @ " +
                    std::to_string(baud) + "bps)。"
                    "esp32dev_usbuart ビルドが書き込まれているか、"
                    "ポートとボーレートが正しいかを確認してください。");
  }
  result.steps.push_back({"uplink", true, uplink[0].Describe()});

  if (uplink[0].HasProtocolError()) {
    throw CommError(
        "FB_STATE に既にプロトコルエラーフラグが立っています。"
        "このフラグは再起動でしか消えないため、--reset（既定）付きで再実行してください。");
  }

  // --- 2. 下り (PC -> ESP32) ---
  // FB_STATE はコマンドへの応答ではなく 10 Hz 周期送信なので、4 種を送った直後の
  // 最初の FB_STATE がそれに対する応答になる。フラグが立たなければ ESP32 側は
  // 全フレームを正常に受理している。
  const auto names = CommandNames();
  WriteAll(*link, NeutralCommandFrames());
  const auto reply = CollectFeedback(*link, parser, NowSeconds() + timeout_s, 1);
  if (reply.empty()) {
    throw CommError(names + " を送ったあとの FB_STATE が返ってきませんでした。");
  }
  if (reply[0].HasProtocolError()) {
    throw CommError(names + " に対してプロトコルエラーフラグが立ちました (" +
                    reply[0].Describe() + ")。"
                    "フレーム長が ESP32 側と一致していない可能性があります。");
  }
  result.steps.push_back({"downlink", true, names + " -> " + reply[0].Describe()});

  // --- 3. 手順 2 の判定基準が機能していることの裏取り ---
  // チェックサムを壊したフレームを送り、フラグが立つことを確認する。
  uint8_t broken[proto::kMaxFrameSize];
  const std::size_t good_n = EncodeVelocity(broken, sizeof(broken), Uniform<VelocityArray>(0));
  broken[good_n - 1] ^= 0xFF;
  link->ResetInputBuffer();
  link->Write(broken, good_n);
  link->Flush();
  if (!WaitProtocolError(*link, parser, NowSeconds() + timeout_s)) {
    throw CommError(
        "意図的に壊したフレームでプロトコルエラーフラグが立ちませんでした。"
        "手順 2 の判定基準が機能していないため結果は採用できません。");
  }
  char flags[8];
  std::snprintf(flags, sizeof(flags), "0x%02X", proto::kFbErrorProtocol);
  result.steps.push_back(
      {"err_flag", true, std::string("corrupt frame -> error_flags |= ") + flags});

  return result;
}

}  // namespace meister_serial_bridge
