#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "umeonigiri_serial_bridge/kinematics.hpp"
#include "umeonigiri_serial_bridge/serial_io.hpp"
#include "umeonigiri_serial_bridge/stream_parser.hpp"

namespace umeonigiri_serial_bridge {

struct BridgeParams {
  std::string serial_port = kDefaultPort;
  int baud = kDefaultBaud;
  TwistKinematics kinematics;
  double cmd_timeout_s = 0.5;
  /// ポートを開けなかったときの再試行間隔 [秒]。
  double reopen_interval_s = 2.0;
};

struct WatchdogVerdict {
  /// 停止フレームを送ったか。true のときだけ送っている。
  bool send_stop = false;
  /// アームが無通信に入った（遷移した）ときだけ true。警告を 1 回出すため。
  bool arm_idle_started = false;
};

/// ROS に依存しないブリッジ中核。UART 1 本の所有、指令の符号化、ウォッチドッグの
/// 判定、受信フレームの復号を持つ。ROS ノードはこの上の薄い適応層で、
/// 時間源とログだけを持ち込む（rclcpp を取了テストが書けないため）。
class BridgeCore {
 public:
  using LinkFactory = std::function<std::shared_ptr<SerialIo>(const std::string&, int)>;

  /// 単調時計 [秒]。テストから明示的に進められるように注入する。
  using Clock = std::function<double()>;

  BridgeCore(BridgeParams params, LinkFactory factory, Clock clock);

  /// 速度指令。舵角と速度の 2 フレームをまとめて送る。
  void OnCmdVel(double vx, double wz);

  /// 関節角 (0.1 度)。軸数がプロトコルの固定長と違うときは送らず false を返す。
  /// コールバックを落とすとノードごと死ぬので、呼び出し側が警告を出す。
  bool OnArmJoint(const std::vector<int16_t>& angles);

  void OnGripper(uint8_t cmd);

  /// 速度 0 のフレームを送る。終了時と、OnWatchdog が send_stop を返したとき。
  void SendStop();

  WatchdogVerdict OnWatchdog(double now_s);

  /// 受信バイトを 1 回だけ読んで復号する。states に FB_STATE、fb_errors に
  /// FB_ERROR のコード、imus に FB_IMU を積む（imus=nullptr なら捨てる）。
  /// リンクが無ければ false。timeout_ms > 0 なら 1 バイト揃うまで待つ。
  bool RxOnce(std::vector<Feedback>* states, std::vector<uint8_t>* fb_errors,
              int timeout_ms, std::vector<ImuReport>* imus = nullptr);

  /// 開いているリンクを返す。開けない場合は nullptr。
  ///
  /// 次の試行は params().reopen_interval_s を空ける。開こうとしたときだけ
  /// error に理由が入る（試行を待っている間は空）。呼び出し側がログを出すのに使う。
  std::shared_ptr<SerialIo> CurrentLink(std::string* error = nullptr);

  /// 読み取り失敗でリンクを隔離する。以降の送信は再オープンまで保留。
  ///
  /// 壊れた libserial オブジェクトは破棄せずプロセス寿命の隔離リストへ移す
  /// （~SerialPort() が noexcept のまま I/O error を投げるため）。
  void DropLink();

  const BridgeParams& params() const { return params_; }

 private:
  /// tx_mutex_ を取った状態で呼ぶ。RX スレッドは link_mutex_ だけを使う。
  void WriteJoined(const std::vector<uint8_t>& bytes);

  void WriteFrames(const std::vector<std::vector<uint8_t>>& frames);

  BridgeParams params_;
  LinkFactory factory_;
  Clock clock_;
  std::mutex tx_mutex_;
  std::mutex link_mutex_;
  std::shared_ptr<SerialIo> link_;
  double next_open_attempt_s_ = 0.0;
  StreamFrameParser parser_;
  double last_cmd_time_s_ = 0.0;
  double last_arm_time_s_ = 0.0;
  bool zero_sent_ = false;
  bool arm_idle_logged_ = false;
};

/// I/O 操作の例外をリンク隔離に変換する。成功なら空、失敗なら理由。
///
/// ROS 層の購読コールバック・ウォッチドッグ・受信ループはこの 1 経路だけを使う。
/// 例外を rclcpp の spin まで通すとプロセスごと落ちるため、捕捉と DropLink を
/// ここに閉じる（テストから実 I/O 例外で検証できる）。
std::string RunSerialOp(BridgeCore& core, const std::function<void()>& op);

}  // namespace umeonigiri_serial_bridge
