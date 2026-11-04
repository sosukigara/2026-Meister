// SPDX-License-Identifier: MIT
/// PC のカメラを sensor_msgs/Image として配信するノード。
///
/// cv::VideoCapture で每隔 1 枚取り、/camera/image_raw に流す。デバイスが
/// 不在でもプロセスを落とさず、周期ごとに開き直す。
///
/// パラメータ:
///   device    (string, /dev/video0) カメラ番号またはデバイスパス
///   width     (int,    640)          取得幅
///   height    (int,    480)          取得高さ
///   fps       (double, 5.0)          取得フレームレート
///   frame_id  (string, camera_link) 配信するフレーム ID
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "meister_vision/image_util.hpp"

namespace meister_vision {
namespace {

constexpr char kNodeName[] = "pc_camera";
constexpr char kTopic[] = "/camera/image_raw";
constexpr int kDefaultWidth = 640;
constexpr int kDefaultHeight = 480;
constexpr double kDefaultFps = 5.0;

class PcCameraNode final : public rclcpp::Node {
 public:
  PcCameraNode()
      : rclcpp::Node(kNodeName),
        device_(declare_parameter<std::string>("device", "/dev/video0")),
        width_(declare_parameter<int>("width", kDefaultWidth)),
        height_(declare_parameter<int>("height", kDefaultHeight)),
        fps_(declare_parameter<double>("fps", kDefaultFps)),
        frame_id_(declare_parameter<std::string>("frame_id", "camera_link")) {
    const double period = fps_ > 0.0 ? 1.0 / fps_ : 1.0 / kDefaultFps;
    pub_ = create_publisher<sensor_msgs::msg::Image>(kTopic, 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(period),
                               [this]() { OnTimer(); });
  }

  ~PcCameraNode() override { CloseCapture(); }

 private:
  void CloseCapture() {
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

  bool OpenDevice() {
    // VideoCapture は数値インデックスとデバイスパスの両方を受け付ける。
    // 数字だけの文字列は整数に寄せて、バックエンド選択の誤動作を避ける。
    const bool numeric = !device_.empty() &&
                         device_.find_first_not_of("0123456789") ==
                             std::string::npos;
    // capture_ に直接開く (std::move が VideoCapture を正しく移動しない環境対策)。
    if (numeric) {
      capture_.open(std::stoi(device_), cv::CAP_V4L2);
    } else {
      capture_.open(device_, cv::CAP_V4L2);
    }
    if (!capture_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "open failed (backend=%d)", cv::CAP_V4L2);
      return false;
    }
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    if (fps_ > 0.0) {
      capture_.set(cv::CAP_PROP_FPS, fps_);
    }
    // V4L2 の既定キューは 4 枚。読み出しが遅いと 100ms 単位で古いフレームを
    // 返し、体感ラグになるため常に最新 1 枚だけを保持させる。
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return true;
  }

  void OnTimer() {
    if (!capture_.isOpened()) {
      // 起動時や切断時にデバイスが不在でも落とさず、次周期に開き直す。
      OpenDevice();
      return;
    }
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_ERROR(get_logger(), "フレーム取得に失敗しました、再接続します");
      CloseCapture();
      return;
    }
    sensor_msgs::msg::Image msg = ImageMsgFromBgr(frame);
    // V4L2 のバッファタイムスタンプ (単調時計) から撮像時刻を復元する。
    // 取得時刻をそのまま入れると下流の遅延が実際より小さく出る。
    double age_s = 0.0;
    const double pos_ms = capture_.get(cv::CAP_PROP_POS_MSEC);
    if (pos_ms > 0.0) {
      const double mono_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      const double candidate = (mono_ms - pos_ms) / 1000.0;
      // 単調時計でないバックエンドでは巨大な値になる。妥当域だけ採用する。
      if (candidate >= 0.0 && candidate <= 0.5) {
        age_s = candidate;
      }
    }
    msg.header.stamp = now() - rclcpp::Duration::from_seconds(age_s);
    msg.header.frame_id = frame_id_;
    pub_->publish(msg);
  }

  std::string device_;
  int width_;
  int height_;
  double fps_;
  std::string frame_id_;
  cv::VideoCapture capture_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace meister_vision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<meister_vision::PcCameraNode>());
  rclcpp::shutdown();
  return 0;
}