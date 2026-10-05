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
///   fps       (double, 15.0)         取得フレームレート
///   frame_id  (string, camera_link) 配信するフレーム ID
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
constexpr double kDefaultFps = 15.0;

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
    cv::VideoCapture cap;
    const bool numeric = !device_.empty() &&
                         device_.find_first_not_of("0123456789") ==
                             std::string::npos;
    if (numeric) {
      cap.open(std::stoi(device_));
    } else {
      cap.open(device_);
    }
    if (!cap.isOpened()) {
      return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    if (fps_ > 0.0) {
      cap.set(cv::CAP_PROP_FPS, fps_);
    }
    capture_ = std::move(cap);
    return true;
  }

  void OnTimer() {
    if (!capture_.isOpened()) {
      // 起動時にデバイスが不在でも落とさず待ち受けるため、エラーを出して
      // 次周期に開き直す。
      RCLCPP_ERROR(get_logger(), "カメラを開けませんでした: %s、リトライします",
                   device_.c_str());
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
    msg.header.stamp = now();
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