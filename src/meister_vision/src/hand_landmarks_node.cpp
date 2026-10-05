// SPDX-License-Identifier: MIT
/// 画像を購読して手の keypoint を geometry_msgs/PoseArray で配信するノード。
///
/// 手なしの配信契約
/// --------------
/// 検出が外れたフレームでは何も配信しない。直前の検出から hand_timeout_sec
/// を超えても手が写っていなければ、poses が空の配列を 1 度だけ配信して、
/// 以降は無音にする。
///
///   - 毎フレーム空配列を流さない理由: 購読側は「手が見えない」と「手が
///     写っているのに keypoint が 1 つも無い」を区別できない。配信件数が
///     カメラレートに比例して増え、件数を数える購読側はそれを「手が消えた」
///     の合図として扱わざるを得なくなる。1 度だけ流せば合図として一意に
///     読める。
///   - 一度も手が見えていない間は無音の理由: 捨てるべき古い状態が無い。
///   - 空配列を 2 度目以降送らない理由: 1 度目で購読側は既に「手なし」を
///     認識済みなので、2 度目以降では状態が変わらない。
///   - 猶予 hand_timeout_sec の既定 0.5 秒の根拠: 手首搭載カメラは
///     5-10 Hz で動く。猶予は「処理済みのフレーム」に対するもので、許容
///     する連続脱落フレーム数は hand_timeout_sec × rate になるため 0.5 秒
///     は 2.5-5 フレーム分に相当する。この程度の脱落は手元の影やアームの
///     振動で普通に起きるため、「手が映っていない」は猶予が尽きた後に
///     だけ言う。
///
/// モデルが無い場合はコンストラクタが例外を投げ、main() はそれを握り潰さず
/// プロセスを落とす。「手なし」を無期限に待ち続けると上流が異常を取り逃す
/// ため、起動時に判明する問題を起動時に出す。
///
/// パラメータ:
///   model_path         (string, "")                     手 keypoint モデルパス
///   image_topic        (string, "/camera/image_raw")
///   score_threshold    (double, 0.2)                    手とみなす最小スコア
///   rate               (double, 3.0)                    最大処理レート [Hz]
///   inference_threads  (int,    2)                      ONNX 推論スレッド数
///   hand_timeout_sec   (double, 0.5)                    手なしを確定するまでの猶予
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "meister_vision/hand_landmark_detector.hpp"
#include "meister_vision/image_util.hpp"

namespace meister_vision {
namespace {

constexpr char kNodeName[] = "hand_landmarks";

geometry_msgs::msg::PoseArray BuildPoseArray(
    const sensor_msgs::msg::Image& msg,
    const std::optional<HandLandmarks>& result) {
  // result が空でも配列を返す。空を「手なし」の唯一の符号にできるのは、
  // Detect() が「K 点ある / 0 点」の 2 値しか返さないため。点数が減る途中
  // 状態が無いので、空が「壊れた検出」と読まれる余地が無い。
  //
  // orientation を恒等クォータニオンで埋めるのは、Pose の既定値が全ゼロで
  // クォータニオンとしては「長さが 0 の不正な回転」を表すため。有効でない
  // クォータニオンを渡すと下流の変換が破綻する。
  geometry_msgs::msg::PoseArray array;
  array.header = msg.header;
  if (!result.has_value()) {
    return array;
  }
  array.poses.reserve(result->landmarks.size());
  for (const cv::Point2f& p : result->landmarks) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    array.poses.push_back(pose);
  }
  return array;
}

class HandLandmarksNode final : public rclcpp::Node {
 public:
  HandLandmarksNode()
      : rclcpp::Node(kNodeName),
        min_interval_(MinInterval(*this)),
        hand_timeout_(declare_parameter<double>("hand_timeout_sec", 0.5)) {
    const std::string image_topic =
        declare_parameter<std::string>("image_topic", "/camera/image_raw");
    detector_ = std::make_unique<HandLandmarkDetector>(
        declare_parameter<std::string>("model_path", ""),
        static_cast<float>(declare_parameter<double>("score_threshold", 0.2)),
        kHandDefaultNumLandmarks,
        declare_parameter<int>("inference_threads", 2));

    RCLCPP_INFO(get_logger(), "手 keypoint モデルを読み込みました: %s",
                detector_->model_path().c_str());
    RCLCPP_INFO(get_logger(),
                "画像トピック '%s' を購読開始 (最大 %.2f Hz, 手なし猶予 %.3f 秒)",
                image_topic.c_str(), min_interval_ > 0.0 ? 1.0 / min_interval_ : 0.0,
                hand_timeout_);

    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("hand_landmarks", 10);
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnImage(*msg); });
  }

 private:
  static double MinInterval(rclcpp::Node& node) {
    const double rate = node.declare_parameter<double>("rate", 3.0);
    return rate > 0.0 ? 1.0 / rate : 0.0;
  }

  /// 今のフレームで「手なし」を 1 度だけ通知してよいか。
  /// 1 度だけ、という不変条件。満たすうちは「消えた」の合図を増やさない。
  bool ClearDue(const rclcpp::Time& stamp) const {
    if (last_detection_time_.nanoseconds() == 0 || clear_published_) {
      return false;
    }
    return (stamp - last_detection_time_).seconds() >= hand_timeout_;
  }

  void OnImage(const sensor_msgs::msg::Image& msg) {
    if (pub_->get_subscription_count() == 0) {
      return;
    }
    const rclcpp::Time stamp = now();
    if (last_process_time_.nanoseconds() != 0 &&
        (stamp - last_process_time_).seconds() < min_interval_) {
      return;
    }
    last_process_time_ = stamp;
    ++process_count_;

    std::optional<HandLandmarks> result;
    try {
      result = detector_->Detect(BgrFromImageMsg(msg));
    } catch (const std::exception& e) {
      // 1 枚の変換失敗で購読を黙って落とさない。不正なエンコーディングも
      // ここで吸収し、ノードは動き続ける。
      RCLCPP_ERROR(get_logger(), "手 keypoint 推定に失敗しました: %s", e.what());
      return;
    }

    // 検出時刻は処理完了時刻で持つ。処理中の時刻だと、推論に時間のかかった
    // フレームで猶予を消費し、落ちかけを「消えた」と誤読する。
    const rclcpp::Time seen_at = now();
    if (!result.has_value()) {
      if (!ClearDue(seen_at)) {
        return;
      }
      clear_published_ = true;
      pub_->publish(BuildPoseArray(msg, std::nullopt));
      RCLCPP_DEBUG(get_logger(), "[%zu] 手なしを確定 (poses 0 で通知)",
                   process_count_);
      return;
    }

    last_detection_time_ = seen_at;
    clear_published_ = false;
    pub_->publish(BuildPoseArray(msg, result));
    RCLCPP_DEBUG(get_logger(), "[%zu] %zu keypoint", process_count_,
                 result->landmarks.size());
  }

  double min_interval_;
  double hand_timeout_;
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  std::size_t process_count_ = 0;
  bool clear_published_ = false;
  std::unique_ptr<HandLandmarkDetector> detector_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
};

}  // namespace
}  // namespace meister_vision

int main(int argc, char** argv) {
  // モデルが無い、ノードの初期化失敗など、構築中の例外を握らないと terminate で
  // abort して終了コードが 134 になる。理由を標準出力に書いて 1 で終わる。
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<meister_vision::HandLandmarksNode>());
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    rclcpp::shutdown();
    rc = 1;
  }
  return rc;
}