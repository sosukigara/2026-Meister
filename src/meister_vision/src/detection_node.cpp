// SPDX-License-Identifier: MIT
/// 画像トピックを購読して YOLOv8n で物体検出するノード。
///
/// 検出結果は vision_msgs/Detection2DArray を detections に、描画済み画像を
/// detection_image に配信する。購読者がいない間は推論自体を止める。
///
/// パラメータ:
///   model_path         (string, "")          モデルパス。空なら自動解決
///   conf_threshold     (double, 0.25)        信頼度しきい値
///   iou_threshold      (double, 0.45)        NMS の IoU しきい値
///   image_topic        (string, "image_raw") 購読する画像トピック
///   publish_annotated  (bool,   true)        描画済み画像を配信するか
///   rate               (double, 3.0)         最大処理レート [Hz]
///   inference_threads  (int,    2)           ONNX 推論スレッド数
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "meister_vision/draw.hpp"
#include "meister_vision/image_util.hpp"
#include "meister_vision/yolo_detector.hpp"

namespace meister_vision {
namespace {

constexpr char kNodeName[] = "meister_vision";

vision_msgs::msg::Detection2DArray BuildDetectionArray(
    const sensor_msgs::msg::Image& msg, const std::vector<Detection>& dets) {
  vision_msgs::msg::Detection2DArray array;
  array.header = msg.header;
  array.detections.reserve(dets.size());
  for (const Detection& det : dets) {
    vision_msgs::msg::Detection2D d2d;
    d2d.header = msg.header;
    d2d.id = det.class_name;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = std::to_string(det.class_id);
    hyp.hypothesis.score = det.confidence;
    d2d.results.push_back(hyp);

    d2d.bbox.center.position.x = (det.x1 + det.x2) / 2.0;
    d2d.bbox.center.position.y = (det.y1 + det.y2) / 2.0;
    d2d.bbox.size_x = det.x2 - det.x1;
    d2d.bbox.size_y = det.y2 - det.y1;
    array.detections.push_back(d2d);
  }
  return array;
}

class DetectionNode final : public rclcpp::Node {
 public:
  DetectionNode()
      : rclcpp::Node(kNodeName),
        publish_annotated_(declare_parameter<bool>("publish_annotated", true)),
        min_interval_(1.0 / declare_parameter<double>("rate", 3.0)) {
    const std::string image_topic =
        declare_parameter<std::string>("image_topic", "image_raw");
    detector_ = std::make_unique<YoloDetector>(
        declare_parameter<std::string>("model_path", ""),
        static_cast<float>(declare_parameter<double>("conf_threshold", 0.25)),
        static_cast<float>(declare_parameter<double>("iou_threshold", 0.45)),
        declare_parameter<int>("inference_threads", 2));

    RCLCPP_INFO(get_logger(), "モデルを読み込みました: %s",
                detector_->model_path().c_str());
    RCLCPP_INFO(get_logger(), "画像トピック '%s' を購読開始 (最大 %.2f Hz)",
                image_topic.c_str(), 1.0 / min_interval_);

    pub_detections_ =
        create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
    pub_annotated_ =
        create_publisher<sensor_msgs::msg::Image>("detection_image", 10);
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          OnImage(*msg);
        });
  }

 private:
  bool AnyoneListening() const {
    if (pub_detections_->get_subscription_count() > 0) {
      return true;
    }
    return publish_annotated_ &&
           pub_annotated_->get_subscription_count() > 0;
  }

  void OnImage(const sensor_msgs::msg::Image& msg) {
    if (!AnyoneListening()) {
      return;
    }
    const rclcpp::Time stamp = now();
    if (last_process_time_.nanoseconds() != 0 &&
        (stamp - last_process_time_).seconds() < min_interval_) {
      return;
    }
    last_process_time_ = stamp;
    ++process_count_;

    std::vector<Detection> detections;
    cv::Mat bgr;
    try {
      bgr = BgrFromImageMsg(msg);
      detections = detector_->Detect(bgr);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "検出処理に失敗しました: %s", e.what());
      return;
    }

    pub_detections_->publish(BuildDetectionArray(msg, detections));

    if (publish_annotated_) {
      // cv_bridge の cv2_to_imgmsg は実行時に KeyError を起こす環境が
      // あるので、手動変換に固定する（購読側の変換は問題なし）。
      sensor_msgs::msg::Image annotated_msg = ImageMsgFromBgr(
          DrawDetections(bgr, detections));
      annotated_msg.header = msg.header;
      pub_annotated_->publish(annotated_msg);
    }

    RCLCPP_DEBUG(get_logger(), "[%zu] %zu 件検出", process_count_,
                 detections.size());
  }

  bool publish_annotated_;
  double min_interval_;
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};
  std::size_t process_count_ = 0;
  std::unique_ptr<YoloDetector> detector_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;
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
    rclcpp::spin(std::make_shared<meister_vision::DetectionNode>());
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    rclcpp::shutdown();
    rc = 1;
  }
  return rc;
}