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
///   rate               (double, 10.0)        処理レート [Hz]
///   inference_threads  (int,    4)           ONNX 推論スレッド数
///   target_class       (int,    -1)          追うクラス (-1 は指定なし)
///   target_lock_radius (double, 120.0)       同一対象とみなす移動量 [px]
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
#include "meister_vision/target_selector.hpp"
#include "meister_vision/yolo_detector.hpp"

namespace meister_vision {
namespace {

constexpr char kNodeName[] = "umeonigiri_vision";

vision_msgs::msg::Detection2D ToDetection2D(
    const std_msgs::msg::Header& header, const Detection& det) {
  vision_msgs::msg::Detection2D d2d;
  d2d.header = header;
  d2d.id = det.class_name;

  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = std::to_string(det.class_id);
  hyp.hypothesis.score = det.confidence;
  d2d.results.push_back(hyp);

  d2d.bbox.center.position.x = (det.x1 + det.x2) / 2.0;
  d2d.bbox.center.position.y = (det.y1 + det.y2) / 2.0;
  d2d.bbox.size_x = det.x2 - det.x1;
  d2d.bbox.size_y = det.y2 - det.y1;
  return d2d;
}

vision_msgs::msg::Detection2DArray BuildDetectionArray(
    const sensor_msgs::msg::Image& msg, const std::vector<Detection>& dets) {
  vision_msgs::msg::Detection2DArray array;
  array.header = msg.header;
  array.detections.reserve(dets.size());
  for (const Detection& det : dets) {
    array.detections.push_back(ToDetection2D(msg.header, det));
  }
  return array;
}

class DetectionNode final : public rclcpp::Node {
 public:
  DetectionNode()
      : rclcpp::Node(kNodeName),
        publish_annotated_(declare_parameter<bool>("publish_annotated", true)) {
    const std::string image_topic =
        declare_parameter<std::string>("image_topic", "image_raw");
    const double rate = std::max(0.1, declare_parameter<double>("rate", 5.0));
    const int inference_threads =
        declare_parameter<int>("inference_threads", 4);
    const float conf_threshold =
        static_cast<float>(declare_parameter<double>("conf_threshold", 0.25));
    TargetSelectorOptions selector_options;
    selector_options.target_class = declare_parameter<int>("target_class", -1);
    selector_options.min_confidence = conf_threshold;
    selector_options.lock_radius_px =
        static_cast<float>(declare_parameter<double>("target_lock_radius", 120.0));
    target_selector_ = std::make_unique<TargetSelector>(selector_options);

    detector_ = std::make_unique<YoloDetector>(
        declare_parameter<std::string>("model_path", ""), conf_threshold,
        static_cast<float>(declare_parameter<double>("iou_threshold", 0.45)),
        inference_threads);

    RCLCPP_INFO(get_logger(), "モデルを読み込みました: %s",
                detector_->model_path().c_str());
    RCLCPP_INFO(get_logger(), "画像トピック '%s' を購読開始 (%.2f Hz, %d スレッド)",
                image_topic.c_str(), rate, inference_threads);

    pub_detections_ =
        create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
    pub_annotated_ =
        create_publisher<sensor_msgs::msg::Image>("detection_image", 10);
    // アーム制御は小さなメッセージだけ見る。depth=1 で古い指令を捨てる。
    pub_target_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "target", rclcpp::QoS(rclcpp::KeepLast(1)));

    // リアルタイム用 QoS。depth=1 で常に最新フレームだけを保持し、
    // 古いフレームの処理待ちで遅延が積み上がらないようにする。
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    image_qos.best_effort();
    image_qos.durability_volatile();
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, image_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          OnImage(std::move(msg));
        });

    // 購読コールバックでは推論せず最新フレームだけを保持し、推論は専用
    // スレッドで一定周期に回す。rclcpp の wall timer はコールバックの
    // 実行時間ぶん周期が伸びるため、絶対時刻スケジュールの worker を使う。
    worker_ = std::thread([this, rate]() { WorkerLoop(rate); });
  }

  ~DetectionNode() override {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      running_ = false;
    }
    frame_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  bool AnyoneListening() const {
    if (pub_detections_->get_subscription_count() > 0 ||
        pub_target_->get_subscription_count() > 0) {
      return true;
    }
    return publish_annotated_ &&
           pub_annotated_->get_subscription_count() > 0;
  }

  /// 推論はせず、最新フレームへの参照を差し替えてワーカーを起こすだけ（O(1)）。
  void OnImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = std::move(msg);
      has_new_frame_ = true;
    }
    frame_cv_.notify_one();
  }

  void WorkerLoop(double rate) {
    const auto period = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
    auto next_allowed = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(frame_mutex_);
    while (running_) {
      // 固定周期でポーリングすると、カメラと周期が近いときに位相がぶつかって
      // 半分のフレームを取りこぼす (30Hz 同士で 15Hz になる)。
      // 到着で起きる方式にして、レート上限だけを守る。
      frame_cv_.wait(lock, [this]() { return !running_ || has_new_frame_; });
      if (!running_) {
        break;
      }
      if (std::chrono::steady_clock::now() < next_allowed) {
        lock.unlock();
        std::this_thread::sleep_until(next_allowed);
        lock.lock();
        if (!running_ || !has_new_frame_) {
          continue;
        }
      }
      sensor_msgs::msg::Image::ConstSharedPtr msg = latest_frame_;
      has_new_frame_ = false;
      lock.unlock();
      next_allowed = std::chrono::steady_clock::now() + period;
      if (AnyoneListening()) {
        ProcessFrame(*msg);
      }
      lock.lock();
    }
  }

  void ProcessFrame(const sensor_msgs::msg::Image& msg) {
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
    PublishTarget(msg, detections);

    // 描画と 921KB の変換は馬鹿にならない。制御系は detections だけを
    // 購読するので、見る人がいないときは作らない。
    if (publish_annotated_ && pub_annotated_->get_subscription_count() > 0) {
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

  /// アームを向ける 1 個を選んで配信する。該当なしは空配列。
  void PublishTarget(const sensor_msgs::msg::Image& msg,
                     const std::vector<Detection>& detections) {
    const Target target = target_selector_->Select(
        detections, static_cast<int>(msg.width),
        static_cast<int>(msg.height));
    vision_msgs::msg::Detection2DArray out;
    out.header = msg.header;
    if (target.valid) {
      out.detections.push_back(ToDetection2D(msg.header, target.detection));
    }
    pub_target_->publish(out);

    RCLCPP_DEBUG(get_logger(),
                 "target valid=%d locked=%d offset=(%.3f, %.3f)", target.valid,
                 target.locked, target.offset_x, target.offset_y);
  }

  bool publish_annotated_;
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_frame_;
  bool has_new_frame_ = false;
  std::size_t process_count_ = 0;
  std::unique_ptr<YoloDetector> detector_;
  std::unique_ptr<TargetSelector> target_selector_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_target_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  std::thread worker_;
  std::atomic<bool> running_{true};
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
