// NPU(RKNN) で yolov8 を回し、物体名・枠・把持候補（中心/角度/幅）を配信する。
//
// 既存の detection_node（ONNX/CPU）と同じトピックを出すので、Web UI と
// アーム制御から見ると差し替え可能。CPU 実装より速く、CPU をほとんど使わない。
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "umeonigiri_vision/detection_msg.hpp"
#include "umeonigiri_vision/image_util.hpp"
#include "umeonigiri_vision/publish_rate_gate.hpp"
#include "umeonigiri_vision/rknn_yolo.hpp"

namespace umeonigiri_vision
{
namespace
{

constexpr int kCaptureWidth = 640;
constexpr int kCaptureHeight = 480;
constexpr float kMinContourRatio = 0.15F;   // 枠に対する面積の下限（ノイズ除去）
constexpr float kMaxContourRatio = 0.95F;   // 枠いっぱいの背景を除く

/// 枠の中を二値化して輪郭を取り、把持に要る中心・角度・幅を出す。
struct GraspTarget
{
  cv::Point2f center;
  cv::Size2f size;
  float angle_deg = 0.0F;
  cv::RotatedRect rect;
};

bool ComputeGrasp(const cv::Mat & image_bgr, const YoloBox & box, GraspTarget & out)
{
  const int x0 = std::max(0, static_cast<int>(box.x0));
  const int y0 = std::max(0, static_cast<int>(box.y0));
  const int x1 = std::min(image_bgr.cols - 1, static_cast<int>(box.x1));
  const int y1 = std::min(image_bgr.rows - 1, static_cast<int>(box.y1));
  if (x1 - x0 < 12 || y1 - y0 < 12) {
    return false;
  }
  const cv::Mat roi = image_bgr(cv::Rect(x0, y0, x1 - x0, y1 - y0));
  cv::Mat gray;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);
  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const float roi_area = static_cast<float>(roi.rows * roi.cols);
  const std::vector<cv::Point> * best = nullptr;
  double best_area = 0.0;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < roi_area * kMinContourRatio || area > roi_area * kMaxContourRatio) {
      continue;
    }
    if (best == nullptr || area > best_area) {
      best = &contour;
      best_area = area;
    }
  }
  if (best == nullptr) {
    return false;
  }

  cv::RotatedRect rect = cv::minAreaRect(*best);
  rect.center.x += static_cast<float>(x0);
  rect.center.y += static_cast<float>(y0);
  // 指を開く向きを長辺に揃える（OpenCV の角度は (-90, 0]）。
  float width = rect.size.width;
  float height = rect.size.height;
  float angle = rect.angle;
  if (width < height) {
    std::swap(width, height);
    angle += 90.0F;
  }
  if (angle > 90.0F) {
    angle -= 180.0F;
  }
  out.center = rect.center;
  out.size = cv::Size2f(width, height);
  out.angle_deg = angle;
  out.rect = rect;
  return true;
}

}  // namespace

class YoloGraspNode
{
public:
  YoloGraspNode(
    const rclcpp::Node::SharedPtr & node, const std::string & model, const std::string & device,
    float conf, float iou, const std::string & labels_csv, double display_fps,
    const std::string & image_topic, bool topic_input, double detection_fps,
    const std::string & frame_id)
  : node_(node), device_(device), conf_(conf), iou_(iou), topic_input_(topic_input),
    frame_id_(frame_id), display_period_(display_fps > 0.0 ? 1.0 / display_fps : 0.0),
    detector_(model), labels_(LoadLabels(labels_csv)), publish_gate_(detection_fps)
  {
    if (!detector_.ok()) {
      throw std::runtime_error("RKNN モデルを読めません: " + model);
    }
    pub_detections_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
    pub_annotated_ = node_->create_publisher<sensor_msgs::msg::Image>("detection_image", 10);
    pub_grasp_ = node_->create_publisher<sensor_msgs::msg::Image>("grasp_image", 10);
    pub_target_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(
      "target", rclcpp::QoS(rclcpp::KeepLast(1)));

    if (topic_input) {
      if (image_topic.empty()) {
        throw std::runtime_error("トピック入力を指定するには image_topic が必要です");
      }
      // 購読はメンバで保持する。ローカルに捨てると Jazzy では即破棄される (P12)。
      image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {OnImage(msg);});
      RCLCPP_INFO(node_->get_logger(), "NPU 検出を開始: 入力トピック %s, ラベル %zu 件",
        image_topic.c_str(), labels_.size());
      return;
    }

    capture_.open(device_, cv::CAP_V4L2);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, kCaptureWidth);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, kCaptureHeight);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (!capture_.isOpened()) {
      throw std::runtime_error("カメラを開けません: " + device);
    }
    RCLCPP_INFO(node_->get_logger(), "NPU 検出を開始: 入力 %d, ラベル %zu 件", detector_.input_size(),
      labels_.size());
  }

  /// 静止画像 1 枚だけで推論して配信し、終了する（動作確認用）。
  void RunOnce(const std::string & image_path)
  {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      throw std::runtime_error("画像を読めません: " + image_path);
    }
    auto boxes = detector_.Detect(image, conf_, iou_);
    std::vector<GraspTarget> grasps(boxes.size());
    std::vector<char> valid(boxes.size(), 0);
    for (size_t i = 0; i < boxes.size(); ++i) {
      valid[i] = ComputeGrasp(image, boxes[i], grasps[i]) ? 1 : 0;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      boxes_ = std::move(boxes);
      grasps_ = std::move(grasps);
      valid_ = std::move(valid);
    }
    ComposeAndPublish(image);
    RCLCPP_INFO(node_->get_logger(), "静止画像: 検出 %zu 件  推論 %.1f ms", boxes_.size(),
      detector_.last_infer_ms());
    // 1 配信で即終了すると購読側の discovery が間に合わずメッセージが届かない。
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void Run()
  {
    std::thread infer([this] {InferLoop();});
    if (topic_input_) {
      // 購読コールバックは executor を回さないと届かない。推論スレッドは別に動く。
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node_);
      executor.spin();
    } else {
      CaptureLoop();
    }
    stop_ = true;
    cond_.notify_all();
    if (infer.joinable()) {
      infer.join();
    }
    capture_.release();
  }

  /// gz bridge が出す rgb8 を BGR に直して最新フレームとして差し替える。
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      cv::Mat bgr = BgrFromImageMsg(*msg);
      const rclcpp::Time stamp(msg->header.stamp);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(bgr);
        latest_stamp_ = stamp;
        has_frame_ = true;
      }
      cond_.notify_one();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "画像を捨てます: %s", e.what());
    }
  }

private:
  static std::vector<std::string> LoadLabels(const std::string & path)
  {
    std::vector<std::string> labels;
    if (path.empty()) {
      return labels;
    }
    FILE * file = std::fopen(path.c_str(), "r");
    if (file == nullptr) {
      return labels;
    }
    char line[128];
    while (std::fgets(line, sizeof(line), file) != nullptr) {
      std::string text(line);
      while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
        text.pop_back();
      }
      if (!text.empty()) {
        labels.push_back(text);
      }
    }
    std::fclose(file);
    return labels;
  }

  /// カメラから読んで、最新の推論結果を重ねて配信する（カメラレート）。
  void CaptureLoop()
  {
    while (rclcpp::ok() && !stop_) {
      cv::Mat frame;
      if (!capture_.read(frame) || frame.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = frame;
        latest_stamp_ = node_->now();
        has_frame_ = true;
      }
      cond_.notify_one();

      const auto now = std::chrono::steady_clock::now();
      const double since_publish =
        std::chrono::duration<double>(now - last_publish_).count();
      if (display_period_ > 0.0 && since_publish < display_period_) {
        continue;  // 配信はレート上限に抑える。カメラは読み続けて最新を保つ。
      }
      last_publish_ = now;
      ComposeAndPublish(frame);
      ++frames_;
      if (frames_ % 60 == 0) {
        RCLCPP_INFO(node_->get_logger(), "表示 %d フレーム  推論 %.1f ms  検出 %zu", frames_,
          detector_.last_infer_ms(), boxes_.size());
      }
    }
  }

  /// 推論は「最新の 1 枚」だけを回す。古いフレームは捨てて遅延を伸ばさない。
  void InferLoop()
  {
    while (!stop_) {
      cv::Mat frame;
      rclcpp::Time stamp;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] {return has_frame_ || stop_;});
        if (stop_) {
          break;
        }
        frame = latest_.clone();
        stamp = latest_stamp_;
        has_frame_ = false;
      }
      // このスレッドで例外が外へ出ると std::terminate でプロセスごと落ちる。
      // main() の try は join を待つ側で、スレッドの例外は拾えない。壊れた
      // フレーム 1 枚でビジョン全体を落とさないよう、ここで受けて次のフレームへ進む。
      try {
        auto boxes = detector_.Detect(frame, conf_, iou_);
        std::vector<GraspTarget> grasps(boxes.size());
        std::vector<char> valid(boxes.size(), 0);
        for (size_t i = 0; i < boxes.size(); ++i) {
          valid[i] = ComputeGrasp(frame, boxes[i], grasps[i]) ? 1 : 0;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          boxes_ = boxes;
          grasps_ = std::move(grasps);
          valid_ = std::move(valid);
        }
        if (topic_input_) {
          const auto now = std::chrono::steady_clock::now();
          const double seconds =
            std::chrono::duration<double>(now.time_since_epoch()).count();
          if (publish_gate_.Ready(seconds)) {
            PublishTopicResults(frame, stamp, boxes);
            ++topic_frames_;
            if (topic_frames_ % 60 == 0) {
              RCLCPP_INFO(node_->get_logger(),
                "トピック入力 %d フレーム  推論 %.1f ms  検出 %zu", topic_frames_,
                detector_.last_infer_ms(), boxes.size());
            }
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "推論に失敗しました (フレームを捨てます): %s", e.what());
      }
    }
  }

  cv::Mat Compose(const cv::Mat & frame)
  {
    cv::Mat canvas = frame.clone();
    std::vector<YoloBox> boxes;
    std::vector<GraspTarget> grasps;
    std::vector<char> valid;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      boxes = boxes_;
      grasps = grasps_;
      valid = valid_;
    }
    for (size_t i = 0; i < boxes.size(); ++i) {
      const bool is_target = i == 0;
      const cv::Scalar color = is_target ? cv::Scalar(0, 255, 0) : cv::Scalar(200, 200, 200);
      const cv::Point p0(static_cast<int>(boxes[i].x0), static_cast<int>(boxes[i].y0));
      const cv::Point p1(static_cast<int>(boxes[i].x1), static_cast<int>(boxes[i].y1));
      cv::rectangle(canvas, p0, p1, color, is_target ? 2 : 1);
      const std::string label = LabelOf(boxes[i].class_id);
      char text[96];
      std::snprintf(text, sizeof(text), "%s %.2f", label.c_str(), boxes[i].score);
      cv::putText(canvas, text, cv::Point(p0.x, std::max(14, p0.y - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
      if (!valid[i]) {
        continue;
      }
      cv::Point2f pts[4];
      grasps[i].rect.points(pts);
      for (int k = 0; k < 4; ++k) {
        cv::line(canvas, pts[k], pts[(k + 1) % 4], cv::Scalar(0, 255, 255), 2);
      }
      cv::circle(canvas, grasps[i].center, 5, cv::Scalar(0, 255, 255), -1);
      if (is_target) {
        char line[160];
        std::snprintf(line, sizeof(line), "TARGET %s (%.0f,%.0f) angle %.1f width %.0fpx",
          label.c_str(), grasps[i].center.x, grasps[i].center.y, grasps[i].angle_deg,
          grasps[i].size.width);
        cv::putText(canvas, line, cv::Point(8, canvas.rows - 12),
          cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
      }
    }
    char header[96];
    std::snprintf(header, sizeof(header), "infer %.1f ms  det %zu", detector_.last_infer_ms(),
      boxes.size());
    cv::putText(canvas, header, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(255, 255, 255), 2);
    return canvas;
  }

  void ComposeAndPublish(const cv::Mat & frame)
  {
    const auto stamp = node_->now();
    const cv::Mat canvas = Compose(frame);
    Publish(canvas, stamp);
    std::vector<YoloBox> boxes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      boxes = boxes_;
    }
    PublishDetections(stamp, boxes);
  }

  /// トピック入力では推論のたびに detections を出す。描画は購読者が居るときだけ。
  void PublishTopicResults(
    const cv::Mat & frame, const rclcpp::Time & stamp, const std::vector<YoloBox> & boxes)
  {
    PublishDetections(stamp, boxes);
    if (pub_annotated_->get_subscription_count() > 0 || pub_grasp_->get_subscription_count() > 0) {
      Publish(Compose(frame), stamp);
    }
  }

  std::string LabelOf(int class_id) const
  {
    if (class_id >= 0 && static_cast<size_t>(class_id) < labels_.size()) {
      return labels_[static_cast<size_t>(class_id)];
    }
    return std::to_string(class_id);
  }

  void Publish(const cv::Mat & canvas, const rclcpp::Time & stamp)
  {
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp = stamp;
    message->header.frame_id = frame_id_;
    message->height = static_cast<uint32_t>(canvas.rows);
    message->width = static_cast<uint32_t>(canvas.cols);
    message->encoding = "bgr8";
    message->is_bigendian = 0;
    message->step = static_cast<uint32_t>(canvas.cols * 3);
    message->data.resize(static_cast<size_t>(message->step) * message->height);
    std::memcpy(message->data.data(), canvas.data, message->data.size());
    pub_grasp_->publish(*message);
    pub_annotated_->publish(std::move(message));
  }

  void PublishDetections(
    const rclcpp::Time & stamp, const std::vector<YoloBox> & boxes)
  {
    vision_msgs::msg::Detection2DArray message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    for (size_t i = 0; i < boxes.size(); ++i) {
      DetectionIdentity identity;
      identity.class_id = boxes[i].class_id;
      identity.class_name = LabelOf(boxes[i].class_id);
      identity.score = boxes[i].score;
      identity.x0 = boxes[i].x0;
      identity.y0 = boxes[i].y0;
      identity.x1 = boxes[i].x1;
      identity.y1 = boxes[i].y1;
      vision_msgs::msg::Detection2D detection;
      FillDetection2D(identity, detection);
      message.detections.push_back(detection);
    }
    pub_detections_->publish(message);
    // アームは先頭（最も手前）だけ見る。
    if (!message.detections.empty()) {
      vision_msgs::msg::Detection2DArray target = message;
      target.detections = {message.detections.front()};
      pub_target_->publish(target);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string device_;
  float conf_;
  float iou_;
  bool topic_input_;
  std::string frame_id_;
  double display_period_;
  RknnYolo detector_;
  std::vector<std::string> labels_;
  PublishRateGate publish_gate_;

  cv::VideoCapture capture_;
  std::mutex mutex_;
  std::condition_variable cond_;
  cv::Mat latest_;
  rclcpp::Time latest_stamp_{};
  bool has_frame_ = false;
  int topic_frames_ = 0;
  bool stop_ = false;
  int frames_ = 0;
  std::chrono::steady_clock::time_point last_publish_ {};
  std::vector<YoloBox> boxes_;
  std::vector<GraspTarget> grasps_;
  std::vector<char> valid_;

  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_target_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_grasp_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace umeonigiri_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 1 プロセス 1 ノード。-r __node:= で複数ノードが同じ名前に潰れるのを避ける (P12)。
  auto node = rclcpp::Node::make_shared("yolo_grasp");
  const std::string model = node->declare_parameter<std::string>("model", "");
  const std::string labels = node->declare_parameter<std::string>("labels", "");
  const std::string device = node->declare_parameter<std::string>("device", "/dev/video0");
  const double conf = node->declare_parameter<double>("conf", 0.45);
  const double iou = node->declare_parameter<double>("iou", 0.45);
  const double display_fps = node->declare_parameter<double>("display_fps", 15.0);
  const double detection_fps = node->declare_parameter<double>("detection_fps", 30.0);
  const std::string image = node->declare_parameter<std::string>("image", "");
  const std::string image_topic = node->declare_parameter<std::string>("image_topic", "");
  const bool topic_input = node->declare_parameter<bool>("use_topic_input", false);
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "camera_link");
  // ここで shutdown すると、この後のノード生成が無効なコンテキストで失敗する。

  try {
    auto grasp = std::make_shared<umeonigiri_vision::YoloGraspNode>(
      node, model, device, static_cast<float>(conf), static_cast<float>(iou), labels, display_fps,
      image_topic, topic_input, detection_fps, frame_id);
    if (image.empty()) {
      grasp->Run();
    } else {
      grasp->RunOnce(image);
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "起動に失敗: %s\n", e.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
