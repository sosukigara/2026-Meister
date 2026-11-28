#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "umeonigiri_vision/box_center_filter.hpp"
#include "umeonigiri_vision/detection_msg.hpp"
#include "umeonigiri_vision/image_util.hpp"
#include "umeonigiri_vision/multi_object_tracker.hpp"
#include "umeonigiri_vision/publish_rate_gate.hpp"
#include "umeonigiri_vision/rknn_fastsam.hpp"
#include "umeonigiri_vision/rknn_yolo.hpp"

namespace umeonigiri_vision
{
namespace
{

constexpr int kCaptureWidth = 640;
constexpr int kCaptureHeight = 480;
constexpr float kMinContourRatio = 0.12F;   // 枠に対する面積の下限（ノイズ除去）
constexpr float kMaxContourRatio = 0.95F;   // 枠いっぱいの背景を除く

/// 枠の中を二値化/エッジ抽出して輪郭を取り、把持に要る中心・角度・幅を出す。
struct GraspTarget
{
  cv::Point2f center;
  cv::Size2f size;
  float angle_deg = 0.0F;
  cv::RotatedRect rect;
};

bool ExtractBestGraspFromMask(
  const cv::Mat & binary, int x0, int y0, float roi_area, GraspTarget & out)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
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
  const float roi_area = static_cast<float>(roi.rows * roi.cols);

  // 1段目: Otsu 二値化（高コントラスト物体に強い）
  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  if (ExtractBestGraspFromMask(binary, x0, y0, roi_area, out)) {
    return true;
  }

  // 2段目: 低コントラストや未知背景向けに Canny エッジ + 膨張クロージングで補完
  cv::Mat edges;
  cv::Canny(gray, edges, 40.0, 120.0);
  cv::dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
  return ExtractBestGraspFromMask(edges, x0, y0, roi_area, out);
}

}  // namespace

class YoloGraspNode
{
public:
  YoloGraspNode(
    const rclcpp::Node::SharedPtr & node, const std::string & model, const std::string & device,
    float conf, float iou, const std::string & labels_csv, double display_fps,
    const std::string & image_topic, bool topic_input, double detection_fps,
    const std::string & frame_id, double output_fps, int annotated_width,
    int npu_workers, bool tracker_enabled, int tracker_min_hits, int tracker_max_missed,
    bool class_agnostic_grasp_fallback, const std::string & fastsam_model)
  : node_(node), device_(device), conf_(conf), iou_(iou), topic_input_(topic_input),
    frame_id_(frame_id), display_period_(display_fps > 0.0 ? 1.0 / display_fps : 0.0),
    output_period_(output_fps > 0.0 ? 1.0 / output_fps : 0.0),
    annotated_width_(annotated_width),
    tracker_enabled_(tracker_enabled),
    class_agnostic_grasp_fallback_(class_agnostic_grasp_fallback),
    detector_(model, npu_workers), labels_(LoadLabels(labels_csv)),
    publish_gate_(detection_fps), output_gate_(output_fps), display_gate_(display_fps),
    tracker_(MakeTrackerConfig(tracker_min_hits, tracker_max_missed)),
    fastsam_(fastsam_model.empty() ? nullptr : std::make_unique<RknnFastSam>(fastsam_model))
  {
    if (!detector_.ok()) {
      throw std::runtime_error("RKNN モデルを読めません: " + model);
    }
    pub_detections_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
    // Web UI (VisionListener) の SensorDataQoS と揃え、古い描画フレームを溜めない
    pub_annotated_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "detection_image", rclcpp::SensorDataQoS());
    pub_grasp_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "grasp_image", rclcpp::SensorDataQoS());
    pub_target_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(
      "target", rclcpp::QoS(rclcpp::KeepLast(1)));
    srv_grasp_once_ = node_->create_service<std_srvs::srv::Trigger>(
      "grasp_once",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) { OnGraspOnce(res); });

    if (topic_input) {
      if (image_topic.empty()) {
        throw std::runtime_error("トピック入力を指定するには image_topic が必要です");
      }
      // 購読はメンバで保持する。ローカルに捨てると Jazzy では即破棄される (P12)。
      image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {OnImage(msg);});
      RCLCPP_INFO(node_->get_logger(),
        "NPU 検出を開始: 入力トピック %s, NPUワーカー %zu, ラベル %zu 件, トラッカ=%s",
        image_topic.c_str(), detector_.worker_count(), labels_.size(),
        tracker_enabled_ ? "on" : "off");
      return;
    }

    capture_.open(device_, cv::CAP_V4L2);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, kCaptureWidth);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, kCaptureHeight);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (!capture_.isOpened()) {
      throw std::runtime_error("カメラを開けません: " + device);
    }
    RCLCPP_INFO(node_->get_logger(),
      "NPU 検出を開始: 入力 %d, NPUワーカー %zu, カメラ %dx%d, 出力 %.1fHz%s, ラベル %zu 件",
      detector_.input_size(), detector_.worker_count(), kCaptureWidth, kCaptureHeight,
      output_period_ > 0.0 ? 1.0 / output_period_ : 0.0,
      output_period_ > 0.0 ? " 固定" : " (カメラ同期)", labels_.size());
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
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void Run()
  {
    // RK3588 の 3 NPU コアを並列稼働させるワーカー群を起動する。
    const size_t n_workers = detector_.worker_count();
    std::vector<std::thread> infer_workers;
    infer_workers.reserve(n_workers);
    for (size_t w = 0; w < n_workers; ++w) {
      infer_workers.emplace_back([this, w] {InferLoop(w);});
    }

    std::thread publish;
    if (!topic_input_ && output_period_ > 0.0) {
      publish = std::thread([this] {PublishLoop();});
    }
    if (topic_input_) {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node_);
      executor.spin();
    } else {
      CaptureLoop();
    }
    stop_ = true;
    cond_.notify_all();
    if (publish.joinable()) {
      publish.join();
    }
    for (auto & th : infer_workers) {
      if (th.joinable()) {
        th.join();
      }
    }
    capture_.release();
  }

  /// アーム停止時ワンショット精密把持サービス（FastSAM 優先、未設定時は ROI 輪郭抽出）。
  void OnGraspOnce(std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    cv::Mat frame;
    std::vector<YoloBox> boxes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_.empty()) {
        res->success = false;
        res->message = "no frame available";
        return;
      }
      frame = latest_;
      boxes = boxes_;
    }

    cv::Rect roi(frame.cols / 4, frame.rows / 4, frame.cols / 2, frame.rows / 2);
    if (!boxes.empty()) {
      const auto & b = boxes.front();
      roi = cv::Rect(
        std::max(0, static_cast<int>(b.x0)),
        std::max(0, static_cast<int>(b.y0)),
        std::max(1, static_cast<int>(b.x1 - b.x0)),
        std::max(1, static_cast<int>(b.y1 - b.y0))) & cv::Rect(0, 0, frame.cols, frame.rows);
    }

    GraspTarget target{};
    bool ok = false;
    std::string mode = "contour";
    if (fastsam_ && fastsam_->ok()) {
      cv::Mat mask;
      if (fastsam_->SegmentBestMask(frame, roi, mask)) {
        const cv::Mat sub = mask(roi);
        ok = ExtractBestGraspFromMask(
          sub, roi.x, roi.y, static_cast<float>(roi.area()), target);
        if (ok) {
          mode = "fastsam";
        }
      }
    }
    if (!ok) {
      YoloBox roi_box{
        static_cast<float>(roi.x), static_cast<float>(roi.y),
        static_cast<float>(roi.x + roi.width), static_cast<float>(roi.y + roi.height),
        1.0F, boxes.empty() ? -1 : boxes.front().class_id};
      ok = ComputeGrasp(frame, roi_box, target);
    }

    if (!ok) {
      res->success = false;
      res->message = "grasp contour not found";
      return;
    }
    char msg[160];
    std::snprintf(
      msg, sizeof(msg), "mode=%s cx=%.1f cy=%.1f angle_deg=%.1f width_px=%.1f",
      mode.c_str(), target.center.x, target.center.y, target.angle_deg, target.size.width);
    res->success = true;
    res->message = msg;
  }

  void PublishLoop()
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cond_.wait(lock, [this] {return !latest_.empty() || stop_;});
    }
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(output_period_));
    auto next_tick = std::chrono::steady_clock::now();
    while (!stop_) {
      next_tick += period;
      std::this_thread::sleep_until(next_tick);
      if (stop_) {
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - next_tick > period) {
        next_tick = now;
      }
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_.empty()) {
          continue;
        }
        // cv::Mat の参照カウント共有で 921KB/frame の不要コピーを排除
        frame = latest_;
      }
      ComposeAndPublish(frame);
    }
  }

  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      cv::Mat bgr = BgrFromImageMsg(*msg);
      const rclcpp::Time stamp(msg->header.stamp);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(bgr);
        latest_stamp_ = stamp;
        ++latest_seq_;
        has_frame_ = true;
      }
      cond_.notify_one();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "画像を捨てます: %s", e.what());
    }
  }

private:
  static TrackerConfig MakeTrackerConfig(int min_hits, int max_missed)
  {
    TrackerConfig cfg{};
    cfg.min_hits = std::max(1, min_hits);
    cfg.max_missed = std::max(0, max_missed);
    return cfg;
  }

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
        ++latest_seq_;
        has_frame_ = true;
      }
      cond_.notify_one();

      if (output_period_ > 0.0) {
        continue;
      }

      const auto now = std::chrono::steady_clock::now();
      const double since_publish =
        std::chrono::duration<double>(now - last_publish_).count();
      if (display_period_ > 0.0 && since_publish < display_period_) {
        continue;
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

  std::vector<YoloBox> ApplyTracking(const std::vector<YoloBox> & raw_boxes)
  {
    if (!tracker_enabled_) {
      return raw_boxes;
    }
    std::vector<TrackInputBox> inputs;
    inputs.reserve(raw_boxes.size());
    for (const auto & b : raw_boxes) {
      inputs.push_back({b.x0, b.y0, b.x1, b.y1, b.score, b.class_id});
    }
    std::vector<TrackedBox> tracked;
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      tracked = tracker_.Update(inputs);
    }
    std::vector<YoloBox> out;
    out.reserve(tracked.size());
    for (const auto & t : tracked) {
      out.push_back({t.x0, t.y0, t.x1, t.y1, t.score, t.class_id});
    }
    return out;
  }

  /// RK3588 NPU ワーカー 1 本分。最新フレームのみを処理し、古い結果の追い越し書き戻しを防ぐ。
  void InferLoop(size_t worker_idx)
  {
    while (!stop_) {
      cv::Mat frame;
      rclcpp::Time stamp;
      uint64_t seq = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] {return has_frame_ || stop_;});
        if (stop_) {
          break;
        }
        frame = latest_;
        stamp = latest_stamp_;
        seq = latest_seq_;
        has_frame_ = false;
      }
      try {
        auto raw_boxes = detector_.DetectOnWorker(worker_idx, frame, conf_, iou_);
        // 並列 NPU ワーカー間で古いフレームの結果が新しい結果を上書きするのを防ぐ
        if (seq < last_committed_seq_.load(std::memory_order_acquire)) {
          continue;
        }
        auto boxes = ApplyTracking(raw_boxes);

        // 未知物体（COCO外）でも手先把持を継続できるよう、/grasp_image 購読時かつ
        // 検出 0 件なら画面中央 ROI でクラス非依存の輪郭把持を試行する。
        std::vector<GraspTarget> grasps(boxes.size());
        std::vector<char> valid(boxes.size(), 0);
        if (pub_grasp_->get_subscription_count() > 0) {
          if (!boxes.empty()) {
            for (size_t i = 0; i < boxes.size(); ++i) {
              valid[i] = ComputeGrasp(frame, boxes[i], grasps[i]) ? 1 : 0;
            }
          } else if (class_agnostic_grasp_fallback_) {
            YoloBox center_roi{};
            center_roi.x0 = static_cast<float>(frame.cols) * 0.25F;
            center_roi.y0 = static_cast<float>(frame.rows) * 0.25F;
            center_roi.x1 = static_cast<float>(frame.cols) * 0.75F;
            center_roi.y1 = static_cast<float>(frame.rows) * 0.75F;
            center_roi.score = 0.50F;
            center_roi.class_id = -1;
            GraspTarget fallback{};
            if (ComputeGrasp(frame, center_roi, fallback)) {
              boxes.push_back(center_roi);
              grasps.push_back(fallback);
              valid.push_back(1);
            }
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (seq < last_committed_seq_.load(std::memory_order_relaxed)) {
            continue;
          }
          last_committed_seq_.store(seq, std::memory_order_release);
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
      if (i >= valid.size() || !valid[i]) {
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
    std::snprintf(header, sizeof(header), "infer %.1f ms  det %zu  npu %zu",
      detector_.last_infer_ms(), boxes.size(), detector_.worker_count());
    cv::putText(canvas, header, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(255, 255, 255), 2);
    return canvas;
  }

  void ComposeAndPublish(const cv::Mat & frame)
  {
    const auto stamp = node_->now();
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    // /detections は 25 Hz 維持しつつ、描画画像だけ display_fps (12〜15 Hz) に独立間引きする
    if ((pub_annotated_->get_subscription_count() > 0 ||
         pub_grasp_->get_subscription_count() > 0) &&
        display_gate_.Ready(now_s))
    {
      const cv::Mat canvas = Compose(frame);
      Publish(canvas, stamp);
    }
    std::vector<YoloBox> boxes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      boxes = boxes_;
    }
    PublishDetections(stamp, boxes);
  }

  void PublishTopicResults(
    const cv::Mat & frame, const rclcpp::Time & stamp, const std::vector<YoloBox> & boxes)
  {
    PublishDetections(stamp, boxes);
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if ((pub_annotated_->get_subscription_count() > 0 || pub_grasp_->get_subscription_count() > 0) &&
        display_gate_.Ready(now_s))
    {
      Publish(Compose(frame), stamp);
    }
  }

  std::string LabelOf(int class_id) const
  {
    if (class_id < 0) {
      return "unknown_object";
    }
    if (static_cast<size_t>(class_id) < labels_.size()) {
      return labels_[static_cast<size_t>(class_id)];
    }
    return std::to_string(class_id);
  }

  void Publish(const cv::Mat & canvas, const rclcpp::Time & stamp)
  {
    cv::Mat out = canvas;
    if (annotated_width_ > 0 && canvas.cols > annotated_width_) {
      const double scale = static_cast<double>(annotated_width_) / canvas.cols;
      cv::resize(canvas, out,
        cv::Size(annotated_width_, std::max(1, static_cast<int>(canvas.rows * scale))));
    }
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp = stamp;
    message->header.frame_id = frame_id_;
    message->height = static_cast<uint32_t>(out.rows);
    message->width = static_cast<uint32_t>(out.cols);
    message->encoding = "bgr8";
    message->is_bigendian = 0;
    message->step = static_cast<uint32_t>(out.cols * 3);
    message->data.resize(static_cast<size_t>(message->step) * message->height);
    std::memcpy(message->data.data(), out.data, message->data.size());
    if (pub_grasp_->get_subscription_count() > 0) {
      pub_grasp_->publish(*message);
    }
    pub_annotated_->publish(std::move(message));
  }

  void PublishDetections(
    const rclcpp::Time & stamp, const std::vector<YoloBox> & boxes)
  {
    vision_msgs::msg::Detection2DArray message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    std::lock_guard<std::mutex> lock(filter_mutex_);
    for (size_t i = 0; i < boxes.size(); ++i) {
      DetectionIdentity identity;
      if (tracker_enabled_) {
        identity.class_id = boxes[i].class_id;
        identity.class_name = LabelOf(boxes[i].class_id);
        identity.score = boxes[i].score;
        identity.x0 = boxes[i].x0;
        identity.y0 = boxes[i].y0;
        identity.x1 = boxes[i].x1;
        identity.y1 = boxes[i].y1;
      } else {
        const SmoothedBox measured{static_cast<double>(boxes[i].x0),
                                   static_cast<double>(boxes[i].y0),
                                   static_cast<double>(boxes[i].x1),
                                   static_cast<double>(boxes[i].y1),
                                   static_cast<double>(boxes[i].score),
                                   boxes[i].class_id};
        const auto smooth = center_filter_.Update(measured);
        identity.class_id = smooth.class_id;
        identity.class_name = LabelOf(smooth.class_id);
        identity.score = smooth.score;
        identity.x0 = smooth.x0;
        identity.y0 = smooth.y0;
        identity.x1 = smooth.x1;
        identity.y1 = smooth.y1;
      }
      vision_msgs::msg::Detection2D detection;
      FillDetection2D(identity, detection);
      message.detections.push_back(detection);
    }
    pub_detections_->publish(message);
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
  double output_period_;
  int annotated_width_;
  bool tracker_enabled_;
  bool class_agnostic_grasp_fallback_;
  RknnYoloPool detector_;
  std::vector<std::string> labels_;
  PublishRateGate publish_gate_;
  PublishRateGate output_gate_;
  PublishRateGate display_gate_;
  BoxCenterFilter center_filter_;
  std::mutex filter_mutex_;
  MultiObjectTracker tracker_;
  std::mutex tracker_mutex_;
  std::unique_ptr<RknnFastSam> fastsam_;

  cv::VideoCapture capture_;
  std::mutex mutex_;
  std::condition_variable cond_;
  cv::Mat latest_;
  rclcpp::Time latest_stamp_{};
  uint64_t latest_seq_ = 0;
  std::atomic<uint64_t> last_committed_seq_{0};
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_grasp_once_;
};

}  // namespace umeonigiri_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yolo_grasp");
  const std::string model = node->declare_parameter<std::string>("model", "");
  const std::string labels = node->declare_parameter<std::string>("labels", "");
  const std::string device = node->declare_parameter<std::string>("device", "/dev/video0");
  const double conf = node->declare_parameter<double>("conf", 0.35);
  const double iou = node->declare_parameter<double>("iou", 0.45);
  const double display_fps = node->declare_parameter<double>("display_fps", 15.0);
  const double detection_fps = node->declare_parameter<double>("detection_fps", 25.0);
  const std::string image = node->declare_parameter<std::string>("image", "");
  const std::string image_topic = node->declare_parameter<std::string>("image_topic", "");
  const bool topic_input = node->declare_parameter<bool>("use_topic_input", false);
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "camera_link");
  const double output_fps = node->declare_parameter<double>("output_fps", 25.0);
  const int annotated_width = node->declare_parameter<int>("annotated_width", 320);
  const int npu_workers = node->declare_parameter<int>("npu_workers", 3);
  const int opencv_threads = node->declare_parameter<int>("opencv_threads", 2);
  const bool tracker_enabled = node->declare_parameter<bool>("tracker_enabled", true);
  const int tracker_min_hits = node->declare_parameter<int>("tracker_min_hits", 2);
  const int tracker_max_missed = node->declare_parameter<int>("tracker_max_missed", 3);
  const bool class_agnostic_grasp_fallback =
    node->declare_parameter<bool>("class_agnostic_grasp_fallback", true);
  const std::string fastsam_model =
    node->declare_parameter<std::string>("fastsam_model", "");

  // RK3588 上で OpenCV が全 8 コアにスレッドを広げて Nav2/SLAM を圧迫するのを防ぐ
  if (opencv_threads > 0) {
    cv::setNumThreads(opencv_threads);
  }

  try {
    auto grasp = std::make_shared<umeonigiri_vision::YoloGraspNode>(
      node, model, device, static_cast<float>(conf), static_cast<float>(iou), labels, display_fps,
      image_topic, topic_input, detection_fps, frame_id, output_fps, annotated_width,
      npu_workers, tracker_enabled, tracker_min_hits, tracker_max_missed,
      class_agnostic_grasp_fallback, fastsam_model);
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
