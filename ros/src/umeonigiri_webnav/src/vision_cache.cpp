// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/vision_cache.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace umeonigiri_webnav {
namespace {

constexpr double kFpsWindowSeconds = 2.0;
constexpr std::size_t kMaxDetections = 12;
constexpr int kJpegQuality = 80;

int ChannelsOf(const std::string& encoding) {
  if (encoding == "mono8") {
    return 1;
  }
  if (encoding == "bgr8" || encoding == "rgb8") {
    return 3;
  }
  if (encoding == "bgra8" || encoding == "rgba8") {
    return 4;
  }
  return 0;
}

bool ToBgrOwned(const sensor_msgs::msg::Image& msg, cv::Mat& bgr) {
  const int channels = ChannelsOf(msg.encoding);
  if (channels == 0 || msg.width == 0 || msg.height == 0) {
    return false;
  }
  if (msg.step < static_cast<std::uint32_t>(msg.width) * channels ||
      msg.data.size() < static_cast<std::size_t>(msg.step) * msg.height) {
    return false;
  }
  const int type = channels == 1 ? CV_8UC1 : (channels == 3 ? CV_8UC3 : CV_8UC4);
  const cv::Mat view(static_cast<int>(msg.height), static_cast<int>(msg.width), type,
                     const_cast<unsigned char*>(msg.data.data()), msg.step);
  if (msg.encoding == "bgr8") {
    // msg の寿命が尽きても安全に遅延 JPEG エンコードできるよう独立バッファへコピーする
    bgr = view.clone();
  } else if (msg.encoding == "rgb8") {
    cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
  } else if (msg.encoding == "bgra8") {
    cv::cvtColor(view, bgr, cv::COLOR_BGRA2BGR);
  } else if (msg.encoding == "rgba8") {
    cv::cvtColor(view, bgr, cv::COLOR_RGBA2BGR);
  } else {
    cv::cvtColor(view, bgr, cv::COLOR_GRAY2BGR);
  }
  return true;
}

double Round(double value, int digits) {
  const double scale = std::pow(10.0, digits);
  return std::round(value * scale) / scale;
}

int AgeMs(const std::chrono::steady_clock::time_point& now,
          const std::chrono::steady_clock::time_point& then) {
  return static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count());
}

}  // namespace

VisionCache::VisionCache(int stale_after_ms) : stale_after_ms_(stale_after_ms) {}

bool VisionCache::SetFrame(const sensor_msgs::msg::Image& msg) {
  cv::Mat bgr;
  if (!ToBgrOwned(msg, bgr)) {
    return false;
  }
  const Clock::time_point now = Clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  latest_bgr_ = std::move(bgr);
  width_ = static_cast<int>(msg.width);
  height_ = static_cast<int>(msg.height);
  frame_id_ = msg.header.frame_id;
  frame_at_ = now;
  has_frame_ = true;
  ++seq_;
  arrivals_.push_back(now);
  const Clock::time_point cutoff =
      now - std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(kFpsWindowSeconds));
  while (arrivals_.size() > 1 && arrivals_.front() < cutoff) {
    arrivals_.pop_front();
  }
  frame_cv_.notify_all();
  return true;
}

void VisionCache::SetDetections(const vision_msgs::msg::Detection2DArray& msg) {
  std::vector<VisionDetection> parsed;
  parsed.reserve(msg.detections.size());
  for (const vision_msgs::msg::Detection2D& detection : msg.detections) {
    VisionDetection entry;
    entry.label = detection.id.empty() ? "unknown" : detection.id;
    entry.score = detection.results.empty() ? 0.0 : detection.results.front().hypothesis.score;
    entry.x = detection.bbox.center.position.x;
    entry.y = detection.bbox.center.position.y;
    entry.width = detection.bbox.size_x;
    entry.height = detection.bbox.size_y;
    parsed.push_back(std::move(entry));
  }
  const std::size_t raw_count = parsed.size();
  std::sort(parsed.begin(), parsed.end(),
            [](const VisionDetection& a, const VisionDetection& b) {
              return a.score > b.score;
            });
  if (parsed.size() > kMaxDetections) {
    parsed.resize(kMaxDetections);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  detections_ = std::move(parsed);
  detection_count_ = raw_count;
  detections_at_ = Clock::now();
  has_detections_ = true;
}

nlohmann::json VisionCache::Status() const {
  const Clock::time_point now = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  double fps = 0.0;
  if (arrivals_.size() >= 2) {
    const double span =
        std::chrono::duration<double>(arrivals_.back() - arrivals_.front()).count();
    if (span > 0.0) {
      fps = static_cast<double>(arrivals_.size() - 1) / span;
    }
  }

  nlohmann::json payload;
  payload["has_frame"] = has_frame_;
  payload["seq"] = seq_;
  payload["fps"] = Round(fps, 1);
  payload["stale"] = has_frame_ && AgeMs(now, frame_at_) > stale_after_ms_;
  payload["frame_age_ms"] = has_frame_ ? nlohmann::json(AgeMs(now, frame_at_))
                                       : nlohmann::json(nullptr);
  payload["width"] = has_frame_ ? nlohmann::json(width_) : nlohmann::json(nullptr);
  payload["height"] = has_frame_ ? nlohmann::json(height_) : nlohmann::json(nullptr);
  payload["frame_id"] = has_frame_ ? nlohmann::json(frame_id_) : nlohmann::json(nullptr);
  payload["detection_count"] = detection_count_;
  payload["detection_age_ms"] =
      has_detections_ ? nlohmann::json(AgeMs(now, detections_at_)) : nlohmann::json(nullptr);

  nlohmann::json list = nlohmann::json::array();
  for (const VisionDetection& detection : detections_) {
    list.push_back(nlohmann::json{{"label", detection.label},
                                  {"score", Round(detection.score, 3)},
                                  {"x", Round(detection.x, 1)},
                                  {"y", Round(detection.y, 1)},
                                  {"width", Round(detection.width, 1)},
                                  {"height", Round(detection.height, 1)}});
  }
  payload["detections"] = std::move(list);
  return payload;
}

std::optional<VisionFrame> VisionCache::Latest(long after_seq, double timeout_s) {
  const auto ready = [this, after_seq] { return has_frame_ && seq_ > after_seq; };
  cv::Mat bgr_to_encode;
  long target_seq = 0;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ready() && timeout_s > 0.0) {
      frame_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), ready);
    }
    if (!ready()) {
      return std::nullopt;
    }
    if (jpeg_seq_ == seq_) {
      return VisionFrame{seq_, jpeg_};
    }
    // 参照カウント共有のままロック外で JPEG エンコードし、受信スレッドを止めない
    bgr_to_encode = latest_bgr_;
    target_seq = seq_;
  }

  std::vector<uchar> encoded;
  if (!cv::imencode(".jpg", bgr_to_encode, encoded, {cv::IMWRITE_JPEG_QUALITY, kJpegQuality})) {
    return std::nullopt;
  }
  std::string jpeg(reinterpret_cast<const char*>(encoded.data()), encoded.size());

  std::lock_guard<std::mutex> lock(mutex_);
  if (seq_ == target_seq) {
    jpeg_ = jpeg;
    jpeg_seq_ = target_seq;
  }
  return VisionFrame{target_seq, std::move(jpeg)};
}

}  // namespace umeonigiri_webnav
