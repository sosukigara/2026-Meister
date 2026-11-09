// SPDX-License-Identifier: MIT
/// VisionCache のテスト (ROS グラフ不要)。
///
/// 見ているのはブラウザに何が渡るか: JPEG が本当に画像として復号できること、
/// 復号できない入力は混ぜないこと、検出の対応付け (id → label) が崩れないこと。
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "meister_web_nav/vision_cache.hpp"

namespace meister_web_nav {
namespace {

using json = nlohmann::json;

int ChannelsOf(const std::string& encoding) {
  if (encoding == "mono8") {
    return 1;
  }
  if (encoding == "bgra8" || encoding == "rgba8") {
    return 4;
  }
  return 3;
}

sensor_msgs::msg::Image MakeImage(int width = 64, int height = 48,
                                  const std::string& encoding = "bgr8") {
  const int channels = ChannelsOf(encoding);
  sensor_msgs::msg::Image msg;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.encoding = encoding;
  msg.step = static_cast<uint32_t>(width) * channels;
  msg.data.assign(static_cast<std::size_t>(msg.step) * height, 0);
  // 単色だと JPEG が数バイトに潰れて「符号化できたか」を確かめられない。
  for (int r = height / 4; r < height / 2; ++r) {
    for (int c = width / 4; c < width / 2; ++c) {
      const std::size_t offset =
          static_cast<std::size_t>(r) * msg.step + static_cast<std::size_t>(c) * channels;
      for (int channel = 0; channel < channels; ++channel) {
        msg.data[offset + channel] = 255;
      }
    }
  }
  msg.header.frame_id = "camera";
  return msg;
}

vision_msgs::msg::Detection2D MakeDetection(const std::string& label, double score,
                                            double x, double y, double width,
                                            double height) {
  vision_msgs::msg::Detection2D detection;
  detection.id = label;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id = "0";
  hypothesis.hypothesis.score = score;
  detection.results.push_back(hypothesis);
  detection.bbox.center.position.x = x;
  detection.bbox.center.position.y = y;
  detection.bbox.size_x = width;
  detection.bbox.size_y = height;
  return detection;
}

vision_msgs::msg::Detection2DArray MakeArray(
    const std::vector<vision_msgs::msg::Detection2D>& detections) {
  vision_msgs::msg::Detection2DArray array;
  array.detections = detections;
  return array;
}

std::vector<std::string> KeysOf(const json& object) {
  std::vector<std::string> keys;
  for (auto it = object.begin(); it != object.end(); ++it) {
    keys.push_back(it.key());
  }
  std::sort(keys.begin(), keys.end());
  return keys;
}

TEST(VisionCacheStatus, EmptyCacheReportsNoFrame) {
  VisionCache cache;

  const json status = cache.Status();

  EXPECT_FALSE(status.at("has_frame").get<bool>());
  EXPECT_FALSE(status.at("stale").get<bool>());
  EXPECT_TRUE(status.at("frame_age_ms").is_null());
  EXPECT_TRUE(status.at("width").is_null());
  EXPECT_TRUE(status.at("frame_id").is_null());
  EXPECT_EQ(status.at("seq").get<long>(), 0);
  EXPECT_EQ(status.at("detection_count").get<std::size_t>(), 0u);
  EXPECT_TRUE(status.at("detections").empty());
}

TEST(VisionCacheStatus, KeySetIsTheFrontendContract) {
  // app.js が読むキー。増やすのは自由だが、消すと UI が黙って壊れる。
  VisionCache cache;

  EXPECT_EQ(KeysOf(cache.Status()),
            (std::vector<std::string>{"detection_age_ms", "detection_count", "detections",
                                      "fps", "frame_age_ms", "frame_id", "has_frame",
                                      "height", "seq", "stale", "width"}));
}

TEST(VisionCacheFrame, StoresShapeAndFrameId) {
  VisionCache cache;

  ASSERT_TRUE(cache.SetFrame(MakeImage()));

  const json status = cache.Status();
  EXPECT_TRUE(status.at("has_frame").get<bool>());
  EXPECT_EQ(status.at("width").get<int>(), 64);
  EXPECT_EQ(status.at("height").get<int>(), 48);
  EXPECT_EQ(status.at("frame_id").get<std::string>(), "camera");
  EXPECT_EQ(status.at("seq").get<long>(), 1);
  EXPECT_FALSE(status.at("stale").get<bool>());
  EXPECT_GE(status.at("frame_age_ms").get<int>(), 0);
  // 1 フレームでは間隔が測れないので 0。
  EXPECT_DOUBLE_EQ(status.at("fps").get<double>(), 0.0);
}

TEST(VisionCacheFrame, JpegDecodesBackToTheSameSize) {
  VisionCache cache;
  ASSERT_TRUE(cache.SetFrame(MakeImage(80, 60)));

  const std::optional<VisionFrame> frame = cache.Latest(-1, 0.0);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->seq, 1);
  ASSERT_GE(frame->jpeg.size(), 4u);
  EXPECT_EQ(frame->jpeg.compare(0, 2, "\xff\xd8"), 0);  // JPEG SOI

  const std::vector<uchar> bytes(frame->jpeg.begin(), frame->jpeg.end());
  const cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, 80);
  EXPECT_EQ(decoded.rows, 60);
}

TEST(VisionCacheFrame, MonoAndAlphaEncodingsAreAccepted) {
  VisionCache cache;

  EXPECT_TRUE(cache.SetFrame(MakeImage(32, 32, "mono8")));
  EXPECT_TRUE(cache.SetFrame(MakeImage(32, 32, "rgba8")));
  EXPECT_EQ(cache.Status().at("seq").get<long>(), 2);
}

TEST(VisionCacheFrame, UnknownEncodingIsRejected) {
  VisionCache cache;

  // mono16 は cv_bridge でも読めるが、8bit に潰す経路は絵が変わるので受けない。
  EXPECT_FALSE(cache.SetFrame(MakeImage(32, 32, "mono16")));
  EXPECT_FALSE(cache.Status().at("has_frame").get<bool>());
}

TEST(VisionCacheFrame, BrokenStepIsRejected) {
  VisionCache cache;
  sensor_msgs::msg::Image msg = MakeImage(32, 32);
  msg.step = 32;  // bgr8 なら 96 以上必要

  EXPECT_FALSE(cache.SetFrame(msg));
  EXPECT_FALSE(cache.Status().at("has_frame").get<bool>());
}

TEST(VisionCacheFrame, ShortBufferIsRejected) {
  VisionCache cache;
  sensor_msgs::msg::Image msg = MakeImage(32, 32);
  msg.data.resize(static_cast<std::size_t>(msg.step) * 16);  // height に足りない

  EXPECT_FALSE(cache.SetFrame(msg));
}

TEST(VisionCacheFrame, StaleFlagFlipsAfterTheThreshold) {
  VisionCache cache(30);
  ASSERT_TRUE(cache.SetFrame(MakeImage()));

  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  const json status = cache.Status();
  EXPECT_TRUE(status.at("stale").get<bool>());
  EXPECT_GE(status.at("frame_age_ms").get<int>(), 30);
}

TEST(VisionCacheDetections, MapsIdToLabelAndBboxToPixels) {
  VisionCache cache;

  cache.SetDetections(MakeArray({MakeDetection("person", 0.9, 10.0, 20.0, 4.0, 6.0),
                                 MakeDetection("cup", 0.5, 1.0, 2.0, 3.0, 4.0)}));

  const json status = cache.Status();
  EXPECT_EQ(status.at("detection_count").get<std::size_t>(), 2u);
  ASSERT_EQ(status.at("detections").size(), 2u);
  const json& first = status.at("detections").at(0);
  EXPECT_EQ(first.at("label").get<std::string>(), "person");
  EXPECT_DOUBLE_EQ(first.at("score").get<double>(), 0.9);
  EXPECT_DOUBLE_EQ(first.at("x").get<double>(), 10.0);
  EXPECT_DOUBLE_EQ(first.at("y").get<double>(), 20.0);
  EXPECT_DOUBLE_EQ(first.at("width").get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(first.at("height").get<double>(), 6.0);
}

TEST(VisionCacheDetections, SortedByScoreAndBounded) {
  VisionCache cache;
  std::vector<vision_msgs::msg::Detection2D> detections;
  for (int i = 0; i < 20; ++i) {
    detections.push_back(
        MakeDetection("class" + std::to_string(i), 0.01 * i, 0.0, 0.0, 1.0, 1.0));
  }

  cache.SetDetections(MakeArray(detections));

  const json status = cache.Status();
  EXPECT_EQ(status.at("detection_count").get<std::size_t>(), 20u);
  EXPECT_EQ(status.at("detections").size(), 12u);  // 上限
  EXPECT_EQ(status.at("detections").at(0).at("label").get<std::string>(), "class19");
}

TEST(VisionCacheLatest, WaitsForTheNextFrame) {
  VisionCache cache;
  ASSERT_TRUE(cache.SetFrame(MakeImage()));

  // 既に持っているフレームは after_seq と一致するので、新しいものが来るまで待つ。
  EXPECT_FALSE(cache.Latest(1, 0.05).has_value());

  std::thread publisher([&cache] {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    cache.SetFrame(MakeImage());
  });
  const std::optional<VisionFrame> frame = cache.Latest(1, 2.0);
  publisher.join();

  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->seq, 2);
}

TEST(VisionCacheLatest, SecondFrameRaisesFps) {
  VisionCache cache;
  ASSERT_TRUE(cache.SetFrame(MakeImage()));
  ASSERT_TRUE(cache.SetFrame(MakeImage()));

  EXPECT_GT(cache.Status().at("fps").get<double>(), 0.0);
}

}  // namespace
}  // namespace meister_web_nav
