// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "meister_vision/draw.hpp"
#include "meister_vision/image_util.hpp"

namespace meister_vision {
namespace {

sensor_msgs::msg::Image MakeImage(const std::string& encoding, uint32_t w,
                                  uint32_t h, size_t line_step,
                                  std::vector<uint8_t> data) {
  sensor_msgs::msg::Image msg;
  msg.encoding = encoding;
  msg.width = w;
  msg.height = h;
  msg.step = static_cast<uint32_t>(line_step);
  msg.data = std::move(data);
  return msg;
}

TEST(BgrFromImageMsg, PassesBgr8Through) {
  const cv::Mat got =
      BgrFromImageMsg(MakeImage("bgr8", 2, 1, 6, {1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(got.type(), CV_8UC3);
  ASSERT_EQ(got.cols, 2);
  EXPECT_EQ(got.at<cv::Vec3b>(0, 0), cv::Vec3b(1, 2, 3));
  EXPECT_EQ(got.at<cv::Vec3b>(0, 1), cv::Vec3b(4, 5, 6));
}

TEST(BgrFromImageMsg, SwapsRgb8ToBgr) {
  const cv::Mat got = BgrFromImageMsg(MakeImage("rgb8", 1, 1, 3, {1, 2, 3}));
  ASSERT_EQ(got.type(), CV_8UC3);
  EXPECT_EQ(got.at<cv::Vec3b>(0, 0), cv::Vec3b(3, 2, 1));
}

TEST(BgrFromImageMsg, ExpandsMono8ToThreeChannels) {
  const cv::Mat got = BgrFromImageMsg(MakeImage("mono8", 2, 1, 2, {10, 20}));
  ASSERT_EQ(got.type(), CV_8UC3);
  EXPECT_EQ(got.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 10, 10));
  EXPECT_EQ(got.at<cv::Vec3b>(0, 1), cv::Vec3b(20, 20, 20));
}

TEST(BgrFromImageMsg, DropsAlphaForRgba) {
  const cv::Mat got =
      BgrFromImageMsg(MakeImage("rgba8", 1, 1, 4, {1, 2, 3, 255}));
  ASSERT_EQ(got.type(), CV_8UC3);
  EXPECT_EQ(got.at<cv::Vec3b>(0, 0), cv::Vec3b(3, 2, 1));
}

TEST(BgrFromImageMsg, KeepsBgraOrder) {
  const cv::Mat got =
      BgrFromImageMsg(MakeImage("bgra8", 1, 1, 4, {1, 2, 3, 255}));
  ASSERT_EQ(got.type(), CV_8UC3);
  EXPECT_EQ(got.at<cv::Vec3b>(0, 0), cv::Vec3b(1, 2, 3));
}

TEST(BgrFromImageMsg, ConvertsYuyv422) {
  const cv::Mat got =
      BgrFromImageMsg(MakeImage("yuv422_yuy2", 2, 1, 4, {128, 0, 128, 255}));
  ASSERT_EQ(got.type(), CV_8UC3);
  ASSERT_EQ(got.cols, 2);
}

TEST(BgrFromImageMsg, KeepsSingleChannelDepth) {
  const cv::Mat u16 =
      BgrFromImageMsg(MakeImage("16UC1", 1, 1, 2, {0x01, 0x02}));
  ASSERT_EQ(u16.type(), CV_16UC1);
  EXPECT_EQ(u16.at<uint16_t>(0, 0), 0x0201);

  const cv::Mat f = BgrFromImageMsg(
      MakeImage("32FC1", 1, 1, 4, {0, 0, 0x80, 0x3f}));
  ASSERT_EQ(f.type(), CV_32FC1);
  EXPECT_FLOAT_EQ(f.at<float>(0, 0), 1.0f);
}

TEST(BgrFromImageMsg, RejectsUnknownEncoding) {
  EXPECT_THROW(BgrFromImageMsg(MakeImage("weird9", 1, 1, 1, {0})),
               ImageConversionError);
}

TEST(BgrFromImageMsg, RejectsShortStep) {
  EXPECT_THROW(BgrFromImageMsg(MakeImage("bgr8", 4, 1, 3, {0, 0, 0})),
               ImageConversionError);
}

TEST(BgrFromImageMsg, RejectsTruncatedData) {
  EXPECT_THROW(BgrFromImageMsg(MakeImage("bgr8", 2, 2, 6, {0, 0})),
               ImageConversionError);
}

TEST(BgrFromImageMsg, RejectsZeroSize) {
  EXPECT_THROW(BgrFromImageMsg(MakeImage("bgr8", 0, 0, 0, {})),
               ImageConversionError);
}

TEST(BgrFromImageMsg, HonorsRowPadding) {
  // step 8 > 幅 2 x 3。行ごとに 2 バイトのパディングがある画像。
  const cv::Mat got = BgrFromImageMsg(
      MakeImage("bgr8", 2, 2, 8, {1, 2, 3, 4, 5, 6, 9, 9, 7, 8, 9, 10, 11, 12, 9, 9}));
  ASSERT_EQ(got.rows, 2);
  EXPECT_EQ(got.at<cv::Vec3b>(0, 0), cv::Vec3b(1, 2, 3));
  EXPECT_EQ(got.at<cv::Vec3b>(0, 1), cv::Vec3b(4, 5, 6));
  EXPECT_EQ(got.at<cv::Vec3b>(1, 0), cv::Vec3b(7, 8, 9));
  EXPECT_EQ(got.at<cv::Vec3b>(1, 1), cv::Vec3b(10, 11, 12));
}

TEST(ImageMsgFromBgr, RoundTripsPixels) {
  const cv::Mat src(2, 2, CV_8UC3, cv::Scalar(7, 8, 9));
  const sensor_msgs::msg::Image msg = ImageMsgFromBgr(src);
  EXPECT_EQ(msg.encoding, "bgr8");
  EXPECT_EQ(msg.width, 2U);
  EXPECT_EQ(msg.height, 2U);
  EXPECT_EQ(msg.step, 6U);
  EXPECT_EQ(msg.data.size(), 12U);
  EXPECT_EQ(msg.data[0], 7);
  EXPECT_EQ(msg.data[2], 9);
  EXPECT_EQ(BgrFromImageMsg(msg).at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
}

TEST(ImageMsgFromBgr, RejectsNonBgr) {
  EXPECT_THROW(ImageMsgFromBgr(cv::Mat(2, 2, CV_8UC1, cv::Scalar::all(1))),
               ImageConversionError);
}

TEST(ImageMsgFromBgr, MakesContiguousCopy) {
  const cv::Mat big(4, 8, CV_8UC3, cv::Scalar::all(1));
  const cv::Mat view = big(cv::Rect(0, 0, 3, 4));
  const sensor_msgs::msg::Image msg = ImageMsgFromBgr(view);
  EXPECT_EQ(msg.width, 3U);
  EXPECT_EQ(msg.height, 4U);
  EXPECT_EQ(msg.step, 9U);
  EXPECT_EQ(msg.data.size(), 36U);
}

TEST(DrawDetections, LeavesInputUntouched) {
  const cv::Mat src = cv::Mat::zeros(40, 40, CV_8UC3);
  std::vector<Detection> dets(1);
  dets[0].class_name = "cat";
  dets[0].confidence = 0.5f;
  dets[0].x1 = 5.0f;
  dets[0].y1 = 10.0f;
  dets[0].x2 = 30.0f;
  dets[0].y2 = 35.0f;
  const cv::Mat out = DrawDetections(src, dets);
  EXPECT_EQ(cv::countNonZero(src.reshape(1)), 0);
  EXPECT_GT(cv::countNonZero(out.reshape(1)), 0);
}

TEST(DrawDetections, ClampsOutOfRangeBoxes) {
  const cv::Mat src = cv::Mat::zeros(32, 32, CV_8UC3);
  std::vector<Detection> dets(1);
  dets[0].class_name = "dog";
  dets[0].x1 = -50.0f;
  dets[0].y1 = -50.0f;
  dets[0].x2 = 500.0f;
  dets[0].y2 = 500.0f;
  EXPECT_EQ(DrawDetections(src, dets).size(), src.size());
}

TEST(DrawDetections, NoDetectionsIsPlainCopy) {
  const cv::Mat src = cv::Mat::zeros(16, 16, CV_8UC3);
  EXPECT_EQ(cv::countNonZero(DrawDetections(src, {}).reshape(1)), 0);
}

}  // namespace
}  // namespace meister_vision
