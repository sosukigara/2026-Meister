// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "meister_vision/letterbox.hpp"

namespace meister_vision {
namespace {

cv::Mat Solid(int h, int w, cv::Scalar color) {
  return cv::Mat(h, w, CV_8UC3, color);
}

TEST(Letterbox, PadsToSquare) {
  const LetterboxResult r =
      Letterbox(Solid(480, 640, cv::Scalar(10, 20, 30)), 640);
  EXPECT_EQ(r.padded.rows, 640);
  EXPECT_EQ(r.padded.cols, 640);
  EXPECT_DOUBLE_EQ(r.ratio, 1.0);
  EXPECT_EQ(r.pad_w, 0);
  EXPECT_EQ(r.pad_h, 80);
}

TEST(Letterbox, KeepsAspectRatio) {
  const LetterboxResult r = Letterbox(Solid(800, 400, cv::Scalar(1, 2, 3)), 640);
  EXPECT_NEAR(r.ratio, 0.8, 1e-9);
  EXPECT_EQ(r.pad_h, 0);
  EXPECT_EQ(r.pad_w, 160);
}

TEST(Letterbox, EnlargesSmallImage) {
  const LetterboxResult r = Letterbox(Solid(50, 100, cv::Scalar(9, 9, 9)), 640);
  EXPECT_NEAR(r.ratio, 6.4, 1e-9);
  EXPECT_EQ(r.padded.rows, 640);
  EXPECT_EQ(r.padded.cols, 640);
}

TEST(Letterbox, ContentLandsInRoi) {
  const cv::Scalar fill(200, 100, 50);
  const LetterboxResult r = Letterbox(Solid(480, 640, fill), 640, 7);
  EXPECT_EQ(r.padded.at<cv::Vec3b>(0, 0)[0], 7);
  EXPECT_EQ(r.padded.at<cv::Vec3b>(r.pad_h + 100, 320)[0], 200);
}

TEST(Letterbox, RejectsEmpty) {
  EXPECT_THROW(Letterbox(cv::Mat(), 640), std::invalid_argument);
}

TEST(Letterbox, RejectsNonPositiveSide) {
  EXPECT_THROW(Letterbox(Solid(10, 10, cv::Scalar::all(0)), 0),
               std::invalid_argument);
}

}  // namespace
}  // namespace meister_vision
