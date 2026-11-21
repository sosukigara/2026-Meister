// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "umeonigiri_vision/letterbox.hpp"

namespace umeonigiri_vision {
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

TEST(Letterbox, RejectsDegenerateAspectRatio) {
  // 短辺が 0 に丸まる入力。以前は cv::resize のアサーションで落ちていた
  // (OpenCV の不可解なエラーになり、原因が分からなかった)。
  EXPECT_THROW(Letterbox(Solid(1, 5000, cv::Scalar::all(0)), 640),
               std::invalid_argument);
  EXPECT_THROW(Letterbox(Solid(5000, 1, cv::Scalar::all(0)), 640),
               std::invalid_argument);
}

TEST(Letterbox, KeepsExtremeButUsableAspectRatio) {
  // 短辺が 1 ピクセル以上残るなら通す。ここを弾くと、細長いカメラ画像を
  // 理由なく捨ててしまう。
  const LetterboxResult wide = Letterbox(Solid(4, 1280, cv::Scalar::all(0)), 640);
  EXPECT_EQ(wide.padded.rows, 640);
  EXPECT_EQ(wide.padded.cols, 640);
  const LetterboxResult tall = Letterbox(Solid(1280, 4, cv::Scalar::all(0)), 640);
  EXPECT_EQ(tall.padded.rows, 640);
  EXPECT_EQ(tall.padded.cols, 640);
}

}  // namespace
}  // namespace umeonigiri_vision
