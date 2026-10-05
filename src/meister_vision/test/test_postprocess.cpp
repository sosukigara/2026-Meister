// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core.hpp>

#include "meister_vision/nms.hpp"
#include "meister_vision/yolo_detector.hpp"

namespace meister_vision {
namespace {

constexpr int kStride = kYoloNumClasses + kYoloBBoxCols;

/// (1, 84, 8400) 相当の平坦配列。全部 0 で、指定 cell だけ埋める。
std::vector<float> MakeOutput() {
  return std::vector<float>(static_cast<size_t>(kStride) * kYoloGridCells, 0.0f);
}

void SetCell(std::vector<float>& out, int cell, double cx, double cy, double w,
             double h, int class_id, float score) {
  const size_t base = static_cast<size_t>(cell) * kStride;
  out[base + 0] = static_cast<float>(cx);
  out[base + 1] = static_cast<float>(cy);
  out[base + 2] = static_cast<float>(w);
  out[base + 3] = static_cast<float>(h);
  out[base + kYoloBBoxCols + class_id] = score;
}

/// letterbox が scale 1.0 / pad 0 だった場合として素通しする。
/// 名前は Run ではなく Decode。gtest の Test::Run と衝突する。
std::vector<Detection> Decode(const std::vector<float>& out, float conf = 0.25f) {
  return YoloPostprocess(out, 1.0, 0, 0, 640, 640, conf, 0.45f);
}

TEST(CocoClasses, HasEightyNames) {
  EXPECT_EQ(CocoClasses().size(), static_cast<size_t>(kYoloNumClasses));
  EXPECT_EQ(CocoClasses()[0], "person");
  EXPECT_EQ(CocoClasses()[79], "toothbrush");
}

TEST(YoloPostprocess, RejectsWrongElementCount) {
  EXPECT_THROW(Decode(std::vector<float>(10, 0.0f)), std::runtime_error);
}

TEST(YoloPostprocess, RejectsNonPositiveRatio) {
  EXPECT_THROW(
      YoloPostprocess(MakeOutput(), 0.0, 0, 0, 640, 640, 0.25f, 0.45f),
      std::invalid_argument);
}

TEST(YoloPostprocess, EmptyWhenNothingPassesThreshold) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 5, 100, 100, 20, 20, 3, 0.10f);
  EXPECT_TRUE(Decode(out, 0.25f).empty());
}

TEST(YoloPostprocess, ConvertsCxCyWhToXyxy) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 7, 200.0, 150.0, 40.0, 20.0, 5, 0.9f);
  const std::vector<Detection> dets = Decode(out);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_NEAR(dets[0].x1, 180.0f, 1e-4);
  EXPECT_NEAR(dets[0].y1, 140.0f, 1e-4);
  EXPECT_NEAR(dets[0].x2, 220.0f, 1e-4);
  EXPECT_NEAR(dets[0].y2, 160.0f, 1e-4);
  EXPECT_EQ(dets[0].class_id, 5);
  EXPECT_EQ(dets[0].class_name, "bus");
  EXPECT_FLOAT_EQ(dets[0].confidence, 0.9f);
}

TEST(YoloPostprocess, PicksHighestScoringClassPerCell) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 1, 100.0, 100.0, 10.0, 10.0, 1, 0.30f);
  SetCell(out, 1, 100.0, 100.0, 10.0, 10.0, 44, 0.80f);
  const std::vector<Detection> dets = Decode(out);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, 44);
  // COCO 44 は spoon。book ではない（COCO には book が無い）。
  EXPECT_EQ(dets[0].class_name, "spoon");
}

TEST(YoloPostprocess, ClampsToImageBounds) {
  std::vector<float> out = MakeOutput();
  // 完全に画像外に出る枠は clamp 後に幅 0 になり NMS の契約外になるので、
  // clamp が効く「一部分だけ外」的座標を使う。
  SetCell(out, 3, 5.0, 5.0, 20.0, 20.0, 2, 0.7f);
  SetCell(out, 4, 635.0, 635.0, 20.0, 20.0, 2, 0.7f);
  const std::vector<Detection> dets = Decode(out);
  ASSERT_EQ(dets.size(), 2U);
  for (const Detection& d : dets) {
    EXPECT_GE(d.x1, 0.0f);
    EXPECT_GE(d.y1, 0.0f);
    EXPECT_LE(d.x2, 640.0f);
    EXPECT_LE(d.y2, 640.0f);
  }
}

TEST(YoloPostprocess, UndoesLetterboxPadAndScale) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 2, 200.0, 100.0, 50.0, 20.0, 0, 0.5f);
  const std::vector<Detection> dets =
      YoloPostprocess(out, 0.5, 20, 10, 640, 480, 0.25f, 0.45f);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_NEAR((dets[0].x1 + dets[0].x2) / 2.0f, 360.0f, 1e-3);
  EXPECT_NEAR((dets[0].y1 + dets[0].y2) / 2.0f, 180.0f, 1e-3);
}

TEST(YoloPostprocess, SuppressesOverlappingBoxes) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 10, 100.0, 100.0, 40.0, 40.0, 0, 0.90f);
  SetCell(out, 11, 102.0, 101.0, 40.0, 40.0, 0, 0.70f);
  EXPECT_EQ(Decode(out).size(), 1U);
}

TEST(YoloPostprocess, KeepsDistantBoxes) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 12, 50.0, 50.0, 10.0, 10.0, 0, 0.9f);
  SetCell(out, 13, 300.0, 300.0, 10.0, 10.0, 1, 0.9f);
  EXPECT_EQ(Decode(out).size(), 2U);
}

TEST(YoloPostprocess, OrdersByDescendingScore) {
  std::vector<float> out = MakeOutput();
  SetCell(out, 20, 50.0, 50.0, 10.0, 10.0, 0, 0.40f);
  SetCell(out, 21, 300.0, 300.0, 10.0, 10.0, 1, 0.95f);
  SetCell(out, 22, 500.0, 500.0, 10.0, 10.0, 2, 0.60f);
  const std::vector<Detection> dets = Decode(out);
  ASSERT_EQ(dets.size(), 3U);
  EXPECT_NEAR(dets[0].confidence, 0.95f, 1e-6);
  EXPECT_NEAR(dets[1].confidence, 0.60f, 1e-6);
  EXPECT_NEAR(dets[2].confidence, 0.40f, 1e-6);
}

TEST(NonMaximumSuppression, EmptyInput) {
  EXPECT_TRUE(NonMaximumSuppression({}, {}, 0.0f, 0.45f).empty());
}

TEST(NonMaximumSuppression, RejectsMismatchedLengths) {
  EXPECT_THROW(NonMaximumSuppression({cv::Rect(0, 0, 1, 1)}, {}, 0.0f, 0.45f),
               std::invalid_argument);
}

TEST(NonMaximumSuppression, KeepsNonOverlappingBoxes) {
  const std::vector<cv::Rect> boxes = {cv::Rect(0, 0, 10, 10),
                                       cv::Rect(100, 100, 10, 10)};
  const std::vector<float> scores = {0.5f, 0.4f};
  EXPECT_EQ(NonMaximumSuppression(boxes, scores, 0.0f, 0.45f).size(), 2U);
}

}  // namespace
}  // namespace meister_vision
