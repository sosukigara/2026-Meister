// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "meister_vision/hand_landmark_detector.hpp"

namespace meister_vision {
namespace {

constexpr int kK = 21;

std::vector<float> MakeOutput(int k = kK, float score = 0.9f) {
  std::vector<float> out(static_cast<size_t>(k) * 3, 0.0f);
  for (int i = 0; i < k; ++i) {
    const size_t base = static_cast<size_t>(i) * 3;
    out[base + 0] = 0.5f;
    out[base + 1] = 0.5f;
    out[base + 2] = score;
  }
  return out;
}

std::optional<HandLandmarks> Decode(const std::vector<float>& out, int k = kK) {
  return DecodeHandLandmarks(out, k, 224, 1.0, 0, 0, 640, 480, 0.2f);
}

TEST(ProbeLayout, NchwFromChannelSecond) {
  int side = 0;
  EXPECT_EQ(ProbeLayout({1, 3, 224, 224}, &side), TensorLayout::kNchw);
  EXPECT_EQ(side, 224);
}

TEST(ProbeLayout, NhwcFromChannelLast) {
  int side = 0;
  EXPECT_EQ(ProbeLayout({1, 224, 224, 3}, &side), TensorLayout::kNhwc);
  EXPECT_EQ(side, 224);
}

TEST(ProbeLayout, RejectsDynamicShape) {
  int side = 0;
  EXPECT_THROW(ProbeLayout({1, 3, -1, -1}, &side), std::runtime_error);
}

TEST(ProbeLayout, RejectsNonFourDimensional) {
  int side = 0;
  EXPECT_THROW(ProbeLayout({1, 3, 224}, &side), std::runtime_error);
}

TEST(ProbeLayout, RejectsMissingChannelThree) {
  int side = 0;
  EXPECT_THROW(ProbeLayout({1, 4, 224, 224}, &side), std::runtime_error);
}

TEST(ProbeLayout, RejectsMonoInput) {
  int side = 0;
  EXPECT_THROW(ProbeLayout({1, 1, 224, 224}, &side), std::runtime_error);
}

TEST(DecodeHandLandmarks, RejectsWrongElementCount) {
  EXPECT_THROW(Decode(std::vector<float>(10, 0.5f)), std::runtime_error);
}

TEST(DecodeHandLandmarks, ReturnsNulloptBelowThreshold) {
  std::vector<float> out = MakeOutput();
  out[2] = 0.05f;  // 1 点だけ低い
  EXPECT_FALSE(Decode(out).has_value());
}

TEST(DecodeHandLandmarks, ConfidenceIsTheMinimumScore) {
  std::vector<float> out = MakeOutput();
  out[4 * 3 + 2] = 0.30f;
  const auto r = Decode(out);
  ASSERT_TRUE(r.has_value());
  EXPECT_FLOAT_EQ(r->confidence, 0.30f);
}

TEST(DecodeHandLandmarks, ScalesNormalizedCoordsToCanvas) {
  std::vector<float> out = MakeOutput();
  out[0] = 0.5f;
  const auto r = Decode(out);
  ASSERT_TRUE(r.has_value());
  ASSERT_EQ(r->landmarks.size(), static_cast<size_t>(kK));
  EXPECT_FLOAT_EQ(r->landmarks[0].x, 112.0f);
  EXPECT_FLOAT_EQ(r->landmarks[0].y, 112.0f);
}

TEST(DecodeHandLandmarks, UndoesPadAndRatio) {
  std::vector<float> out = MakeOutput();
  const auto r = DecodeHandLandmarks(out, kK, 224, 0.5, 20, 0, 640, 480, 0.2f);
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->landmarks[0].x, 184.0f, 1e-3);
  EXPECT_NEAR(r->landmarks[0].y, 224.0f, 1e-3);
}

TEST(DecodeHandLandmarks, ClampsToImageBounds) {
  std::vector<float> lo = MakeOutput();
  lo[0] = 0.0f;
  lo[1] = 0.0f;
  const auto l = Decode(lo);
  ASSERT_TRUE(l.has_value());
  EXPECT_FLOAT_EQ(l->landmarks[0].x, 0.0f);
  EXPECT_FLOAT_EQ(l->landmarks[0].y, 0.0f);

  std::vector<float> hi = MakeOutput();
  hi[0] = 1.0f;
  hi[1] = 1.0f;
  const auto h = Decode(hi);
  ASSERT_TRUE(h.has_value());
  EXPECT_FLOAT_EQ(h->landmarks[0].x, 224.0f);
  EXPECT_FLOAT_EQ(h->landmarks[0].y, 224.0f);
}

TEST(DecodeHandLandmarks, KeepsLandmarkOrder) {
  std::vector<float> out = MakeOutput();
  for (int i = 0; i < kK; ++i) {
    out[static_cast<size_t>(i) * 3 + 0] = static_cast<float>(i) / 100.0f;
  }
  const auto r = Decode(out);
  ASSERT_TRUE(r.has_value());
  for (int i = 0; i < kK; ++i) {
    EXPECT_NEAR(r->landmarks[static_cast<size_t>(i)].x, i * 2.24f, 1e-3);
  }
}

TEST(DecodeHandLandmarks, HonorsNumLandmarksParameter) {
  const auto r = Decode(MakeOutput(5), 5);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->landmarks.size(), 5U);
}

TEST(DecodeHandLandmarks, RejectsNonPositiveRatio) {
  EXPECT_THROW(DecodeHandLandmarks(MakeOutput(), kK, 224, 0.0, 0, 0, 640, 480,
                                   0.2f),
               std::invalid_argument);
}

TEST(ResolveHandModelPath, IsNonEmpty) {
  // モデルが無くても「最後の候補」を返す契約。空文字を返すと
  // 「モデルが無い」理由が失われる。
  EXPECT_FALSE(ResolveHandModelPath().empty());
}

}  // namespace
}  // namespace meister_vision
