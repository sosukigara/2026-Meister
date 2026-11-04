// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <vector>

#include "meister_vision/target_selector.hpp"

namespace meister_vision {
namespace {

Detection MakeDetection(float x1, float y1, float x2, float y2, int class_id,
                        float confidence) {
  Detection det;
  det.x1 = x1;
  det.y1 = y1;
  det.x2 = x2;
  det.y2 = y2;
  det.class_id = class_id;
  det.class_name = "c" + std::to_string(class_id);
  det.confidence = confidence;
  return det;
}

TargetSelectorOptions DefaultOptions() {
  TargetSelectorOptions options;
  options.target_class = -1;
  options.min_confidence = 0.25f;
  options.lock_radius_px = 120.0f;
  // 基本の選択ロジックを見るテストでは猶予なしで即座に乗り換える。
  options.lock_grace_frames = 1;
  return options;
}

TEST(TargetSelector, EmptyWhenNoDetections) {
  TargetSelector selector(DefaultOptions());
  const Target target = selector.Select({}, 640, 480);
  EXPECT_FALSE(target.valid);
}

TEST(TargetSelector, RejectsNonPositiveImageSize) {
  TargetSelector selector(DefaultOptions());
  const auto dets = {MakeDetection(300, 220, 340, 260, 0, 0.9f)};
  EXPECT_FALSE(selector.Select(dets, 0, 480).valid);
  EXPECT_FALSE(selector.Select(dets, 640, -1).valid);
}

TEST(TargetSelector, PicksDetectionClosestToImageCenter) {
  TargetSelector selector(DefaultOptions());
  const std::vector<Detection> dets = {
      MakeDetection(0, 0, 40, 40, 0, 0.99f),          // 左上 (中心から遠い)
      MakeDetection(300, 220, 340, 260, 0, 0.60f),    // ほぼ中央
  };
  const Target target = selector.Select(dets, 640, 480);
  ASSERT_TRUE(target.valid);
  EXPECT_FLOAT_EQ(target.detection.confidence, 0.60f);
  // 中心 (320, 240) に対する (320, 240) はオフセット 0。
  EXPECT_NEAR(target.offset_x, 0.0f, 1e-5);
  EXPECT_NEAR(target.offset_y, 0.0f, 1e-5);
}

TEST(TargetSelector, NormalizesOffsetWithRightAndDownPositive) {
  TargetSelector selector(DefaultOptions());
  // 中心 (64, 48) から右下へずらした枠。
  const auto dets = {MakeDetection(76, 58, 92, 74, 0, 0.9f)};
  const Target target = selector.Select(dets, 128, 96);
  ASSERT_TRUE(target.valid);
  EXPECT_GT(target.offset_x, 0.0f);
  EXPECT_GT(target.offset_y, 0.0f);
  EXPECT_NEAR(target.offset_x, (84.0f - 64.0f) / 64.0f, 1e-5);
  EXPECT_NEAR(target.offset_y, (66.0f - 48.0f) / 48.0f, 1e-5);
}

TEST(TargetSelector, FiltersByClass) {
  TargetSelectorOptions options = DefaultOptions();
  options.target_class = 3;
  TargetSelector selector(options);
  const std::vector<Detection> dets = {
      MakeDetection(300, 220, 340, 260, 0, 0.90f),  // 中央だがクラス違い
      MakeDetection(20, 20, 60, 60, 3, 0.50f),      // クラス一致
  };
  const Target target = selector.Select(dets, 640, 480);
  ASSERT_TRUE(target.valid);
  EXPECT_EQ(target.detection.class_id, 3);
}

TEST(TargetSelector, FiltersByConfidence) {
  TargetSelector selector(DefaultOptions());
  const std::vector<Detection> dets = {
      MakeDetection(300, 220, 340, 260, 0, 0.10f),  // しきい値未満
  };
  EXPECT_FALSE(selector.Select(dets, 640, 480).valid);
}

TEST(TargetSelector, KeepsLockWhileTargetMovesSlightly) {
  TargetSelector selector(DefaultOptions());
  const auto first = selector.Select(
      {MakeDetection(300, 220, 340, 260, 0, 0.90f)}, 640, 480);
  ASSERT_TRUE(first.valid);
  EXPECT_FALSE(first.locked);  // 初回は乗り換え扱い

  // 近くに別クラスの高信頼物体が現れても、ロック中の物体を追う。
  const std::vector<Detection> dets = {
      MakeDetection(10, 10, 50, 50, 1, 0.99f),
      MakeDetection(305, 225, 345, 265, 0, 0.80f),
  };
  const Target second = selector.Select(dets, 640, 480);
  ASSERT_TRUE(second.valid);
  EXPECT_TRUE(second.locked);
  EXPECT_EQ(second.detection.class_id, 0);
}

TEST(TargetSelector, ReleasesLockWhenTargetDisappears) {
  TargetSelector selector(DefaultOptions());
  selector.Select({MakeDetection(300, 220, 340, 260, 0, 0.90f)}, 640, 480);
  // 中央付近から消えたら、画像中心に近い別の物体へ乗り換える。
  const std::vector<Detection> dets = {
      MakeDetection(400, 300, 440, 340, 2, 0.70f),
  };
  const Target target = selector.Select(dets, 640, 480);
  ASSERT_TRUE(target.valid);
  EXPECT_FALSE(target.locked);
  EXPECT_EQ(target.detection.class_id, 2);
}

TEST(TargetSelector, DoesNotJumpBeyondLockRadius) {
  TargetSelector selector(DefaultOptions());
  selector.Select({MakeDetection(300, 220, 340, 260, 0, 0.90f)}, 640, 480);
  // ロック半径 120px を超えて離れた同クラス物体は追わない。
  const std::vector<Detection> dets = {
      MakeDetection(560, 400, 600, 440, 0, 0.90f),
  };
  const Target target = selector.Select(dets, 640, 480);
  ASSERT_TRUE(target.valid);
  EXPECT_FALSE(target.locked);
}

TEST(TargetSelector, ResetsWhenAllTargetsDisappear) {
  TargetSelector selector(DefaultOptions());
  selector.Select({MakeDetection(300, 220, 340, 260, 0, 0.90f)}, 640, 480);
  EXPECT_FALSE(selector.Select({}, 640, 480).valid);
  // リセット後はロック扱いにならず、中心優先で選び直す。
  const Target after = selector.Select(
      {MakeDetection(300, 220, 340, 260, 0, 0.9f)}, 640, 480);
  ASSERT_TRUE(after.valid);
  EXPECT_FALSE(after.locked);
}

TEST(TargetSelector, HoldsTargetThroughBriefDropout) {
  TargetSelectorOptions options = DefaultOptions();
  options.lock_grace_frames = 3;
  TargetSelector selector(options);
  selector.Select({MakeDetection(300, 220, 340, 260, 0, 0.90f)}, 640, 480);

  // 1 フレーム検出が飛んでも、別の物体へ乗り換えずに「なし」を返す。
  EXPECT_FALSE(selector.Select({}, 640, 480).valid);
  EXPECT_FALSE(selector.Select({}, 640, 480).valid);

  // 復帰したら同じ対象をロック継続として戻す。
  const Target back = selector.Select(
      {MakeDetection(302, 222, 342, 262, 0, 0.80f)}, 640, 480);
  ASSERT_TRUE(back.valid);
  EXPECT_TRUE(back.locked);
  EXPECT_EQ(back.detection.class_id, 0);
}

TEST(TargetSelector, ReacquiresAfterGraceExpires) {
  TargetSelectorOptions options = DefaultOptions();
  options.lock_grace_frames = 3;
  TargetSelector selector(options);
  selector.Select({MakeDetection(300, 220, 340, 260, 0, 0.90f)}, 640, 480);

  // 猶予を使い切ったら別の物体へ乗り換える。
  EXPECT_FALSE(selector.Select({}, 640, 480).valid);
  EXPECT_FALSE(selector.Select({}, 640, 480).valid);
  const Target switched = selector.Select(
      {MakeDetection(400, 300, 440, 340, 2, 0.70f)}, 640, 480);
  ASSERT_TRUE(switched.valid);
  EXPECT_FALSE(switched.locked);
  EXPECT_EQ(switched.detection.class_id, 2);
}

TEST(TargetSelector, IgnoresDegenerateBoxes) {
  TargetSelector selector(DefaultOptions());
  const std::vector<Detection> dets = {
      MakeDetection(300, 220, 300, 260, 0, 0.99f),  // 幅 0
  };
  EXPECT_FALSE(selector.Select(dets, 640, 480).valid);
}

}  // namespace
}  // namespace meister_vision
