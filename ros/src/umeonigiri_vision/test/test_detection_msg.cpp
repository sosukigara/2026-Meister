// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "umeonigiri_vision/detection_msg.hpp"

namespace umeonigiri_vision {
namespace {

TEST(FillDetection2D, CarriesBothClassNameAndNumericId) {
  DetectionIdentity identity;
  identity.class_id = 7;
  identity.class_name = "cat";
  identity.score = 0.8;
  identity.x0 = 10.0;
  identity.y0 = 20.0;
  identity.x1 = 30.0;
  identity.y1 = 60.0;

  vision_msgs::msg::Detection2D out;
  FillDetection2D(identity, out);

  EXPECT_EQ(out.id, "cat");
  ASSERT_EQ(out.results.size(), 1u);
  EXPECT_EQ(out.results[0].hypothesis.class_id, "7");
  EXPECT_DOUBLE_EQ(out.results[0].hypothesis.score, 0.8);
  EXPECT_DOUBLE_EQ(out.bbox.center.position.x, 20.0);
  EXPECT_DOUBLE_EQ(out.bbox.center.position.y, 40.0);
  EXPECT_DOUBLE_EQ(out.bbox.size_x, 20.0);
  EXPECT_DOUBLE_EQ(out.bbox.size_y, 40.0);
}

TEST(FillDetection2D, ReplacesPreviousResults) {
  DetectionIdentity identity;
  vision_msgs::msg::Detection2D out;
  vision_msgs::msg::ObjectHypothesisWithPose stale;
  stale.hypothesis.class_id = "stale";
  out.results.push_back(stale);

  FillDetection2D(identity, out);
  ASSERT_EQ(out.results.size(), 1u);
  EXPECT_EQ(out.results[0].hypothesis.class_id, "0");
}

}  // namespace
}  // namespace umeonigiri_vision
