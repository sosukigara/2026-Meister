// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "umeonigiri_vision/publish_rate_gate.hpp"

namespace umeonigiri_vision {
namespace {

TEST(PublishRateGate, FirstCallAlwaysPasses) {
  PublishRateGate gate(30.0);
  EXPECT_TRUE(gate.Ready(10.0));
}

TEST(PublishRateGate, BlocksFasterThanConfigured) {
  PublishRateGate gate(30.0);
  ASSERT_TRUE(gate.Ready(0.0));
  EXPECT_FALSE(gate.Ready(0.02));
  EXPECT_FALSE(gate.Ready(1.0 / 30.0 - 0.0001));
  EXPECT_TRUE(gate.Ready(1.0 / 30.0));
}

TEST(PublishRateGate, ZeroHzMeansUnlimited) {
  PublishRateGate gate(0.0);
  EXPECT_TRUE(gate.Ready(0.0));
  EXPECT_TRUE(gate.Ready(0.0));
}

}  // namespace
}  // namespace umeonigiri_vision
