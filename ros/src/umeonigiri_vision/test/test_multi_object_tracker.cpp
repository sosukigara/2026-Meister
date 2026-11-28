#include <gtest/gtest.h>

#include "umeonigiri_vision/multi_object_tracker.hpp"

namespace umeonigiri_vision
{
namespace
{

TEST(MultiObjectTrackerTest, SuppressesSingleFrameFalsePositiveAndConfirmsPersistentObject)
{
  TrackerConfig cfg{};
  cfg.min_hits = 2;
  cfg.max_missed = 2;
  MultiObjectTracker tracker(cfg);

  // Frame 1: 本物(class 0) と単発ノイズ(class 5)
  auto out1 = tracker.Update({
    {100.0F, 100.0F, 200.0F, 200.0F, 0.75F, 0},
    {300.0F, 300.0F, 350.0F, 350.0F, 0.35F, 5},
  });
  EXPECT_TRUE(out1.empty());

  // Frame 2: 本物のみ継続
  auto out2 = tracker.Update({
    {104.0F, 102.0F, 204.0F, 202.0F, 0.80F, 0},
  });
  ASSERT_EQ(out2.size(), 1U);
  EXPECT_EQ(out2[0].class_id, 0);
  const uint64_t stable_id = out2[0].track_id;

  // Frame 3: 1フレーム一瞬見失っても max_missed=2 の間は保持される
  auto out3 = tracker.Update({});
  ASSERT_EQ(out3.size(), 1U);
  EXPECT_EQ(out3[0].track_id, stable_id);
  EXPECT_EQ(out3[0].missed, 1);

  // Frame 4: 再検出で同じ ID が維持される
  auto out4 = tracker.Update({
    {110.0F, 105.0F, 210.0F, 205.0F, 0.82F, 0},
  });
  ASSERT_EQ(out4.size(), 1U);
  EXPECT_EQ(out4[0].track_id, stable_id);
  EXPECT_EQ(out4[0].missed, 0);
}

TEST(MultiObjectTrackerTest, DropsTrackAfterMaxMissedFrames)
{
  TrackerConfig cfg{};
  cfg.min_hits = 1;
  cfg.max_missed = 2;
  MultiObjectTracker tracker(cfg);

  ASSERT_EQ(tracker.Update({{10.0F, 10.0F, 50.0F, 50.0F, 0.9F, 1}}).size(), 1U);
  EXPECT_EQ(tracker.Update({}).size(), 1U);  // missed=1
  EXPECT_EQ(tracker.Update({}).size(), 1U);  // missed=2
  EXPECT_TRUE(tracker.Update({}).empty());   // missed=3 -> dropped
}

}  // namespace
}  // namespace umeonigiri_vision
