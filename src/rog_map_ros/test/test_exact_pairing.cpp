#include <gtest/gtest.h>

#include "rog_map_ros/exact_pairing.hpp"

namespace uav::nav::rog {

TEST(ExactPairingTest, RequiresIntegerNanosecondEquality) {
  ExactPairingCache<int, int> cache(2);
  cache.insertCloud(1000000001, 1);
  cache.insertOdom(1000000002, 2);
  EXPECT_FALSE(cache.takePair(1000000001).has_value());
  cache.insertOdom(1000000001, 3);
  ASSERT_TRUE(cache.takePair(1000000001).has_value());
  EXPECT_EQ(cache.counters().paired, 1U);
  EXPECT_EQ(cache.counters().timestamp_mismatch, 0U);
}

TEST(ExactPairingTest, DuplicateAndCapacityAreBounded) {
  ExactPairingCache<int, int> cache(1);
  cache.insertCloud(1, 1);
  cache.insertCloud(1, 2);
  cache.insertCloud(2, 3);
  EXPECT_EQ(cache.counters().duplicate_cloud, 1U);
  EXPECT_EQ(cache.counters().expired_cloud, 1U);
  EXPECT_EQ(cache.cloudDepth(), 1U);
}

}  // namespace uav::nav::rog
