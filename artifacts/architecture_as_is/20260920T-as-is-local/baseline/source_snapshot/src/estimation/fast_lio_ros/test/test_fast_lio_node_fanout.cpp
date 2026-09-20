#include <gtest/gtest.h>

#include "fast_lio_ros/fast_lio_node.hpp"

namespace uav::nav::lio {
namespace {

TEST(FastLioNodeFanoutTest, MainRejectedSampleOnlyRequestsShedding) {
  EXPECT_EQ(propagatedImuFanoutAction(false, false),
            PropagatedImuFanoutAction::kRequestLoadSheddingOnly);
  EXPECT_EQ(propagatedImuFanoutAction(false, true),
            PropagatedImuFanoutAction::kRequestLoadSheddingOnly);
}

TEST(FastLioNodeFanoutTest, AcceptedSampleIsEnqueuedEvenAtOverloadThreshold) {
  EXPECT_EQ(propagatedImuFanoutAction(true, false),
            PropagatedImuFanoutAction::kEnqueue);
  EXPECT_EQ(propagatedImuFanoutAction(true, true),
            PropagatedImuFanoutAction::kRequestLoadSheddingAndEnqueue);
}

}  // namespace
}  // namespace uav::nav::lio
