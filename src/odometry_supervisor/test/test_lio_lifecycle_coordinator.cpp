#include <gtest/gtest.h>

#include "odometry_supervisor/lio_lifecycle_coordinator.hpp"

namespace {

odometry_supervisor::OdometryState corrected(std::int64_t timestamp) {
  odometry_supervisor::OdometryState state;
  state.timestamp_ns = timestamp;
  state.frame_id = "lio_odom";
  state.child_frame_id = "base_link";
  state.valid = true;
  return state;
}

}  // namespace

TEST(LioLifecycleCoordinator, CapturesLastGoodSnapshotAndDropsItOnGenerationChange) {
  odometry_supervisor::LioLifecycleCoordinator coordinator;
  coordinator.observe({1, true, false, false, corrected(1'000'000'000)});
  ASSERT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kTracking);
  ASSERT_TRUE(coordinator.snapshot().has_value());
  coordinator.observe({2, true, false, false, std::nullopt});
  EXPECT_FALSE(coordinator.snapshot().has_value());
  EXPECT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kStartup);
}

TEST(LioLifecycleCoordinator, ReinitializationNeedsLastGoodSnapshot) {
  odometry_supervisor::LioLifecycleCoordinator coordinator;
  coordinator.observe({1, true, false, false, corrected(1'000'000'000)});
  coordinator.observe({1, false, false, true, std::nullopt});
  EXPECT_TRUE(coordinator.requestReinitialization());
  EXPECT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kReinitializing);
  EXPECT_EQ(coordinator.reinitialization_count(), 1U);
  coordinator.acceptReinitialization(2);
  EXPECT_FALSE(coordinator.snapshot().has_value());
  EXPECT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kStartup);
}

TEST(LioLifecycleCoordinator, DistinguishesStartupInvalidityFromLostTracking) {
  odometry_supervisor::LioLifecycleCoordinator coordinator;
  coordinator.observe({1, false, false, false, std::nullopt});
  EXPECT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kStartup);
  EXPECT_FALSE(coordinator.trackingEverConfirmed());

  coordinator.observe({1, true, false, false, corrected(1'000'000'000)});
  EXPECT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kTracking);
  EXPECT_TRUE(coordinator.trackingEverConfirmed());

  coordinator.observe({1, false, false, false, std::nullopt});
  EXPECT_EQ(coordinator.state(), odometry_supervisor::LioLifecycleState::kLost);
  EXPECT_TRUE(coordinator.trackingEverConfirmed());
}
