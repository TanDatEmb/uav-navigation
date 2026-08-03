#include <gtest/gtest.h>

#include <cmath>

#include "odometry_supervisor/world_alignment.hpp"

namespace {

odometry_supervisor::OdometryState source_state() {
  odometry_supervisor::OdometryState state;
  state.timestamp_ns = 42;
  state.frame_id = "px4_odom";
  state.child_frame_id = "base_link";
  state.position_odom = Eigen::Vector3d(1.0, 2.0, 3.0);
  state.orientation_odom_base = Eigen::Quaterniond::Identity();
  state.velocity_base = Eigen::Vector3d(0.5, -0.2, 0.1);
  state.valid = true;
  return state;
}

}  // namespace

TEST(WorldAlignment, AppliesCapturedTransformAndKeepsBodyVelocity) {
  auto alignment = odometry_supervisor::WorldAlignment{};
  alignment.valid = true;
  alignment.target_from_source_orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(
          std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ()));
  alignment.target_from_source_translation = Eigen::Vector3d(10.0, -1.0, 0.5);

  const auto aligned = odometry_supervisor::applyWorldAlignment(source_state(), alignment);
  ASSERT_TRUE(aligned.has_value());
  EXPECT_EQ(aligned->frame_id, "lio_odom");
  EXPECT_EQ(aligned->child_frame_id, "base_link");
  EXPECT_EQ(aligned->timestamp_ns, 42);
  EXPECT_TRUE(aligned->position_odom.isApprox(Eigen::Vector3d(8.0, 0.0, 3.5), 1e-12));
  EXPECT_TRUE(aligned->velocity_base.isApprox(source_state().velocity_base, 1e-12));
  EXPECT_TRUE(aligned->orientation_odom_base.isApprox(
      Eigen::Quaterniond(Eigen::AngleAxisd(
          std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ())), 1e-12));
}

TEST(WorldAlignment, RejectsUncontractedFrame) {
  auto alignment = odometry_supervisor::WorldAlignment{};
  alignment.valid = true;
  auto state = source_state();
  state.frame_id = "odom";
  EXPECT_FALSE(odometry_supervisor::applyWorldAlignment(state, alignment).has_value());
}
