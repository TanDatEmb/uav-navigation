#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "px4_navigation_external_mode/velocity_only_continuity.hpp"

namespace {

using namespace px4_navigation_external_mode::velocity_only;

Identity identity(const Role role = Role::kMain) {
  Identity value;
  value.mission_id = "mission";
  value.waypoint_index = 2U;
  value.request_id = 7U;
  value.bundle_generation = 11U;
  value.role = role;
  return value;
}

Policy policy() {
  return Policy{5.0, 2.0, 4.0};
}

TEST(VelocityOnlyContinuity, FirstCommandIsAcceptedWithoutManufacturedPreviousState) {
  const auto result = limit(Eigen::Vector3d{1.0, 2.0, 0.0}, 1'000'000'000LL,
                            identity(), policy(), nullptr);
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.velocity_enu.isApprox(Eigen::Vector3d{1.0, 2.0, 0.0}));
  EXPECT_FALSE(result.limited);
}

TEST(VelocityOnlyContinuity, AccelerationAndJerkAreBoundedAcrossSameOwner) {
  Previous previous;
  previous.identity = identity();
  previous.velocity_enu = Eigen::Vector3d::Zero();
  previous.acceleration_enu = Eigen::Vector3d::Zero();
  previous.stamp_ns = 1'000'000'000LL;

  const auto result = limit(Eigen::Vector3d{4.0, 0.0, 0.0}, 1'100'000'000LL,
                            identity(), policy(), &previous);
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.limited);
  EXPECT_LE(result.acceleration_enu.norm(), 2.0 + 1.0e-12);
  EXPECT_LE(((result.acceleration_enu - previous.acceleration_enu) / 0.1).norm(),
            4.0 + 1.0e-12);
  EXPECT_TRUE(result.velocity_enu.allFinite());
}

TEST(VelocityOnlyContinuity, MainToBackupPreservesContinuityOwner) {
  Previous previous;
  previous.identity = identity(Role::kMain);
  previous.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  previous.stamp_ns = 1'000'000'000LL;

  const auto result = limit(Eigen::Vector3d{1.5, 0.0, 0.0}, 1'100'000'000LL,
                            identity(Role::kBackup), policy(), &previous);
  EXPECT_TRUE(result.success());
}

TEST(VelocityOnlyContinuity, AuthorizedWaypointHandoffPreservesContinuity) {
  Previous previous;
  previous.identity = identity();
  previous.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  previous.acceleration_enu = Eigen::Vector3d{0.5, 0.0, 0.0};
  previous.stamp_ns = 1'000'000'000LL;
  auto successor = identity();
  successor.waypoint_index++;
  successor.request_id++;
  successor.bundle_generation++;
  const auto result = limit(Eigen::Vector3d{1.1, 0.0, 0.0}, 1'100'000'000LL,
                            successor, policy(), &previous);
  EXPECT_TRUE(result.success());
}

TEST(VelocityOnlyContinuity, RejectsPostFilterVelocityCapViolation) {
  Previous previous;
  previous.identity = identity();
  previous.velocity_enu = Eigen::Vector3d{1.99, 0.0, 0.0};
  previous.acceleration_enu = Eigen::Vector3d{2.0, 0.0, 0.0};
  previous.stamp_ns = 1'000'000'000LL;
  auto bounded = policy();
  bounded.maximum_velocity_mps = 2.0;
  const auto result = limit(Eigen::Vector3d{2.0, 0.0, 0.0}, 1'020'000'000LL,
                            identity(), bounded, &previous);
  EXPECT_EQ(result.failure, Failure::kVelocityLimit);
}

TEST(VelocityOnlyContinuity, ProjectsNearCapTurnIntoJointReachableVelocityAccelerationJerkSet) {
  Previous previous;
  previous.identity = identity();
  previous.velocity_enu = Eigen::Vector3d{2.0, 0.0, 0.0};
  previous.acceleration_enu = Eigen::Vector3d::Zero();
  previous.stamp_ns = 1'000'000'000LL;
  auto bounded = Policy{2.0, 2.0, 4.0};

  constexpr double dt_s = 0.02;
  constexpr std::int64_t dt_ns = 20'000'000LL;
  for (int step = 1; step <= 100; ++step) {
    const double angle = 0.2 * static_cast<double>(step) * dt_s;
    const Eigen::Vector3d desired{
        2.0 * std::cos(angle), 2.0 * std::sin(angle), 0.0};
    const auto result = limit(
        desired, previous.stamp_ns + dt_ns, previous.identity, bounded, &previous);
    ASSERT_TRUE(result.success()) << "step=" << step
                                  << " failure=" << static_cast<int>(result.failure)
                                  << " v=" << result.velocity_enu.norm()
                                  << " a=" << result.acceleration_enu.norm()
                                  << " jerk="
                                  << (result.acceleration_enu - previous.acceleration_enu).norm() /
                                         dt_s;
    EXPECT_LE(result.velocity_enu.norm(), bounded.maximum_velocity_mps + 1.0e-10);
    EXPECT_LE(result.acceleration_enu.norm(), bounded.maximum_acceleration_mps2 + 1.0e-10);
    EXPECT_LE(
        (result.acceleration_enu - previous.acceleration_enu).norm() / dt_s,
        bounded.maximum_jerk_mps3 + 1.0e-10);
    previous.velocity_enu = result.velocity_enu;
    previous.acceleration_enu = result.acceleration_enu;
    previous.stamp_ns += dt_ns;
  }
}

TEST(VelocityOnlyContinuity, RejectsUnboundedPreviousAcceleration) {
  Previous previous;
  previous.identity = identity();
  previous.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  previous.acceleration_enu = Eigen::Vector3d{2.1, 0.0, 0.0};
  previous.stamp_ns = 1'000'000'000LL;
  const auto result = limit(Eigen::Vector3d{1.1, 0.0, 0.0}, 1'100'000'000LL,
                            identity(), policy(), &previous);
  EXPECT_EQ(result.failure, Failure::kAccelerationLimit);
}

TEST(VelocityOnlyContinuity, NewCertifiedBundleWithinSameOwnerPreservesContinuity) {
  Previous previous;
  previous.identity = identity();
  previous.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  previous.stamp_ns = 1'000'000'000LL;
  auto successor = identity();
  successor.bundle_generation++;
  const auto result = limit(Eigen::Vector3d{1.1, 0.0, 0.0}, 1'100'000'000LL,
                            successor, policy(), &previous);
  EXPECT_TRUE(result.success());
}

TEST(VelocityOnlyContinuity, ObsoleteOrEmergencyOwnerFailsClosed) {
  Previous previous;
  previous.identity = identity(Role::kBackup);
  previous.stamp_ns = 1'000'000'000LL;

  auto changed = identity(Role::kMain);
  changed.request_id++;
  EXPECT_EQ(limit(Eigen::Vector3d::Zero(), 1'100'000'000LL, changed, policy(), &previous).failure,
            Failure::kIdentityBoundary);

  changed = identity(Role::kMain);
  changed.role = static_cast<Role>(2U);
  EXPECT_EQ(limit(Eigen::Vector3d::Zero(), 1'100'000'000LL, changed, policy(), &previous).failure,
            Failure::kIdentityBoundary);
}

TEST(VelocityOnlyContinuity, ResetOrNonMonotonicTimeDoesNotReusePreviousCommand) {
  Previous previous;
  previous.identity = identity();
  previous.stamp_ns = 1'000'000'000LL;
  EXPECT_EQ(limit(Eigen::Vector3d::Zero(), 1'000'000'000LL, identity(), policy(), &previous).failure,
            Failure::kNonPositiveDelta);
  previous.velocity_enu.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(limit(Eigen::Vector3d::Zero(), 1'100'000'000LL, identity(), policy(), &previous).failure,
            Failure::kInvalidInput);
}

}  // namespace
