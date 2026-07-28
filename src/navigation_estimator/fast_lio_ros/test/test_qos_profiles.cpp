#include <gtest/gtest.h>

#include <rmw/types.h>

#include "fast_lio_ros/qos_profiles.hpp"

namespace uav::nav::lio {
namespace {

void expectLivoxDriverCompatible(const rclcpp::QoS& qos) {
  const auto& profile = qos.get_rmw_qos_profile();
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 256U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

}  // namespace

TEST(QosProfilesTest, LivoxLidarMatchesDriverPublisherPolicy) {
  expectLivoxDriverCompatible(QosProfiles::livoxLidarInput());
}

TEST(QosProfilesTest, LivoxImuMatchesDriverPublisherPolicy) {
  expectLivoxDriverCompatible(QosProfiles::livoxImuInput());
}

}  // namespace uav::nav::lio
