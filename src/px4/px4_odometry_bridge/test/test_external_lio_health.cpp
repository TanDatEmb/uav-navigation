#include <gtest/gtest.h>

#include <px4_odometry_bridge/external_lio_health.hpp>

namespace px4_odometry_bridge {
namespace {

navigation_contracts::msg::EstimatorHealth healthy_message() {
  navigation_contracts::msg::EstimatorHealth message;
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 345;
  message.localization_epoch = 7U;
  message.state = navigation_contracts::msg::EstimatorHealth::TRACKING;
  message.navigation_valid = true;
  message.covariance_valid = true;
  message.observability_valid = true;
  message.correction_fresh = true;
  message.propagation_valid = true;
  return message;
}

TEST(ExternalLioHealthTest, TypedEpochIsTheResetGenerationAuthority) {
  const auto snapshot = externalLioHealthSnapshot(healthy_message());

  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->source_stamp_ns, 12'000'000'345LL);
  EXPECT_EQ(snapshot->public_frame_generation, 7U);
  EXPECT_TRUE(snapshot->public_frame_generation_valid);
  EXPECT_TRUE(snapshot->lio_valid);
  EXPECT_TRUE(snapshot->covariance_valid);
}

TEST(ExternalLioHealthTest, InvalidTypedHealthFailsClosed) {
  auto message = healthy_message();
  message.localization_epoch = 0U;
  EXPECT_FALSE(externalLioHealthSnapshot(message).has_value());

  message = healthy_message();
  message.correction_fresh = false;
  const auto stale = externalLioHealthSnapshot(message);
  ASSERT_TRUE(stale.has_value());
  EXPECT_FALSE(stale->lio_valid);
  EXPECT_TRUE(stale->public_frame_generation_valid);
}

TEST(ExternalLioHealthTest, RejectsStaleHealthWithoutReplacingAcceptedState) {
  const auto accepted = externalLioHealthSnapshot(healthy_message());
  ASSERT_TRUE(accepted.has_value());

  auto delayed = healthy_message();
  delayed.header.stamp.sec = 11;
  const auto delayed_snapshot = externalLioHealthSnapshot(delayed);
  ASSERT_TRUE(delayed_snapshot.has_value());
  EXPECT_FALSE(externalLioHealthIsNewer(
      delayed_snapshot, accepted->source_stamp_ns));
  EXPECT_TRUE(accepted->lio_valid);

  auto newer_invalid = healthy_message();
  newer_invalid.header.stamp.sec = 13;
  newer_invalid.correction_fresh = false;
  const auto newer_snapshot = externalLioHealthSnapshot(newer_invalid);
  ASSERT_TRUE(newer_snapshot.has_value());
  EXPECT_TRUE(externalLioHealthIsNewer(
      newer_snapshot, accepted->source_stamp_ns));
  EXPECT_FALSE(newer_snapshot->lio_valid);

  auto malformed = healthy_message();
  malformed.localization_epoch = 0U;
  EXPECT_FALSE(externalLioHealthIsNewer(
      externalLioHealthSnapshot(malformed), accepted->source_stamp_ns));
}

}  // namespace
}  // namespace px4_odometry_bridge
