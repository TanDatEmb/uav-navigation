// Message conversion at the boundary of the px4_bridge node (SYSTEM_DESIGN §4.1, §6.3, §6.5; D33/O21, O22; Review
// Focus 5). Pure conversions: no rclcpp context, no DDS.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "uavnav/px4bridge/ros_conversion.hpp"

using namespace uavnav;
using namespace uavnav::px4bridge;

namespace {

constexpr float kNaNF = std::numeric_limits<float>::quiet_NaN();

px4_msgs::msg::VehicleLocalPosition GoodLocalPosition() {
  px4_msgs::msg::VehicleLocalPosition m;
  m.timestamp = 12'400'000;
  m.timestamp_sample = 12'345'678;
  m.xy_valid = true;
  m.z_valid = true;
  m.x = 1.5F;
  m.y = -2.5F;
  m.z = -3.0F;
  m.heading = 0.75F;
  m.xy_reset_counter = 3;
  m.z_reset_counter = 4;
  m.heading_reset_counter = 255;
  m.delta_xy = {0.25F, -0.5F};
  m.delta_z = 0.125F;
  m.delta_heading = -0.0625F;
  return m;
}

uavnav_interfaces::msg::LioOdometry GoodOdometry() {
  uavnav_interfaces::msg::LioOdometry m;
  m.stamp.sec = 12;
  m.stamp.nanosec = 345'000'000;
  m.epoch = 7;
  m.reset_counter = 99;  // the output predictor's counter: never the EV epoch (F14)
  m.pose.position.x = 1.0;
  m.pose.position.y = 2.0;
  m.pose.position.z = 3.0;
  m.pose.orientation.w = 0.5;
  m.pose.orientation.x = 0.5;
  m.pose.orientation.y = 0.5;
  m.pose.orientation.z = 0.5;
  m.velocity.x = 0.1;
  m.velocity.y = 0.2;
  m.velocity.z = 0.3;
  for (std::size_t i = 0; i < 6; ++i) m.pose_covariance[i * 6 + i] = 0.01 * static_cast<double>(i + 1);
  m.pose_covariance[1] = 0.001;  // row 0, column 1
  for (std::size_t i = 0; i < 3; ++i) m.velocity_covariance[i * 3 + i] = 0.1 * static_cast<double>(i + 1);
  m.velocity_covariance[5] = 0.002;  // row 1, column 2
  m.quality = 80;
  return m;
}

uavnav_interfaces::msg::LioHealth GoodHealth() {
  uavnav_interfaces::msg::LioHealth m;
  m.stamp.sec = 12;
  m.stamp.nanosec = 500;
  m.state = uavnav_interfaces::msg::LioHealth::DEGRADED;
  m.epoch = 3;
  return m;
}

template <class T>
BridgeReason Reason(const Result<T, BridgeReason>& r) {
  EXPECT_FALSE(r.has_value());
  return r ? BridgeReason::kAccepted : r.error();
}

}  // namespace

TEST(Px4BridgeRos, LocalPositionConversion) {
  const auto r = to_px4_local_position(GoodLocalPosition());
  ASSERT_TRUE(r.has_value()) << to_string(r.error());
  EXPECT_EQ(r->sample.t.ns, 12'345'678'000) << "timestamp_sample (not timestamp) in us -> ns, SITL identity";
  EXPECT_DOUBLE_EQ(time::to_seconds(r->sample.t), 12.345678);
  EXPECT_TRUE(r->xy_valid);
  EXPECT_TRUE(r->z_valid);
  EXPECT_EQ(r->sample.p_ned_m, Eigen::Vector3d(1.5, -2.5, -3.0));
  EXPECT_DOUBLE_EQ(r->sample.yaw_rad, 0.75);
  EXPECT_EQ(r->sample.xy_reset_counter, 3);
  EXPECT_EQ(r->sample.z_reset_counter, 4);
  EXPECT_EQ(r->sample.heading_reset_counter, 255);
  EXPECT_EQ(r->sample.delta_xy_m, Eigen::Vector2d(0.25, -0.5));
  EXPECT_DOUBLE_EQ(r->sample.delta_z_m, 0.125);
  EXPECT_DOUBLE_EQ(r->sample.delta_heading_rad, -0.0625);

  px4_msgs::msg::VehicleLocalPosition invalid = GoodLocalPosition();
  invalid.xy_valid = false;
  invalid.z_valid = true;
  const auto flags = to_px4_local_position(invalid);
  ASSERT_TRUE(flags.has_value());
  EXPECT_FALSE(flags->xy_valid) << "the flags are copied; the bridge decides what an invalid sample means";
  EXPECT_TRUE(flags->z_valid);
}

TEST(Px4BridgeRos, OdometryAndHealthConversion) {
  const auto o = to_lio_odometry_in(GoodOdometry());
  ASSERT_TRUE(o.has_value()) << to_string(o.error());
  EXPECT_EQ(o->t.ns, 12'345'000'000);
  EXPECT_EQ(o->epoch, 7U) << "the sample's own epoch, never reset_counter (F14)";
  EXPECT_EQ(o->p_flu_m, Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(o->q_flu.coeffs(), Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));  // Eigen stores (x, y, z, w)
  EXPECT_EQ(o->v_flu_mps, Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_DOUBLE_EQ(o->pose_cov(0, 0), 0.01);
  EXPECT_DOUBLE_EQ(o->pose_cov(5, 5), 0.06);
  EXPECT_DOUBLE_EQ(o->pose_cov(0, 1), 0.001) << "row-major, (position, rotation)";
  EXPECT_DOUBLE_EQ(o->pose_cov(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(o->vel_cov(2, 2), 0.3);
  EXPECT_DOUBLE_EQ(o->vel_cov(1, 2), 0.002);
  EXPECT_EQ(o->quality, 80);

  const auto h = to_lio_health_in(GoodHealth());
  ASSERT_TRUE(h.has_value()) << to_string(h.error());
  EXPECT_EQ(h->t.ns, 12'000'000'500);
  EXPECT_EQ(h->state, LioHealthCode::kDegraded);
  EXPECT_EQ(h->epoch, 3U);
  for (std::uint8_t s = 0; s <= 4; ++s) {
    uavnav_interfaces::msg::LioHealth m = GoodHealth();
    m.state = s;
    const auto each = to_lio_health_in(m);
    ASSERT_TRUE(each.has_value()) << static_cast<int>(s);
    EXPECT_EQ(static_cast<std::uint8_t>(each->state), s) << "LioHealthCode equals the LioHealth constants";
  }
}

TEST(Px4BridgeRos, RejectsGarbagePx4Messages) {  // Review Focus 5
  // PX4 microseconds beyond the representable stamp: from_px4_us kOutOfRange -> kNotFinite.
  px4_msgs::msg::VehicleLocalPosition m = GoodLocalPosition();
  m.timestamp_sample = std::numeric_limits<std::uint64_t>::max();
  EXPECT_EQ(Reason(to_px4_local_position(m)), BridgeReason::kNotFinite);

  // Every number the alignment reads must be finite.
  using Local = px4_msgs::msg::VehicleLocalPosition;
  const std::array<void (*)(Local&), 10> corrupt = {
      [](Local& l) { l.heading = kNaNF; },
      [](Local& l) { l.x = kNaNF; },
      [](Local& l) { l.y = std::numeric_limits<float>::infinity(); },
      [](Local& l) { l.z = kNaNF; },
      [](Local& l) { l.delta_xy[0] = kNaNF; },
      [](Local& l) { l.delta_xy[1] = kNaNF; },
      [](Local& l) { l.delta_z = kNaNF; },
      [](Local& l) { l.delta_heading = std::numeric_limits<float>::infinity(); },
      [](Local& l) { l.heading = -std::numeric_limits<float>::infinity(); },
      [](Local& l) { l.z = std::numeric_limits<float>::infinity(); },
  };
  for (std::size_t i = 0; i < corrupt.size(); ++i) {
    px4_msgs::msg::VehicleLocalPosition bad = GoodLocalPosition();
    corrupt[i](bad);
    EXPECT_EQ(Reason(to_px4_local_position(bad)), BridgeReason::kNotFinite) << "case " << i;
  }

  // LIO odometry and health: stamps (nanosec 1e9, negative seconds) and the health state code.
  uavnav_interfaces::msg::LioOdometry o = GoodOdometry();
  o.stamp.nanosec = 1'000'000'000;
  EXPECT_EQ(Reason(to_lio_odometry_in(o)), BridgeReason::kNotFinite);
  o = GoodOdometry();
  o.stamp.sec = -1;
  EXPECT_EQ(Reason(to_lio_odometry_in(o)), BridgeReason::kNotFinite);

  uavnav_interfaces::msg::LioHealth h = GoodHealth();
  h.state = 9;
  EXPECT_EQ(Reason(to_lio_health_in(h)), BridgeReason::kNotFinite);
  h = GoodHealth();
  h.state = 5;
  EXPECT_EQ(Reason(to_lio_health_in(h)), BridgeReason::kNotFinite) << "the first code past RESTARTING";
  h = GoodHealth();
  h.stamp.nanosec = 1'000'000'000;
  EXPECT_EQ(Reason(to_lio_health_in(h)), BridgeReason::kNotFinite);
  h = GoodHealth();
  h.stamp.sec = -5;
  EXPECT_EQ(Reason(to_lio_health_in(h)), BridgeReason::kNotFinite);
}

TEST(Px4BridgeRos, VehicleOdometryFields) {
  EvSample s{};
  s.timestamp_sample_us = 12'345'000;
  s.pose_frame = kPoseFrameFrd;
  s.position = {1.0F, -2.0F, -3.0F};
  s.q_wxyz = {0.5F, 0.5F, -0.5F, -0.5F};
  s.velocity_frame = kVelocityFrameFrd;
  s.velocity = {0.1F, -0.2F, -0.3F};
  s.position_variance = {0.01F, 0.02F, 0.03F};
  s.orientation_variance = {0.001F, 0.002F, 0.003F};
  s.velocity_variance = {0.04F, 0.05F, 0.06F};
  s.reset_counter = 7;
  s.quality = 80;

  const px4_msgs::msg::VehicleOdometry m = to_vehicle_odometry(s, 12'400'000);
  EXPECT_EQ(m.pose_frame, px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD);
  EXPECT_EQ(m.velocity_frame, px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD);
  EXPECT_EQ(m.pose_frame, 2);
  EXPECT_EQ(m.velocity_frame, 2);
  EXPECT_EQ(m.timestamp, 12'400'000U);
  EXPECT_EQ(m.timestamp_sample, 12'345'000U);
  EXPECT_EQ(m.position, s.position);
  EXPECT_EQ(m.q, s.q_wxyz) << "w first, as PX4";
  EXPECT_EQ(m.velocity, s.velocity);
  for (const float w : m.angular_velocity) EXPECT_TRUE(std::isnan(w)) << "angular velocity is unknown: NaN";
  EXPECT_EQ(m.position_variance, s.position_variance);
  EXPECT_EQ(m.orientation_variance, s.orientation_variance);
  EXPECT_EQ(m.velocity_variance, s.velocity_variance);
  EXPECT_EQ(m.reset_counter, 7);
  EXPECT_EQ(m.quality, 80);
}

TEST(Px4BridgeRos, AlignmentMessageRawIsNanWhenAbsent) {
  AlignmentOutput out;
  out.state = AlignmentState::kFrozen;
  out.filtered = Pose4{1.0, 2.0, 3.0, 0.3};
  out.raw = std::nullopt;
  out.stamp = time::SensorTime{12'345'000'000};
  out.age_s = 1.25;
  out.residual_position_m = 0.125;
  out.residual_yaw_rad = -0.0625;

  const uavnav_interfaces::msg::Alignment m = to_msg(out);
  EXPECT_EQ(m.state, uavnav_interfaces::msg::Alignment::FROZEN);
  EXPECT_EQ(m.state, static_cast<std::uint8_t>(out.state)) << "the state value equals the enum value";
  EXPECT_EQ(m.stamp.sec, 12);
  EXPECT_EQ(m.stamp.nanosec, 345'000'000U);
  EXPECT_DOUBLE_EQ(m.x_m, 1.0);
  EXPECT_DOUBLE_EQ(m.y_m, 2.0);
  EXPECT_DOUBLE_EQ(m.z_m, 3.0);
  EXPECT_DOUBLE_EQ(m.yaw_rad, 0.3);
  EXPECT_TRUE(std::isnan(m.raw_x_m));
  EXPECT_TRUE(std::isnan(m.raw_y_m));
  EXPECT_TRUE(std::isnan(m.raw_z_m));
  EXPECT_TRUE(std::isnan(m.raw_yaw_rad));
  EXPECT_DOUBLE_EQ(m.age_s, 1.25);
  EXPECT_DOUBLE_EQ(m.residual_position_m, 0.125);
  EXPECT_DOUBLE_EQ(m.residual_yaw_rad, -0.0625);

  out.raw = Pose4{4.0, 5.0, 6.0, -0.4};
  const uavnav_interfaces::msg::Alignment with_raw = to_msg(out);
  EXPECT_DOUBLE_EQ(with_raw.raw_x_m, 4.0);
  EXPECT_DOUBLE_EQ(with_raw.raw_y_m, 5.0);
  EXPECT_DOUBLE_EQ(with_raw.raw_z_m, 6.0);
  EXPECT_DOUBLE_EQ(with_raw.raw_yaw_rad, -0.4);
  EXPECT_DOUBLE_EQ(with_raw.x_m, 1.0) << "filtered stays separate from raw";
}

TEST(Px4BridgeRos, AlignmentStateValuesEqualTheMessageConstants) {
  using Msg = uavnav_interfaces::msg::Alignment;
  const std::array<std::pair<AlignmentState, std::uint8_t>, 4> pairs = {{{AlignmentState::kInit, Msg::INIT},
                                                                         {AlignmentState::kValid, Msg::VALID},
                                                                         {AlignmentState::kFrozen, Msg::FROZEN},
                                                                         {AlignmentState::kInvalid, Msg::INVALID}}};
  for (const auto& [state, value] : pairs) {
    AlignmentOutput out;
    out.state = state;
    EXPECT_EQ(to_msg(out).state, value);
  }
}

TEST(Px4BridgeRos, AlignmentStampThatCannotBeAMessageStampIsTheEpoch) {
  AlignmentOutput out;
  out.stamp = time::SensorTime{-1};
  const uavnav_interfaces::msg::Alignment m = to_msg(out);
  EXPECT_EQ(m.stamp.sec, 0);
  EXPECT_EQ(m.stamp.nanosec, 0U);
}

// The strict rule (ros_conversion.hpp): a NaN position is refused as NOT_FINITE also when PX4 marks it invalid, so
// it is reported by the node as Px4SampleRejected NOT_FINITE and not as the core's PX4_POSITION_INVALID.
TEST(Px4BridgeRos, NanPositionWithValidityFlagFalseIsRefusedAsNotFinite) {
  px4_msgs::msg::VehicleLocalPosition m = GoodLocalPosition();
  m.xy_valid = false;
  m.x = kNaNF;
  EXPECT_EQ(Reason(to_px4_local_position(m)), BridgeReason::kNotFinite);

  m = GoodLocalPosition();
  m.z_valid = false;
  m.z = kNaNF;
  EXPECT_EQ(Reason(to_px4_local_position(m)), BridgeReason::kNotFinite);

  // Finite numbers with the flag false still convert (the core skips them with PX4_POSITION_INVALID).
  m = GoodLocalPosition();
  m.xy_valid = false;
  const auto r = to_px4_local_position(m);
  ASSERT_TRUE(r.has_value());
  EXPECT_FALSE(r->xy_valid);
}
