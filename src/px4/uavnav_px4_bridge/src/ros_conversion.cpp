#include "uavnav/px4bridge/ros_conversion.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

#include "builtin_interfaces/msg/time.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/limits.hpp"
#include "uavnav/px4bridge/px4_time.hpp"
#include "uavnav/ros/time_msg.hpp"

namespace uavnav::px4bridge {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Result<time::SensorTime, BridgeReason> sensor_stamp(const builtin_interfaces::msg::Time& stamp) {
  const auto t = rosutil::from_msg<time::SensorTag>(stamp);
  if (!t) return std::unexpected(BridgeReason::kNotFinite);
  return *t;
}

bool finite(float v) noexcept { return std::isfinite(v); }

// The Alignment.msg constants are the AlignmentState values (the node copies the state as a number).
static_assert(static_cast<std::uint8_t>(AlignmentState::kInit) == uavnav_interfaces::msg::Alignment::INIT);
static_assert(static_cast<std::uint8_t>(AlignmentState::kValid) == uavnav_interfaces::msg::Alignment::VALID);
static_assert(static_cast<std::uint8_t>(AlignmentState::kFrozen) == uavnav_interfaces::msg::Alignment::FROZEN);
static_assert(static_cast<std::uint8_t>(AlignmentState::kInvalid) == uavnav_interfaces::msg::Alignment::INVALID);
static_assert(static_cast<std::uint8_t>(LioHealthCode::kInitializing) == uavnav_interfaces::msg::LioHealth::INITIALIZING);
static_assert(static_cast<std::uint8_t>(LioHealthCode::kTracking) == uavnav_interfaces::msg::LioHealth::TRACKING);
static_assert(static_cast<std::uint8_t>(LioHealthCode::kDegraded) == uavnav_interfaces::msg::LioHealth::DEGRADED);
static_assert(static_cast<std::uint8_t>(LioHealthCode::kLost) == uavnav_interfaces::msg::LioHealth::LOST);
static_assert(static_cast<std::uint8_t>(LioHealthCode::kRestarting) == uavnav_interfaces::msg::LioHealth::RESTARTING);

}  // namespace

Result<Px4LocalPositionIn, BridgeReason> to_px4_local_position(const px4_msgs::msg::VehicleLocalPosition& msg) {
  // timestamp_sample is an external microsecond count: only the range-checked converter may read it (INV-TIME-1).
  const auto t = from_px4_us(msg.timestamp_sample, limits::kClockMode);
  if (!t) return std::unexpected(BridgeReason::kNotFinite);
  if (!(finite(msg.x) && finite(msg.y) && finite(msg.z) && finite(msg.heading) && finite(msg.delta_xy[0]) &&
        finite(msg.delta_xy[1]) && finite(msg.delta_z) && finite(msg.delta_heading))) {
    return std::unexpected(BridgeReason::kNotFinite);
  }
  Px4PoseSample s;
  s.t = *t;
  s.p_ned_m = Eigen::Vector3d(msg.x, msg.y, msg.z);
  s.yaw_rad = msg.heading;
  s.xy_reset_counter = msg.xy_reset_counter;
  s.z_reset_counter = msg.z_reset_counter;
  s.heading_reset_counter = msg.heading_reset_counter;
  s.delta_xy_m = Eigen::Vector2d(msg.delta_xy[0], msg.delta_xy[1]);
  s.delta_z_m = msg.delta_z;
  s.delta_heading_rad = msg.delta_heading;
  return Px4LocalPositionIn{s, msg.xy_valid, msg.z_valid};
}

Result<LioOdometryIn, BridgeReason> to_lio_odometry_in(const uavnav_interfaces::msg::LioOdometry& msg) {
  const auto t = sensor_stamp(msg.stamp);
  if (!t) return std::unexpected(t.error());
  LioOdometryIn in;
  in.t = *t;
  in.epoch = msg.epoch;
  in.p_flu_m = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  // Eigen's (w, x, y, z) constructor; the quaternion is NOT normalised here (a non-rotation is the core's to refuse).
  in.q_flu = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z);
  in.v_flu_mps = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = 0; c < 6; ++c) in.pose_cov(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = msg.pose_covariance[r * 6 + c];
  }
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) in.vel_cov(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = msg.velocity_covariance[r * 3 + c];
  }
  in.quality = msg.quality;
  return in;
}

Result<LioHealthIn, BridgeReason> to_lio_health_in(const uavnav_interfaces::msg::LioHealth& msg) {
  const auto t = sensor_stamp(msg.stamp);
  if (!t) return std::unexpected(t.error());
  if (msg.state > uavnav_interfaces::msg::LioHealth::RESTARTING) return std::unexpected(BridgeReason::kNotFinite);
  return LioHealthIn{*t, static_cast<LioHealthCode>(msg.state), msg.epoch};
}

px4_msgs::msg::VehicleOdometry to_vehicle_odometry(const EvSample& ev, std::uint64_t timestamp_us) {
  px4_msgs::msg::VehicleOdometry m;
  m.timestamp = timestamp_us;
  m.timestamp_sample = ev.timestamp_sample_us;
  m.pose_frame = ev.pose_frame;
  m.position = ev.position;
  m.q = ev.q_wxyz;
  m.velocity_frame = ev.velocity_frame;
  m.velocity = ev.velocity;
  m.angular_velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN()};
  m.position_variance = ev.position_variance;
  m.orientation_variance = ev.orientation_variance;
  m.velocity_variance = ev.velocity_variance;
  m.reset_counter = ev.reset_counter;
  m.quality = ev.quality;
  return m;
}

uavnav_interfaces::msg::Alignment to_msg(const AlignmentOutput& out) {
  uavnav_interfaces::msg::Alignment m;
  // A stamp that cannot be a message stamp (never from the bridge: the inputs are range-checked) is the epoch.
  const auto stamp = rosutil::to_msg(out.stamp);
  m.stamp = stamp ? *stamp : builtin_interfaces::msg::Time();
  m.state = static_cast<std::uint8_t>(out.state);
  m.x_m = out.filtered.x_m;
  m.y_m = out.filtered.y_m;
  m.z_m = out.filtered.z_m;
  m.yaw_rad = out.filtered.yaw_rad;
  m.raw_x_m = out.raw ? out.raw->x_m : kNaN;
  m.raw_y_m = out.raw ? out.raw->y_m : kNaN;
  m.raw_z_m = out.raw ? out.raw->z_m : kNaN;
  m.raw_yaw_rad = out.raw ? out.raw->yaw_rad : kNaN;
  m.age_s = out.age_s;
  m.residual_position_m = out.residual_position_m;
  m.residual_yaw_rad = out.residual_yaw_rad;
  return m;
}

}  // namespace uavnav::px4bridge
