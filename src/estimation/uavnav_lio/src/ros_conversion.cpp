#include "uavnav/lio_node/ros_conversion.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio_node/limits.hpp"
#include "uavnav/ros/time_msg.hpp"

namespace uavnav::lio_node {

namespace {

using sensor_msgs::msg::PointField;

Result<time::SensorTime, ConversionError> sensor_stamp(const builtin_interfaces::msg::Time& stamp) {
  const auto t = rosutil::from_msg<time::SensorTag>(stamp);
  if (!t) return std::unexpected(ConversionError::kBadStamp);
  return *t;
}

bool finite(const geometry_msgs::msg::Vector3& v) noexcept {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/// The byte offset of the one field `name` when it is a readable float32 scalar inside the point; nullopt when
/// it is missing, duplicated, of another type or count, or reads past point_step.
std::optional<std::uint32_t> float_field(const sensor_msgs::msg::PointCloud2& msg, std::string_view name) {
  std::optional<std::uint32_t> offset;
  for (const PointField& f : msg.fields) {
    if (f.name != name) continue;
    if (offset) return std::nullopt;  // duplicated
    if (f.datatype != PointField::FLOAT32 || f.count != 1) return std::nullopt;
    if (static_cast<std::uint64_t>(f.offset) + sizeof(float) > msg.point_step) return std::nullopt;
    offset = f.offset;
  }
  return offset;
}

/// Two 4-byte fields at offsets a and b do not share a byte.
constexpr bool disjoint(std::uint32_t a, std::uint32_t b) noexcept {
  return a >= b ? a - b >= sizeof(float) : b - a >= sizeof(float);
}

float read_float(const std::uint8_t* p) noexcept {
  float v;
  std::memcpy(&v, p, sizeof v);  // unaligned, little-endian data on a little-endian host (checked below)
  return v;
}

geometry_msgs::msg::Vector3 vec(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Vector3 m;
  m.x = v.x();
  m.y = v.y();
  m.z = v.z();
  return m;
}

geometry_msgs::msg::Pose pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Pose m;
  m.position.x = p.x();
  m.position.y = p.y();
  m.position.z = p.z();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  m.orientation.w = q.w();
  return m;
}

/// Row-major copy of an Eigen matrix (column-major storage) into a message array.
template <int N, class Array>
void row_major(const Eigen::Matrix<double, N, N>& m, Array& out) {
  static_assert(std::tuple_size_v<Array> == static_cast<std::size_t>(N * N));
  for (int r = 0; r < N; ++r) {
    for (int c = 0; c < N; ++c) out[static_cast<std::size_t>(r * N + c)] = m(r, c);
  }
}

static_assert(std::endian::native == std::endian::little, "PointCloud2 decoding assumes a little-endian host");

}  // namespace

builtin_interfaces::msg::Time stamp_msg(time::SensorTime t) noexcept {
  const auto m = rosutil::to_msg(t);
  return m ? *m : builtin_interfaces::msg::Time{};
}

Result<lio::ImuInput, ConversionError> to_imu_input(const sensor_msgs::msg::Imu& msg) {
  const auto t = sensor_stamp(msg.header.stamp);
  if (!t) return std::unexpected(t.error());
  if (msg.header.frame_id != limits::kImuFrameId) return std::unexpected(ConversionError::kWrongFrame);
  if (!finite(msg.angular_velocity) || !finite(msg.linear_acceleration)) {
    return std::unexpected(ConversionError::kNotFinite);
  }
  const auto& w = msg.angular_velocity;
  const auto& a = msg.linear_acceleration;
  return lio::ImuInput{*t, Eigen::Vector3d(w.x, w.y, w.z), Eigen::Vector3d(a.x, a.y, a.z)};
}

Result<lio::ScanInput, ConversionError> to_scan_input(const sensor_msgs::msg::PointCloud2& msg, CloudStats& stats) {
  const auto t = sensor_stamp(msg.header.stamp);
  if (!t) return std::unexpected(t.error());
  if (msg.header.frame_id != limits::kLidarFrameId) return std::unexpected(ConversionError::kWrongFrame);

  const auto ox = float_field(msg, "x");
  const auto oy = float_field(msg, "y");
  const auto oz = float_field(msg, "z");
  if (!ox || !oy || !oz || !disjoint(*ox, *oy) || !disjoint(*ox, *oz) || !disjoint(*oy, *oz) ||
      msg.point_step < 3 * sizeof(float) || msg.is_bigendian) {
    return std::unexpected(ConversionError::kMalformedCloud);
  }
  // Sizes in 64 bits: width * point_step < 2^64; width * height * point_step may not be, so check it by division.
  const std::uint64_t row = static_cast<std::uint64_t>(msg.width) * msg.point_step;
  if (msg.row_step != row) return std::unexpected(ConversionError::kMalformedCloud);
  if (msg.height != 0 && row > std::numeric_limits<std::uint64_t>::max() / msg.height) {
    return std::unexpected(ConversionError::kMalformedCloud);
  }
  if (msg.data.size() != row * msg.height) return std::unexpected(ConversionError::kMalformedCloud);

  // data.size() fits in memory, so the point count fits in a size_t; it fits the uint32 stats as well whenever
  // point_step >= 12 and the data is below 48 GiB.
  const std::size_t n = msg.data.size() / msg.point_step;
  if (n > std::numeric_limits<std::uint32_t>::max()) return std::unexpected(ConversionError::kMalformedCloud);
  lio::ScanInput scan{*t, *t, {}, false};
  scan.points.reserve(n);
  std::uint32_t non_finite = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t* p = msg.data.data() + i * msg.point_step;
    const Eigen::Vector3f xyz(read_float(p + *ox), read_float(p + *oy), read_float(p + *oz));
    if (!xyz.allFinite()) {
      ++non_finite;
      continue;
    }
    uav::nav::lio::LidarPoint point;
    point.position_lidar_m = xyz;
    scan.points.push_back(point);
  }
  stats = CloudStats{static_cast<std::uint32_t>(n), non_finite};
  return scan;
}

uavnav_interfaces::msg::LioState to_msg(const LioStateOut& s) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  uavnav_interfaces::msg::LioState m;
  m.stamp = stamp_msg(s.t);
  m.epoch = s.epoch;
  m.reset_counter = s.reset_counter;
  m.lio_state = static_cast<std::uint8_t>(s.state);
  m.pose = pose(s.p_world_base, s.q_world_base);
  m.velocity = vec(s.v_base_mps);
  m.angular_velocity = vec(s.w_base_rad_s);
  m.pose_covariance.fill(kNaN);
  m.velocity_covariance.fill(kNaN);
  m.reset_delta_position = vec(s.last_reset.position_m);
  m.reset_delta_velocity = vec(s.last_reset.velocity_mps);
  m.reset_delta_yaw = s.last_reset.yaw_rad;
  return m;
}

uavnav_interfaces::msg::LioHealth to_msg(const lio::HealthOutput& h) {
  uavnav_interfaces::msg::LioHealth m;
  m.stamp = stamp_msg(h.t);
  m.state = static_cast<std::uint8_t>(h.state);
  m.epoch = h.epoch;
  m.reason = static_cast<std::uint16_t>(h.reason);
  m.reason_name = std::string(lio::to_string(h.reason));
  m.translation_min_eigenvalue = h.translation_min_eigenvalue;
  m.rotation_min_eigenvalue = h.rotation_min_eigenvalue;
  m.correction_age_s = h.correction_age_s;
  m.output_error_attitude_rad = h.output_tracking_error.x();
  m.output_error_velocity_mps = h.output_tracking_error.y();
  m.output_error_position_m = h.output_tracking_error.z();
  return m;
}

uavnav_interfaces::msg::LioOdometry to_msg(const lio::OdometryOutput& o) {
  uavnav_interfaces::msg::LioOdometry m;
  m.stamp = stamp_msg(o.t);
  m.epoch = o.epoch;
  m.reset_counter = o.reset_counter;
  m.pose = pose(o.p_world_m, o.q_world_base);
  m.velocity = vec(o.v_world_mps);
  row_major(o.pose_cov, m.pose_covariance);
  row_major(o.vel_cov, m.velocity_covariance);
  m.quality = o.quality;
  return m;
}

// LioState.lio_state and LioHealth.state carry the LioState value: the message constants must match it.
using HealthMsg = uavnav_interfaces::msg::LioHealth;
static_assert(HealthMsg::INITIALIZING == static_cast<std::uint8_t>(lio::LioState::kInitializing));
static_assert(HealthMsg::TRACKING == static_cast<std::uint8_t>(lio::LioState::kTracking));
static_assert(HealthMsg::DEGRADED == static_cast<std::uint8_t>(lio::LioState::kDegraded));
static_assert(HealthMsg::LOST == static_cast<std::uint8_t>(lio::LioState::kLost));
static_assert(HealthMsg::RESTARTING == static_cast<std::uint8_t>(lio::LioState::kRestarting));

}  // namespace uavnav::lio_node
