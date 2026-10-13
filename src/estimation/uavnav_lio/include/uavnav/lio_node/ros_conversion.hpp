#pragma once

#include <cstdint>
#include <string_view>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/lio/outputs.hpp"
#include "uavnav/lio/types.hpp"
#include "uavnav/lio_node/ingest.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"
#include "uavnav_interfaces/msg/lio_odometry.hpp"
#include "uavnav_interfaces/msg/lio_state.hpp"

// Message conversion at the boundary of the lio node (SYSTEM_DESIGN §3.3, §6.3, §6.5; D33/O20, O21; Review
// Focus 5). Inputs: a message is either converted whole or refused with one ConversionError and changes nothing
// (no partial scan, no stats update). Outputs: total mappings of the ingest's outputs onto the v2 messages.
namespace uavnav::lio_node {

/// Why a sensor message was refused at the boundary (the node emits a rate-limited "LioInputRejected").
enum class ConversionError : std::uint8_t {
  kBadStamp,       ///< header.stamp: sec < 0 or nanosec >= 1e9 (rosutil::from_msg, time::from_stamp)
  kNotFinite,      ///< an IMU angular velocity or linear acceleration component is NaN or inf
  kWrongFrame,     ///< header.frame_id is not limits::kImuFrameId / kLidarFrameId
  kMalformedCloud  ///< the PointCloud2 layout cannot be read as float32 x, y, z points (see to_scan_input)
};

constexpr std::string_view to_string(ConversionError e) {
  switch (e) {
    case ConversionError::kBadStamp:
      return "BAD_STAMP";
    case ConversionError::kNotFinite:
      return "NOT_FINITE";
    case ConversionError::kWrongFrame:
      return "WRONG_FRAME";
    case ConversionError::kMalformedCloud:
      return "MALFORMED_CLOUD";
  }
  return "";
}

static_assert(ReasonEnum<ConversionError>);

/// Per-cloud point counts, written only when the cloud is converted.
struct CloudStats {
  std::uint32_t points_in;          ///< width * height
  std::uint32_t points_non_finite;  ///< dropped: any of x, y, z NaN or inf (Gazebo's no-returns are inf)
};

/// sensor_msgs/Imu -> lio::ImuInput. Checks, in order: the stamp (kBadStamp), the frame (kWrongFrame: the frame
/// id must be limits::kImuFrameId, the Gazebo IMU's gz_frame_id; any other frame would put the samples in an
/// unknown frame, so it fails closed), every gyro and accel component finite (kNotFinite). The stamp is sensor
/// time (D29); angular velocity in rad/s and linear acceleration in m/s^2 are copied as published (O20). The
/// orientation and every covariance are ignored (the LIO estimates attitude itself).
Result<lio::ImuInput, ConversionError> to_imu_input(const sensor_msgs::msg::Imu& msg);

/// sensor_msgs/PointCloud2 -> lio::ScanInput. Checks, in order:
///   kBadStamp       header.stamp (as for the IMU);
///   kWrongFrame     header.frame_id != limits::kLidarFrameId;
///   kMalformedCloud exactly one field each named x, y and z, each FLOAT32 with count 1 and offset + 4 <=
///                   point_step; point_step >= 12; little-endian data (is_bigendian false, the host order);
///                   row_step == width * point_step (no row padding); data.size() == width * height *
///                   point_step (computed without overflow). Other fields (intensity, ring) are ignored.
/// Points whose x, y or z is not finite are dropped and counted (stats.points_non_finite; O20: about 69% of a
/// Gazebo scan); the others keep their order. A Gazebo scan is one instant: start = end = stamp, no per-point
/// time (per_point_time false). An empty cloud is a valid scan (the core answers kScanEmpty). `stats` is written
/// only on success.
Result<lio::ScanInput, ConversionError> to_scan_input(const sensor_msgs::msg::PointCloud2& msg, CloudStats& stats);

/// /lio/state (O21): stamp = sensor time of the sample; the base_link pose in lio_odom; velocity and angular
/// velocity (the raw gyro) in base_link; pose_covariance and velocity_covariance all NaN (the output predictor
/// carries no covariance); reset_delta_* = the latched delta of the last applied restart (zero before one);
/// lio_state = the LioState value, equal to the LioHealth constants.
uavnav_interfaces::msg::LioState to_msg(const LioStateOut& s);

/// /lio/health: stamp = sensor time of the newest IMU sample evaluated; reason = the LioReason value and
/// reason_name = to_string(LioReason); output_error_{attitude_rad, velocity_mps, position_m} = the three
/// components of the output predictor's tracking error, in that order (output_predictor.hpp).
uavnav_interfaces::msg::LioHealth to_msg(const lio::HealthOutput& h);

/// /lio/odometry: stamp = scan end; base_link pose and base_link origin velocity in lio_odom; pose_covariance =
/// pose_cov row-major ((position, rotation) in lio_odom, the order the core's converters produce);
/// velocity_covariance = vel_cov row-major (lio_odom); quality as is (0..100).
uavnav_interfaces::msg::LioOdometry to_msg(const lio::OdometryOutput& o);

/// A sensor-time stamp as a message stamp. Every stamp the ingest outputs is a stamp that came in through
/// to_imu_input / to_scan_input (0 <= sec <= INT32_MAX), so rosutil::to_msg cannot fail for it; should it ever
/// (a negative or post-2038 time), the stamp is {0, 0}: the clock's epoch, older than any real sample.
builtin_interfaces::msg::Time stamp_msg(time::SensorTime t) noexcept;

}  // namespace uavnav::lio_node
