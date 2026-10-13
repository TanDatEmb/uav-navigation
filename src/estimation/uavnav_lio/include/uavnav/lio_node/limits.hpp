#pragma once

#include <string_view>

#include "uavnav/core/time.hpp"

// Tier-(a) constants of the lio node and its ingest layer (SYSTEM_DESIGN §6.2, D33/O20, O21). Changing one is a
// design-review change; each states how its value was derived.
namespace uavnav::lio_node::limits {

/// /lio/state period: one sample per 10 ms cell of SENSOR time (SYSTEM_DESIGN §3.3 "/lio/state 100 Hz", §6.4
/// "state 100 Hz", O21). With the 200 Hz Mid-360 IMU every second sample is published; the cells are absolute
/// (floor(t / period)), so jitter and gaps never shift the phase.
inline constexpr time::Duration kStatePeriod = time::milliseconds(10);

/// Longest wait of a scan for IMU coverage (newest accepted IMU stamp >= scan end), in ROS time (O20). One scan
/// period at 10 Hz: a scan still uncovered after it would be superseded by the next scan anyway, and the IMU is
/// then 20 samples (200 Hz) late, i.e. stalled. A scan that waited this long plus one ICP (20-50 ms, §3.5) is
/// still inside the 300 ms predictor buffer (uavnav/lio/limits.hpp kPredictorBufferSpan) and younger than the
/// beta lifecycle_gap_degraded_s (0.25 s). Expiry is strict: a scan that waited exactly this long still waits.
inline constexpr time::Duration kScanImuWaitMax = time::milliseconds(100);

/// A sensor stamp later than ROS now + this is refused (kStampInFuture, O20). In SITL sensor time IS ROS (sim)
/// time (D29), and a real message is stamped before it is received, so a correct stamp is never ahead; the bound
/// only refuses garbage before it reaches the core, where an accepted future IMU stamp would make every later
/// true sample out of order (F22). The value is O20's: a margin far above any clock skew between the stamping
/// and the receiving side, which in SITL share one clock.
inline constexpr time::Duration kMaxStampAhead = time::milliseconds(500);

// Frame ids (O20: tier (a)). The Gazebo model sets the sensor frames (src/sim/uav_simulation/models/lidar_mid360/
// model.sdf, gz_frame_id); lio_odom and base_link are the frames of /lio/state and /lio/odometry (LioState.msg).
inline constexpr std::string_view kWorldFrameId = "lio_odom";
inline constexpr std::string_view kBaseFrameId = "base_link";
inline constexpr std::string_view kLidarFrameId = "livox_frame";
inline constexpr std::string_view kImuFrameId = "livox_imu_frame";

}  // namespace uavnav::lio_node::limits
