#pragma once

#include <string_view>

#include "uavnav/core/time.hpp"
#include "uavnav/lio/limits.hpp"  // kGapDegradedMin, kScanPeriod (the core's lifecycle bounds)

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

/// A sensor stamp later than ROS now + this is refused (kStampInFuture, O20, D40, AR4). The refusal is a technical
/// impossibility, not an age rule: an accepted IMU stamp ahead of the true time by delta becomes the newest IMU
/// stamp, every true sample of the next delta is then out of order (F22), and the lifecycle's gap rule sees a LiDAR
/// gap longer by delta. Derivation: in SITL sensor time IS ROS (sim) time (D29) and a message is stamped before it
/// is received, so the skew is 0 plus transport jitter (measured in two S1b probe runs: /lidar/imu stamp - newest
/// /clock at arrival, max 0 ms over ~9.7k samples each); 50 ms covers that jitter. Harm bound: kGapDegradedMin -
/// kScanPeriod = 0.1 s, the margin a healthy 10 Hz stream leaves under the smallest allowed
/// lifecycle_gap_degraded_s, so an accepted delta cannot by itself cause DEGRADED (static_assert below). The
/// hardware clock mode (Livox, PTP) is P8, not this constant.
inline constexpr time::Duration kMaxStampAhead = time::milliseconds(50);
static_assert(kMaxStampAhead < ::uavnav::lio::limits::kGapDegradedMin - ::uavnav::lio::limits::kScanPeriod,
              "an accepted future stamp must not be able to cause DEGRADED by itself (D40, AR4)");

// Frame ids (O20: tier (a)). The Gazebo model sets the sensor frames (src/sim/uav_simulation/models/lidar_mid360/
// model.sdf, gz_frame_id); lio_odom and base_link are the frames of /lio/state and /lio/odometry (LioState.msg).
inline constexpr std::string_view kWorldFrameId = "lio_odom";
inline constexpr std::string_view kBaseFrameId = "base_link";
inline constexpr std::string_view kLidarFrameId = "livox_frame";
inline constexpr std::string_view kImuFrameId = "livox_imu_frame";

}  // namespace uavnav::lio_node::limits
