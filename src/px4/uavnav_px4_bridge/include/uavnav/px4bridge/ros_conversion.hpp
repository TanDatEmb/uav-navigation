#pragma once

#include <cstdint>

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/px4bridge/alignment.hpp"
#include "uavnav/px4bridge/bridge.hpp"
#include "uavnav/px4bridge/ev_encoder.hpp"
#include "uavnav_interfaces/msg/alignment.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"
#include "uavnav_interfaces/msg/lio_odometry.hpp"

// Message conversion at the boundary of the px4_bridge node (SYSTEM_DESIGN §4.1, §6.3, §6.5; D33/O21, O22; Review
// Focus 5). This is the only header of the package that names a message type; the core (bridge.hpp and below)
// is ROS-free (test_core_target_has_no_ros). Inputs: a message is either converted whole or refused with
// BridgeReason::kNotFinite and changes nothing; the node counts the refusal and emits a rate-limited event.
// Outputs: total mappings of the core's outputs onto the messages.
namespace uavnav::px4bridge {

/// px4_msgs/VehicleLocalPosition -> Px4LocalPositionIn. The sample is stamped with timestamp_sample (the time of
/// the measurement, not `timestamp`, the time of the publication) through from_px4_us(limits::kClockMode), the
/// range-checked converter (SITL: PX4 microseconds are simulation time, D29). x, y, z (NED), heading, the three
/// reset counters and the deltas are copied; xy_valid and z_valid are copied as flags (the bridge decides what an
/// invalid sample means, O22). kNotFinite when the microseconds are out of range or when any of x, y, z, heading,
/// delta_xy, delta_z, delta_heading is NaN or inf, also while xy_valid or z_valid is false: a deliberate strict
/// rule (the sample is dropped and reported as Px4SampleRejected NOT_FINITE by the node, not as
/// PX4_POSITION_INVALID by the core). Neither path consumes the sample's reset counters, so a reset it carries is
/// applied by the next valid sample. Unverified: whether EKF2 ever publishes a NaN while a validity flag is false.
/// The SITL hover report (S1b Task 11) counts the NOT_FINITE refusals, expected 0; a non-zero count with the flags
/// false means the rule should check x, y, delta_xy only under xy_valid and z, delta_z only under z_valid.
Result<Px4LocalPositionIn, BridgeReason> to_px4_local_position(const px4_msgs::msg::VehicleLocalPosition& msg);

/// uavnav_interfaces/LioOdometry -> LioOdometryIn: stamp (SensorTime, from_msg), the sample's OWN epoch
/// (`epoch`, never `reset_counter`, which is the output predictor's counter; F14), base_link pose in lio_odom
/// (FLU), velocity, pose_covariance (6x6 row-major: position, rotation, lio_odom axes), velocity_covariance (3x3)
/// and quality as published. kNotFinite when the stamp is negative or has nanosec >= 1e9. A NaN in the pose or
/// the covariances is NOT a conversion refusal: the encoder (EvRejected: kNonFiniteState, kBadCovariance,
/// kBadQuaternion) and the alignment (kInputRejected) report it with their own events.
Result<LioOdometryIn, BridgeReason> to_lio_odometry_in(const uavnav_interfaces::msg::LioOdometry& msg);

/// uavnav_interfaces/LioHealth -> LioHealthIn: stamp, state (0..4, the LioHealthCode values equal the message
/// constants) and epoch. kNotFinite when the stamp is invalid or the state is past RESTARTING.
Result<LioHealthIn, BridgeReason> to_lio_health_in(const uavnav_interfaces::msg::LioHealth& msg);

/// EvSample -> px4_msgs/VehicleOdometry, field by field (EvSample maps 1:1): pose_frame and velocity_frame FRD
/// (2), position, q (w first), velocity, the three variances, reset_counter (the sample's epoch % 256) and
/// quality as encoded. timestamp = `timestamp_us` (the publication time), timestamp_sample = the sample's.
/// angular_velocity is NaN: the LIO does not publish a body rate in this message (PX4: NaN = invalid/unknown).
px4_msgs::msg::VehicleOdometry to_vehicle_odometry(const EvSample& ev, std::uint64_t timestamp_us);

/// AlignmentOutput -> /alignment: stamp = the SensorTime of the newest pair (the epoch {0, 0} when it cannot be a
/// message stamp: negative, or after 2038); state = the AlignmentState value (equal to the Alignment constants);
/// x_m..yaw_rad = the filtered T (lio_odom FRD -> PX4 local NED); raw_* = the instant T of the newest formed pair,
/// NaN when no pair was formed in the call that produced the output; age_s; residual_* measured AT THE VEHICLE
/// (alignment.hpp). `raw - filtered` includes the lever arm and is not a residual.
uavnav_interfaces::msg::Alignment to_msg(const AlignmentOutput& out);

}  // namespace uavnav::px4bridge
