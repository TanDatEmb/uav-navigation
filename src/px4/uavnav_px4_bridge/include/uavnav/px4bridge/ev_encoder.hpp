#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/px4_time.hpp"

// External-vision (EV) encoder: LIO odometry in lio_odom (FLU) -> a PX4 VehicleOdometry-shaped
// sample in FRD (SYSTEM_DESIGN §4.1, D20, D21, D28, F14, F34). Pure, no ROS, no clock reads.
namespace uavnav::px4bridge {

inline constexpr std::uint8_t kPoseFrameFrd = 2;      ///< VehicleOdometry.POSE_FRAME_FRD
inline constexpr std::uint8_t kVelocityFrameFrd = 2;  ///< VehicleOdometry.VELOCITY_FRAME_FRD

/// Maps 1:1 onto px4_msgs::msg::VehicleOdometry (the node copies it in S1b).
struct EvSample {
  std::uint64_t timestamp_sample_us;  ///< [us] PX4 time of the measurement
  std::uint8_t pose_frame;            ///< always kPoseFrameFrd (2), never NED (F34)
  std::array<float, 3> position;      ///< [m] position of the sensor frame in lio_odom, FRD axes
  std::array<float, 4> q_wxyz;        ///< unit quaternion, FRD world <- FRD body, w >= 0
  std::uint8_t velocity_frame;        ///< always kVelocityFrameFrd (2)
  std::array<float, 3> velocity;      ///< [m/s] in FRD local axes
  std::array<float, 3> position_variance;     ///< [m^2], each > 0 and finite
  std::array<float, 3> orientation_variance;  ///< [rad^2], small-rotation variance in body FRD axes
  std::array<float, 3> velocity_variance;     ///< [m^2/s^2]
  std::uint8_t reset_counter;                 ///< epoch % 256, from the sample's own epoch (F14)
  std::int8_t quality;                        ///< input quality clamped to 127 (D28)
};

/// kNotTracking is evaluated first, so a lost or initialising LIO never yields a sample.
enum class EvReason : std::uint8_t { kEncoded, kNotTracking, kNonFiniteState, kBadCovariance, kTime };

constexpr std::string_view to_string(EvReason r) {
  switch (r) {
    case EvReason::kEncoded: return "ENCODED";
    case EvReason::kNotTracking: return "NOT_TRACKING";
    case EvReason::kNonFiniteState: return "NON_FINITE_STATE";
    case EvReason::kBadCovariance: return "BAD_COVARIANCE";
    case EvReason::kTime: return "TIME";
  }
  return "";
}
static_assert(ReasonEnum<EvReason>);

/// One LIO odometry sample in lio_odom (FLU world, FLU body), SI units.
struct EvInput {
  time::SensorTime t;           ///< sensor stamp of the measurement
  std::uint32_t epoch;          ///< LIO reset epoch of THIS sample (F14)
  bool tracking;                ///< LIO state is TRACKING for this sample
  Eigen::Vector3d p_m;          ///< position in lio_odom, FLU
  Eigen::Quaterniond q;         ///< q_world_body, lio_odom <- body, FLU, Hamilton
  Eigen::Vector3d v_mps;        ///< velocity in lio_odom axes, FLU
  Eigen::Matrix<double, 6, 6> pose_cov;  ///< [position(0:3), small-rotation in body axes(3:6)]
  Eigen::Matrix3d vel_cov;      ///< velocity covariance, lio_odom axes
  std::uint8_t quality;         ///< 0..100 (D28)
};

/// Encodes one sample. Checks run in this order and the first failure wins:
///  1. !tracking                                 -> kNotTracking
///  2. to_px4 / to_px4_us failure                -> kTime
///  3. non-finite p, q, v (also after the float32 cast), or q not unit
///     (|norm - 1| > 1e-3, zero norm included)   -> kNonFiniteState
///  4. any covariance diagonal entry (position, orientation, velocity block) that is not finite
///     or not > 0, also after the float32 cast   -> kBadCovariance
/// Only the diagonals of the blocks are used (the message carries variances); off-diagonal
/// entries, symmetric or not, finite or not, are ignored. Diagonals are invariant under the
/// FLU->FRD flip D S D^T, so the reported variances are the diagonals of the FRD blocks.
/// The quaternion is normalised, converted to FRD and canonicalised to w >= 0.
/// quality > 127 is clamped to 127 (the field is int8; D28 specifies 0..100).
Result<EvSample, EvReason> encode_ev(const EvInput& in, ClockMode mode);

}  // namespace uavnav::px4bridge
