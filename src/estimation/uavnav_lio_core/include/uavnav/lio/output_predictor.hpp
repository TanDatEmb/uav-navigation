#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/limits.hpp"
#include "uavnav/lio/types.hpp"  // EstimatorSnapshot

// EKF2-style output predictor (SYSTEM_DESIGN §3.3, F35, D20): the /lio/state output at the newest IMU
// time, tracking the ESKF, which runs at the (delayed) scan time. It follows PX4
// ekf2/EKF/output_predictor/output_predictor.cpp (calculateOutputStates, correctOutputStates):
//   - a strapdown integrator at the IMU rate, every output pushed into a ring buffer on the IMU time axis;
//   - at each correction, the error between the estimator snapshot and the buffered output AT THE
//     SNAPSHOT TIME; attitude corrected through a delta-angle term with gain 0.5 * dt_imu / delay, held
//     and applied on the following IMU steps for one delay (see difference 3); velocity and position
//     corrected by a PI complementary filter (gain = dt_corr / tau, integral 0.1 * gain^2) added to the
//     WHOLE buffer;
//   - averaged and clamped dt; a separate vertical channel (vertical velocity whose integral tracks the
//     vertical position); explicit resets with a counter and deltas.
// Three differences from PX4, all deliberate:
//   1. the delayed sample is looked up by timestamp (closest buffered output at or before s.t, within one
//      IMU period), not taken as the oldest sample (PX4's own TODO);
//   2. a correction older than the buffer is rejected (kOlderThanBuffer) and changes nothing;
//   3. the held attitude correction (delta angle err * 0.5 * dt / delay) is added only for `delay`
//      seconds of IMU steps after each applied correction (a countdown decremented by the clamped dt),
//      then zeroed; a new applied correction replaces it and restarts the window. PX4 refreshes this term
//      every EKF step (~100 Hz), so it never outlives its measurement; this predictor is corrected at scan
//      rate only, and holding it until the next scan would overshoot when delay < scan period and, if
//      corrections stop (LiDAR gap, stall, rejections), turn into a constant false angular rate of
//      0.5 * err / delay. When the correction interval is <= delay this is the same as PX4.
//
// Single-threaded: the caller serialises align / on_imu / on_correction / reset_to (no internal lock).
// Pure logic: no clock reads, threads, locks or ROS; on_imu and on_correction do not allocate. All time
// comes from the inputs' SensorTime stamps, which must come from the range-checked converters (the time
// subtractions here assume no int64 overflow, as in lifecycle.hpp).
//
// Frames and signs: "world" is the estimator's world (lio_odom). q_world_imu rotates IMU-frame vectors
// into the world. gravity_world is the gravity VECTOR in the world, e.g. (0, 0, -9.81) for a z-up world.
// delta_velocity_mps is the accelerometer's specific force integrated over the step, in the IMU frame, so
//   world acceleration = R(q_world_imu) * f + gravity_world,   i.e.   v += R * dv + gravity_world * dt.
// A body at rest in a z-up world therefore reports dv = (0, 0, +9.81 * dt). The vertical channel is the
// world z axis (the world is assumed gravity-aligned, as the estimator's is).
namespace uavnav::lio {

/// One IMU step: the integrals over (t - dt_s, t] of the angular rate and of the specific force, both in
/// the IMU (body) frame. Built from consecutive IMU samples.
struct ImuDelta {
  time::SensorTime t;  ///< stamp of the sample that ENDS the step; becomes the output time
  Eigen::Vector3d delta_angle_rad;
  Eigen::Vector3d delta_velocity_mps;
  double dt_s;
};

struct OutputSample {
  time::SensorTime t;
  Eigen::Quaterniond q_world_imu;
  Eigen::Vector3d v_world_mps;
  Eigen::Vector3d p_world_m;
  std::uint32_t reset_counter;
};

enum class PredictorReason : std::uint8_t { kApplied, kNotInitialized, kOlderThanBuffer, kNewerThanOutput, kNonFinite };

constexpr std::string_view to_string(PredictorReason r) {
  switch (r) {
    case PredictorReason::kApplied: return "APPLIED";
    case PredictorReason::kNotInitialized: return "NOT_INITIALIZED";
    case PredictorReason::kOlderThanBuffer: return "OLDER_THAN_BUFFER";
    case PredictorReason::kNewerThanOutput: return "NEWER_THAN_OUTPUT";
    case PredictorReason::kNonFinite: return "NON_FINITE";
  }
  return "";
}

static_assert(ReasonEnum<PredictorReason>);

/// new - old output at the newest output time. yaw_rad is the shortest-arc difference of the two yaw
/// angles (atan2(R(1,0), R(0,0)) of each attitude), wrapped to [-pi, pi].
struct ResetDelta {
  Eigen::Vector3d position_m;
  Eigen::Vector3d velocity_mps;
  double yaw_rad;
};

/// Tier (b), loaded from predictor_tau_vel_s / predictor_tau_pos_s (see config.hpp). Beta 0.25 s, 0.25 s.
struct PredictorConfig {
  time::Duration tau_vel;
  time::Duration tau_pos;
};

class OutputPredictor {
 public:
  explicit OutputPredictor(const PredictorConfig& cfg) noexcept;

  /// First init / epoch start: the output becomes the snapshot at s.t, the buffer holds only that sample,
  /// and the correction state (held delta-angle correction, PI integrals, averaged dts, tracking error) is
  /// reset. reset_counter is NOT changed. A snapshot with any non-finite value (or a zero quaternion) is
  /// ignored: the predictor stays exactly as it was (unaligned if it was).
  void align(const EstimatorSnapshot& s) noexcept;

  /// Integrates one IMU step and returns the new output (stamped d.t), or nullopt and NO change when:
  ///   - not aligned yet;
  ///   - any field is non-finite, dt_s < 0, or dt_s > kPredictorBufferSpan (not a consecutive-sample delta);
  ///   - d.t is not after the newest output time (duplicate or out of order).
  /// Integration uses the measured dt_s (the deltas are integrals over it, as in PX4); the averaged IMU dt
  /// that sets the attitude gain and the lookup tolerance use dt_s clamped to [kPredictorDtMin, kPredictorDtMax].
  std::optional<OutputSample> on_imu(const ImuDelta& d) noexcept;

  /// Corrects the output towards the estimator snapshot taken at s.t. Checks, in this order:
  ///   kNotInitialized   before the first align / reset_to;
  ///   kNonFinite        any non-finite value in s, or a zero quaternion;
  ///   kNewerThanOutput  s.t later than the newest output time (equal is accepted);
  ///   kOlderThanBuffer  s.t earlier than the oldest buffered sample or than newest - kPredictorBufferSpan
  ///                     (both limits inclusive: s.t equal to either is accepted), OR the closest
  ///                     buffered sample at or before s.t is more than one IMU period before it (an IMU
  ///                     gap: the history does not cover s.t). One IMU period = the last accepted dt_s
  ///                     clamped to [kPredictorDtMin, kPredictorDtMax]; a distance equal to it is accepted.
  /// Every rejection changes nothing. On kApplied the snapshot's biases and gravity are taken for the
  /// following IMU steps, the attitude correction is set for a window of `delay` seconds of IMU steps
  /// (replacing any previous one), and the vel/pos (and vertical) corrections are
  /// applied to every buffered sample, so the newest output moves by one small PI step, never to the
  /// snapshot; consecutive outputs stay continuous.
  Result<void, PredictorReason> on_correction(const EstimatorSnapshot& s) noexcept;

  /// Explicit reset: reset_counter + 1, returns new - old newest output. If s.t is matched in the buffer
  /// (same rule as on_correction), every buffered sample is shifted so that the matched one equals s
  /// exactly (PX4 style: attitude by q_s * q_matched^-1 applied in the world, velocity and position by
  /// constant offsets); the time axis and the correction state are kept. Otherwise (s.t newer than the
  /// output, older than the buffer, or in a gap) the predictor restarts at s like align(), so the output
  /// time becomes s.t and IMU samples not after it are ignored. Before any align it aligns to s and returns
  /// a zero delta (there is no old output). A non-finite snapshot is ignored: zero delta, nothing changes.
  /// The matched branch also clears the held attitude correction (it was measured before the shift).
  ///
  /// Caution: a snapshot the buffer cannot match but that is OLDER than the newest output (stale, or
  /// inside an IMU gap) restarts the output at that older stamp, i.e. rewinds it. IMU deltas already
  /// consumed are not replayed, so the motion since s.t is lost and shows up in the ResetDelta. Callers
  /// must pass a snapshot the buffer can match (or one at/after the newest output time).
  ResetDelta reset_to(const EstimatorSnapshot& s) noexcept;

  /// (attitude rad, velocity m/s, position m): the magnitudes of the errors snapshot - buffered output
  /// observed at the matched sample by the last APPLIED correction, before that correction was applied
  /// (PX4 _output_tracking_error). Attitude is |2 * vec(q_s^-1 * q_out)| (sign-fixed). Zero after align or
  /// a restarting reset; rejected corrections do not change it.
  Eigen::Vector3d tracking_error() const noexcept { return tracking_error_; }

  /// Number of reset_to calls so far (the value the next output sample carries). 0 before any reset.
  std::uint32_t reset_counter() const noexcept { return reset_counter_; }

  /// The newest buffered output, including the shifts of applied corrections and resets since on_imu
  /// returned it; nullopt before align.
  std::optional<OutputSample> latest() const noexcept {
    if (size_ == 0) return std::nullopt;
    return newest().out;
  }

  /// Vertical velocity (world z, m/s) of the separate vertical channel at the newest output (PX4
  /// z_deriv): unlike v_world_mps.z(), its integral is the channel's vertical position, which tracks
  /// the estimator's p.z through velocity corrections only. 0 before align.
  double vertical_velocity_mps() const noexcept;

 private:
  struct Entry {
    OutputSample out;
    double vert_vel;  ///< vertical channel velocity (world z)
    double vert_pos;  ///< vertical channel position, integral of the main channel's dz (PX4 vert_vel_integ)
    double dt;        ///< integration step that produced this entry (0 for the aligned sample)
  };

  Entry& at(std::size_t i) noexcept { return ring_[(head_ + i) % ring_.size()]; }
  const Entry& at(std::size_t i) const noexcept { return ring_[(head_ + i) % ring_.size()]; }
  const Entry& newest() const noexcept { return at(size_ - 1); }
  void push(const Entry& e) noexcept;
  void restart(const EstimatorSnapshot& s) noexcept;
  /// Index (0 = oldest) of the sample matched to t, or the rejection reason (never kApplied).
  Result<std::size_t, PredictorReason> match(time::SensorTime t) const noexcept;

  PredictorConfig cfg_;
  std::array<Entry, limits::kPredictorBufferCapacity> ring_{};  // fixed storage, no heap
  std::size_t head_{0};                                        // index of the oldest entry
  std::size_t size_{0};                                        // 0 <=> not aligned
  std::uint32_t reset_counter_{0};

  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world_{Eigen::Vector3d::Zero()};

  double dt_imu_avg_s_;                  // averaged clamped IMU dt (PX4 _dt_update_states_avg)
  double dt_corr_avg_s_;                 // averaged clamped correction interval (PX4 _dt_correct_states_avg)
  double imu_period_s_;                  // last accepted dt_s, clamped: the lookup tolerance
  std::optional<time::SensorTime> last_correction_t_;

  Eigen::Vector3d delta_angle_corr_{Eigen::Vector3d::Zero()};  // added to IMU delta angles inside its window
  double att_corr_remaining_s_{0.0};                           // remaining window of delta_angle_corr_
  Eigen::Vector3d vel_err_integ_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_err_integ_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tracking_error_{Eigen::Vector3d::Zero()};
};

}  // namespace uavnav::lio
