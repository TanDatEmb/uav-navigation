#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Core>

namespace px4_navigation_external_mode {

// This file is deliberately ROS- and PX4-message-free.  It is the typed,
// deterministic seam between NavigationMode's paired snapshots and the later
// Level A runtime integration.  The helper never owns temporal state and never
// returns a default-looking setpoint on failure.
namespace tracking_adapter {

enum class Mode : std::uint8_t { kOff = 0U, kShadow = 1U, kLevelA = 2U };

// This is a pure setpoint-boundary candidate.  It is intentionally not wired
// into NavigationMode until the control review closes the timing, reset and
// role-handoff evidence.
enum class SetpointBoundary : std::uint8_t {
  kPositionVelocityAcceleration = 0U,
  kVelocityOnly = 1U,
};

enum class FailureReason : std::uint8_t {
  kNone = 0U,
  kPolicyDisabled,
  kInvalidPolicy,
  kInvalidReference,
  kInvalidLioState,
  kInvalidPx4State,
  kInvalidTiming,
  kResetMismatch,
  kUnsupportedFrame,
  kRelativeHeadingInvalid,
  kDeadReckoning,
  kNonFiniteOutput,
};

enum class ReferenceFrame : std::uint8_t { kLioEnu = 0U };

enum class CommandRole : std::uint8_t {
  kMain = 0U,
  kBackup = 1U,
  kEmergency = 2U,
  kPositionHold = 3U,
};

struct Identity final {
  std::uint64_t localization_epoch{0U};
  std::string mission_id;
  std::uint32_t waypoint_index{0U};
  std::uint64_t goal_epoch{0U};
  std::uint64_t request_id{0U};
  std::uint64_t bundle_generation{0U};
  std::uint64_t sample_id{0U};
  CommandRole role{CommandRole::kMain};
  std::int64_t reference_sample_time_ns{0};
  std::int64_t lease_valid_until_ns{0};
};

struct Reference final {
  ReferenceFrame frame{ReferenceFrame::kLioEnu};
  Identity identity;
  Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_enu{Eigen::Vector3d::Zero()};
  double yaw_enu{0.0};
  double yaw_rate_enu_rad_s{0.0};
};

struct LioState final {
  Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  double yaw_enu{0.0};
  bool position_valid{false};
  bool velocity_valid{false};
  bool orientation_valid{false};
  bool navigation_valid{false};
  bool covariance_valid{false};
  bool observability_valid{false};
  bool correction_fresh{false};
  bool propagation_valid{false};
  bool relative_heading_valid{false};
  bool tilt_valid{false};
  bool extrinsic_valid{false};
  std::uint64_t localization_epoch{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_steady_ns{0};
};

struct ResetCounters final {
  std::uint8_t xy{0U};
  std::uint8_t z{0U};
  std::uint8_t vxy{0U};
  std::uint8_t vz{0U};
  std::uint8_t heading{0U};

  [[nodiscard]] friend bool operator==(const ResetCounters& lhs,
                                       const ResetCounters& rhs) noexcept {
    return lhs.xy == rhs.xy && lhs.z == rhs.z && lhs.vxy == rhs.vxy &&
           lhs.vz == rhs.vz && lhs.heading == rhs.heading;
  }
};

struct RawPx4State final {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  double yaw_ned{0.0};
  std::array<bool, 3U> position_valid{{false, false, false}};
  std::array<bool, 3U> velocity_valid{{false, false, false}};
  bool heading_valid{false};
  bool heading_good_for_control{false};
  bool dead_reckoning{false};
  double heading_variance_rad2{0.0};
  std::uint64_t timestamp_us{0U};
  std::uint64_t timestamp_sample_us{0U};
  std::int64_t receive_steady_ns{0};
  ResetCounters reset_counters;
};

struct TimingWitness final {
  static constexpr std::uint8_t kConservativeBoundModelV1{1U};

  // These IDs are copied from the exact snapshots used to build this timing
  // witness.  They are not diagnostic labels: adapt() rejects a mismatched
  // tuple so a fresh raw state cannot be paired with an older LIO/reference.
  std::uint64_t reference_sample_id{0U};
  std::uint64_t lio_localization_epoch{0U};
  std::uint64_t lio_sequence{0U};
  std::uint64_t px4_timestamp_us{0U};
  std::uint64_t px4_timestamp_sample_us{0U};
  std::uint64_t clock_mapping_generation{0U};
  std::uint8_t conservative_bound_model{0U};
  bool common_time_contract_valid{false};
  double clock_mapping_uncertainty_s{0.0};
  std::int64_t expected_reference_use_time_ns{0};
  double reference_age_s{0.0};
  double pair_skew_s{0.0};
  double lio_source_age_s{0.0};
  double lio_receive_age_s{0.0};
  double px4_source_age_s{0.0};
  double px4_receive_age_s{0.0};
  double predicted_anchor_age_s{0.0};
  double output_transport_age_s{0.0};
  double px4_consume_age_s{0.0};
  double total_bound_s{0.0};
};

struct Policy final {
  Mode mode{Mode::kOff};
  SetpointBoundary boundary{SetpointBoundary::kPositionVelocityAcceleration};
  std::string experiment_id;
  // Level A intentionally does not mix a raw PX4 velocity feedback term into
  // the planner reference.  Keep this explicit so a future Level B cannot be
  // enabled by accidentally changing a default.
  double velocity_lambda{0.0};
  double lio_position_feedback_gain_s_inv{0.0};
  std::optional<double> maximum_velocity_mps;
  std::optional<double> maximum_timing_bound_s;
  std::optional<double> maximum_reference_age_s;
  std::optional<ResetCounters> expected_px4_reset_counters;
  std::optional<std::uint64_t> expected_lio_localization_epoch;
  bool reject_dead_reckoning{true};
};

struct Witness final {
  Identity identity;
  std::string experiment_id;
  Eigen::Matrix3d rotation_lio_enu_to_px4_ned{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d error_lio_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d error_adapter_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_command_lio_enu{Eigen::Vector3d::Zero()};
  bool velocity_limited{false};
  ResetCounters px4_reset_counters;
  TimingWitness timing;
  Mode mode{Mode::kOff};
  SetpointBoundary boundary{SetpointBoundary::kPositionVelocityAcceleration};
};

struct AdaptedReference final {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned{Eigen::Vector3d::Zero()};
  double yaw_ned{0.0};
  double yaw_rate_ned_rad_s{0.0};
  Witness witness;
};

struct Result final {
  std::optional<AdaptedReference> output;
  FailureReason failure{FailureReason::kNone};

  [[nodiscard]] bool success() const noexcept {
    return output.has_value() && failure == FailureReason::kNone;
  }
};

namespace detail {

[[nodiscard]] inline bool finite(const double value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] inline bool finite(const Eigen::Vector3d& value) noexcept {
  return value.allFinite();
}

[[nodiscard]] inline double wrapPi(const double angle) noexcept {
  if (!finite(angle)) return std::numeric_limits<double>::quiet_NaN();
  return std::remainder(angle, 2.0 * std::numbers::pi_v<double>);
}

[[nodiscard]] inline bool validNonNegative(const double value) noexcept {
  return finite(value) && value >= 0.0;
}

[[nodiscard]] inline bool validPositive(const double value) noexcept {
  return finite(value) && value > 0.0;
}

[[nodiscard]] inline bool validIdentity(const Identity& identity) noexcept {
  return identity.localization_epoch > 0U && !identity.mission_id.empty() &&
         identity.goal_epoch > 0U && identity.request_id > 0U &&
         identity.bundle_generation > 0U && identity.sample_id > 0U &&
         identity.reference_sample_time_ns > 0 &&
         identity.lease_valid_until_ns >= identity.reference_sample_time_ns;
}

[[nodiscard]] inline bool validTiming(const TimingWitness& timing) noexcept {
  return timing.reference_sample_id > 0U && timing.lio_localization_epoch > 0U &&
         timing.lio_sequence > 0U && timing.px4_timestamp_us > 0U &&
         timing.px4_timestamp_sample_us > 0U &&
         timing.clock_mapping_generation > 0U &&
         timing.conservative_bound_model == TimingWitness::kConservativeBoundModelV1 &&
         timing.common_time_contract_valid &&
         timing.expected_reference_use_time_ns > 0 &&
         validNonNegative(timing.clock_mapping_uncertainty_s) &&
         validNonNegative(timing.reference_age_s) &&
         validNonNegative(timing.pair_skew_s) &&
         validNonNegative(timing.lio_source_age_s) &&
         validNonNegative(timing.lio_receive_age_s) &&
         validNonNegative(timing.px4_source_age_s) &&
         validNonNegative(timing.px4_receive_age_s) &&
         validNonNegative(timing.predicted_anchor_age_s) &&
         validNonNegative(timing.output_transport_age_s) &&
         validNonNegative(timing.px4_consume_age_s) &&
         validNonNegative(timing.total_bound_s);
}

[[nodiscard]] inline bool validTimingSnapshotTuple(const TimingWitness& timing,
                                                   const Reference& reference,
                                                   const LioState& lio,
                                                   const RawPx4State& raw_px4) noexcept {
  return timing.reference_sample_id == reference.identity.sample_id &&
         timing.lio_localization_epoch == lio.localization_epoch &&
         timing.lio_sequence == lio.sequence &&
         timing.px4_timestamp_us == raw_px4.timestamp_us &&
         timing.px4_timestamp_sample_us == raw_px4.timestamp_sample_us;
}

[[nodiscard]] inline bool referenceAgeMatchesSampleAndEvaluation(
    const TimingWitness& timing, const Reference& reference) noexcept {
  if (timing.expected_reference_use_time_ns <
      reference.identity.reference_sample_time_ns) {
    return false;
  }
  const auto delta_ns = timing.expected_reference_use_time_ns -
                        reference.identity.reference_sample_time_ns;
  const double computed_age_s = static_cast<double>(delta_ns) * 1.0e-9;
  return finite(computed_age_s) &&
         std::abs(computed_age_s - timing.reference_age_s) <= 1.0e-9;
}

[[nodiscard]] inline double conservativeTimingBoundV1(
    const TimingWitness& timing) noexcept {
  // Source/pair/anchor ages describe overlapping portions of the same
  // timeline.  Only transport and FC-consume stages are sequential additions;
  // taking the maximum for the overlapping portion avoids double counting.
  const double overlapping_age = std::max({
      timing.reference_age_s, timing.pair_skew_s, timing.lio_source_age_s,
      timing.lio_receive_age_s, timing.px4_source_age_s,
      timing.px4_receive_age_s, timing.predicted_anchor_age_s});
  return timing.clock_mapping_uncertainty_s + overlapping_age +
         timing.output_transport_age_s + timing.px4_consume_age_s;
}

[[nodiscard]] inline Eigen::Matrix3d basisEnuToNed() noexcept {
  return (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                                1.0, 0.0, 0.0,
                                0.0, 0.0, -1.0)
      .finished();
}

[[nodiscard]] inline Eigen::Matrix3d yawRotation(const double yaw) noexcept {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return (Eigen::Matrix3d() << c, -s, 0.0,
                                s, c, 0.0,
                                0.0, 0.0, 1.0)
      .finished();
}

[[nodiscard]] inline bool validPolicy(const Policy& policy) noexcept {
  if (policy.mode != Mode::kShadow && policy.mode != Mode::kLevelA) return false;
  if (policy.experiment_id.empty() || !finite(policy.velocity_lambda) ||
      std::abs(policy.velocity_lambda) > 1.0e-12) {
    return false;
  }
  if (policy.boundary == SetpointBoundary::kVelocityOnly) {
    if (!validPositive(policy.lio_position_feedback_gain_s_inv) ||
        !policy.maximum_velocity_mps.has_value() ||
        !validPositive(*policy.maximum_velocity_mps)) {
      return false;
    }
  } else if (policy.boundary != SetpointBoundary::kPositionVelocityAcceleration ||
             !finite(policy.lio_position_feedback_gain_s_inv) ||
             policy.lio_position_feedback_gain_s_inv != 0.0 ||
             (policy.maximum_velocity_mps.has_value() &&
              !validPositive(*policy.maximum_velocity_mps))) {
    return false;
  }
  if (policy.maximum_timing_bound_s.has_value() &&
      !validNonNegative(*policy.maximum_timing_bound_s)) {
    return false;
  }
  if (policy.maximum_reference_age_s.has_value() &&
      !validNonNegative(*policy.maximum_reference_age_s)) {
    return false;
  }
  if (policy.mode == Mode::kLevelA &&
      (!policy.maximum_timing_bound_s.has_value() ||
       !policy.maximum_reference_age_s.has_value() ||
       !policy.expected_px4_reset_counters.has_value())) {
    return false;
  }
  return true;
}

}  // namespace detail

[[nodiscard]] inline Result adapt(const Reference& reference,
                                  const LioState& lio,
                                  const RawPx4State& raw_px4,
                                  const TimingWitness& timing,
                                  const Policy& policy) {
  Result result;
  if (policy.mode == Mode::kOff) {
    result.failure = FailureReason::kPolicyDisabled;
    return result;
  }
  if (!detail::validPolicy(policy)) {
    result.failure = FailureReason::kInvalidPolicy;
    return result;
  }
  if (reference.frame != ReferenceFrame::kLioEnu ||
      !detail::validIdentity(reference.identity) ||
      !detail::finite(reference.position_enu) ||
      !detail::finite(reference.velocity_enu) ||
      !detail::finite(reference.acceleration_enu) ||
      !detail::finite(reference.yaw_enu) ||
      !detail::finite(reference.yaw_rate_enu_rad_s)) {
    result.failure = reference.frame != ReferenceFrame::kLioEnu
                         ? FailureReason::kUnsupportedFrame
                         : FailureReason::kInvalidReference;
    return result;
  }
  if (!detail::finite(lio.position_enu) || !detail::finite(lio.velocity_enu) ||
      !detail::finite(lio.yaw_enu) || !lio.position_valid || !lio.velocity_valid ||
      !lio.orientation_valid || !lio.navigation_valid || !lio.covariance_valid ||
      !lio.observability_valid || !lio.correction_fresh || !lio.propagation_valid ||
      !lio.localization_epoch || !lio.sequence || lio.source_stamp_ns <= 0 ||
      lio.receive_steady_ns <= 0 ||
      lio.localization_epoch != reference.identity.localization_epoch ||
      (policy.expected_lio_localization_epoch.has_value() &&
       lio.localization_epoch != *policy.expected_lio_localization_epoch)) {
    result.failure = FailureReason::kInvalidLioState;
    return result;
  }
  const bool position_boundary =
      policy.boundary == SetpointBoundary::kPositionVelocityAcceleration;
  if ((position_boundary &&
       (!detail::finite(raw_px4.position_ned) || !raw_px4.position_valid[0] ||
        !raw_px4.position_valid[1] || !raw_px4.position_valid[2])) ||
      !detail::finite(raw_px4.velocity_ned) || !detail::finite(raw_px4.yaw_ned) ||
      !raw_px4.velocity_valid[0] ||
      !raw_px4.velocity_valid[1] || !raw_px4.velocity_valid[2] ||
      !raw_px4.heading_valid || !raw_px4.heading_good_for_control ||
      !detail::finite(raw_px4.heading_variance_rad2) ||
      raw_px4.heading_variance_rad2 < 0.0 || raw_px4.timestamp_us == 0U ||
      raw_px4.timestamp_sample_us == 0U || raw_px4.receive_steady_ns <= 0) {
    result.failure = FailureReason::kInvalidPx4State;
    return result;
  }
  if (policy.reject_dead_reckoning && raw_px4.dead_reckoning) {
    result.failure = FailureReason::kDeadReckoning;
    return result;
  }
  if (!lio.relative_heading_valid || !lio.tilt_valid || !lio.extrinsic_valid) {
    result.failure = FailureReason::kRelativeHeadingInvalid;
    return result;
  }
  if (!detail::validTiming(timing)) {
    result.failure = FailureReason::kInvalidTiming;
    return result;
  }
  if (!detail::validTimingSnapshotTuple(timing, reference, lio, raw_px4) ||
      !detail::referenceAgeMatchesSampleAndEvaluation(timing, reference) ||
      timing.total_bound_s + 1.0e-12 < detail::conservativeTimingBoundV1(timing)) {
    result.failure = FailureReason::kInvalidTiming;
    return result;
  }
  if (policy.maximum_reference_age_s.has_value() &&
      timing.reference_age_s > *policy.maximum_reference_age_s) {
    result.failure = FailureReason::kInvalidTiming;
    return result;
  }
  if (policy.maximum_timing_bound_s.has_value() &&
      timing.total_bound_s > *policy.maximum_timing_bound_s) {
    result.failure = FailureReason::kInvalidTiming;
    return result;
  }
  if (policy.expected_px4_reset_counters.has_value() &&
      raw_px4.reset_counters != *policy.expected_px4_reset_counters) {
    result.failure = FailureReason::kResetMismatch;
    return result;
  }
  if (timing.expected_reference_use_time_ns > reference.identity.lease_valid_until_ns) {
    result.failure = FailureReason::kInvalidReference;
    return result;
  }
  if (raw_px4.timestamp_sample_us > raw_px4.timestamp_us) {
    result.failure = FailureReason::kInvalidPx4State;
    return result;
  }

  const double px4_yaw_enu = detail::wrapPi(
      std::numbers::pi_v<double> / 2.0 - raw_px4.yaw_ned);
  const double delta = detail::wrapPi(px4_yaw_enu - lio.yaw_enu);
  const Eigen::Matrix3d rotation = detail::basisEnuToNed() * detail::yawRotation(delta);
  const Eigen::Vector3d error_lio = reference.position_enu - lio.position_enu;
  Eigen::Vector3d velocity_command_lio = reference.velocity_enu;
  bool velocity_limited = false;
  if (policy.boundary == SetpointBoundary::kVelocityOnly) {
    velocity_command_lio += policy.lio_position_feedback_gain_s_inv * error_lio;
    const double speed = velocity_command_lio.norm();
    if (!detail::finite(speed)) {
      result.failure = FailureReason::kNonFiniteOutput;
      return result;
    }
    if (speed > *policy.maximum_velocity_mps) {
      velocity_command_lio *= *policy.maximum_velocity_mps / speed;
      velocity_limited = true;
    }
  }

  AdaptedReference adapted;
  if (policy.boundary == SetpointBoundary::kVelocityOnly) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    adapted.position_ned = Eigen::Vector3d::Constant(nan);
    adapted.acceleration_ned = Eigen::Vector3d::Constant(nan);
  } else {
    adapted.position_ned = raw_px4.position_ned + rotation * error_lio;
    adapted.acceleration_ned = rotation * reference.acceleration_enu;
  }
  adapted.velocity_ned = rotation * velocity_command_lio;
  adapted.yaw_ned = detail::wrapPi(
      raw_px4.yaw_ned - detail::wrapPi(reference.yaw_enu - lio.yaw_enu));
  adapted.yaw_rate_ned_rad_s = -reference.yaw_rate_enu_rad_s;
  if ((policy.boundary != SetpointBoundary::kVelocityOnly &&
       (!detail::finite(adapted.position_ned) || !detail::finite(adapted.acceleration_ned))) ||
      !detail::finite(adapted.velocity_ned) || !detail::finite(adapted.yaw_ned) ||
      !detail::finite(adapted.yaw_rate_ned_rad_s)) {
    result.failure = FailureReason::kNonFiniteOutput;
    return result;
  }

  adapted.witness.identity = reference.identity;
  adapted.witness.experiment_id = policy.experiment_id;
  adapted.witness.rotation_lio_enu_to_px4_ned = rotation;
  adapted.witness.error_lio_enu = error_lio;
  if (policy.boundary == SetpointBoundary::kVelocityOnly) {
    adapted.witness.error_adapter_ned =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  } else {
    adapted.witness.error_adapter_ned = adapted.position_ned - raw_px4.position_ned;
  }
  adapted.witness.velocity_command_lio_enu = velocity_command_lio;
  adapted.witness.velocity_limited = velocity_limited;
  adapted.witness.px4_reset_counters = raw_px4.reset_counters;
  adapted.witness.timing = timing;
  adapted.witness.mode = policy.mode;
  adapted.witness.boundary = policy.boundary;
  result.output = std::move(adapted);
  return result;
}

}  // namespace tracking_adapter
}  // namespace px4_navigation_external_mode
