#pragma once

#include <cmath>
#include <cstdint>
#include <navigation_planning/candidate_bundle.hpp>

namespace navigation_execution {

// Immutable future state sampled from the command that execution owns.
struct ExecutionAnchor final {
  std::uint64_t active_bundle_generation{0};
  std::uint64_t localization_epoch{0};
  std::uint64_t goal_epoch{0};
  std::uint64_t request_id{0};
  std::int64_t request_stamp_ns{0};
  std::int64_t activation_stamp_ns{0};
  navigation_planning::TrajectoryPoint state{};
  navigation_planning::CandidateRole active_role{
      navigation_planning::CandidateRole::kMain};
  std::int64_t active_main_end_ns{0};
  std::int64_t active_bundle_end_ns{0};
  navigation_world_model::WorldSnapshotIdentity command_world{};

  [[nodiscard]] bool valid() const noexcept {
    return active_bundle_generation != 0U && localization_epoch != 0U &&
           goal_epoch != 0U && request_id != 0U && request_stamp_ns > 0 &&
           activation_stamp_ns >= request_stamp_ns && state.finite() &&
           navigation_planning::candidateRoleValid(active_role) &&
           active_main_end_ns >= request_stamp_ns &&
           active_bundle_end_ns >= active_main_end_ns &&
           command_world.localization_epoch == localization_epoch &&
           command_world.generation != 0U && command_world.revision != 0U &&
           command_world.observation_stamp_ns > 0;
  }
};

enum class AnchorMatchResult : std::uint8_t {
  kMatch,
  kInvalidAnchor,
  kInvalidCandidate,
  kNoSample,
  kPositionMismatch,
  kVelocityMismatch,
  kAccelerationMismatch,
  kJerkMismatch,
  kYawMismatch,
  kYawRateMismatch,
};

// Analytic anchor compatibility is separate from lease/world admission. This
// keeps a genuine PVAJ discontinuity distinguishable from an expired command.
[[nodiscard]] inline AnchorMatchResult candidateMatchesAnchor(
    const navigation_planning::CandidateBundle& candidate,
    const ExecutionAnchor& anchor) noexcept {
  if (!anchor.valid()) return AnchorMatchResult::kInvalidAnchor;
  if (!candidate.valid() || candidate.kind ==
          navigation_planning::CandidateBundleKind::kEmergencyBrake) {
    return AnchorMatchResult::kInvalidCandidate;
  }
  if (candidate.valid_from_ns != anchor.activation_stamp_ns) {
    return AnchorMatchResult::kNoSample;
  }
  const auto sample = candidate.sample(anchor.activation_stamp_ns);
  if (!sample) return AnchorMatchResult::kNoSample;
  constexpr double kPositionToleranceM = 1.0e-5;
  constexpr double kVelocityToleranceMps = 1.0e-5;
  constexpr double kAccelerationToleranceMps2 = 1.0e-4;
  constexpr double kJerkToleranceMps3 = 1.0e-3;
  constexpr double kYawToleranceRad = 1.0e-6;
  constexpr double kYawRateToleranceRadS = 1.0e-5;
  if ((sample->position_world - anchor.state.position_world).norm() >
      kPositionToleranceM) return AnchorMatchResult::kPositionMismatch;
  if ((sample->velocity_world - anchor.state.velocity_world).norm() >
      kVelocityToleranceMps) return AnchorMatchResult::kVelocityMismatch;
  if ((sample->acceleration_world - anchor.state.acceleration_world).norm() >
      kAccelerationToleranceMps2) return AnchorMatchResult::kAccelerationMismatch;
  if ((sample->jerk_world - anchor.state.jerk_world).norm() >
      kJerkToleranceMps3) return AnchorMatchResult::kJerkMismatch;
  const double yaw_residual = std::remainder(
      sample->yaw - anchor.state.yaw, 2.0 * std::acos(-1.0));
  if (std::abs(yaw_residual) > kYawToleranceRad) return AnchorMatchResult::kYawMismatch;
  if (std::abs(sample->yaw_rate - anchor.state.yaw_rate) >
      kYawRateToleranceRadS) return AnchorMatchResult::kYawRateMismatch;
  return AnchorMatchResult::kMatch;
}

}  // namespace navigation_execution
