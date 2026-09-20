#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/planning_timing.hpp>

namespace navigation_planning {

[[nodiscard]] inline double remainingMainHorizonAtCommit(
    const CandidateBundle& candidate, double commit_wall_time_s) noexcept {
  if (!candidate.hasTrajectoryMetadata() ||
      !std::isfinite(commit_wall_time_s)) {
    return -std::numeric_limits<double>::infinity();
  }
  const double elapsed_s = std::max(
      0.0, commit_wall_time_s - candidate.start_wall_time_s);
  return candidate.backup_start_time_s - elapsed_s;
}

[[nodiscard]] inline bool certifiedTerminalStopAtEndpoint(
    const CandidateBundle& candidate) {
  if ((!candidate.terminal_stop &&
       candidate.kind != CandidateBundleKind::kTerminalStop) ||
      !candidate.certificates.completeFor(candidate.kind) ||
      !candidate.certificates.terminal_stop) {
    return false;
  }
  const auto endpoint = candidate.sampleAtDeclaredEnd();
  if (!endpoint || !endpoint->finished) return false;
  constexpr double kRoundoff = 1.0e-9;
  return endpoint->velocity_world.norm() <= kRoundoff &&
         endpoint->acceleration_world.norm() <= kRoundoff &&
         endpoint->jerk_world.norm() <= kRoundoff;
}

[[nodiscard]] inline bool candidateHasRequiredMainReserve(
    const CandidateBundle& candidate, double commit_wall_time_s,
    double minimum_reserve_s =
        PlanningTimingContract::kMinimumMainReserveS) {
  if (!candidate.valid() || !std::isfinite(minimum_reserve_s) ||
      minimum_reserve_s <= 0.0) {
    return false;
  }
  if (candidate.terminal_stop || candidate.kind == CandidateBundleKind::kTerminalStop) {
    return certifiedTerminalStopAtEndpoint(candidate);
  }
  if (candidate.kind == CandidateBundleKind::kBackupOnly ||
      candidate.kind == CandidateBundleKind::kEmergencyBrake) {
    return true;
  }
  return remainingMainHorizonAtCommit(candidate, commit_wall_time_s) + 1.0e-9 >=
         minimum_reserve_s;
}

}  // namespace navigation_planning
