#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace navigation_planning_backend {

struct CommandTrajectoryTime {
  bool finished{false};
  double trajectory_time_s{0.0};
};

// Preserve planner backend's established wall-time to trajectory-time contract: samples
// before start retain a negative TT, while samples after completion pin to the
// terminal state. Changing the lower-bound policy is a behavior change and must
// be certified separately from command provenance.
inline CommandTrajectoryTime commandTrajectoryTime(double wall_time_s,
                                                    double start_wall_time_s,
                                                    double duration_s) noexcept {
  const double elapsed_s = wall_time_s - start_wall_time_s;
  const bool finished = elapsed_s > duration_s;
  return {finished, finished ? duration_s : elapsed_s};
}

// Exact execution-boundary variant.  Absolute epoch seconds are not used for
// this contract: converting a nanosecond stamp to a large double and
// subtracting another large double loses tens to hundreds of nanoseconds.
// Keep the delta integral until the final conversion to the small trajectory
// clock, and clamp at the rounded declared endpoint.
inline CommandTrajectoryTime commandTrajectoryTime(
    const std::int64_t stamp_ns, const std::int64_t start_ns,
    const std::int64_t end_ns, const double duration_s) noexcept {
  if (start_ns <= 0 || end_ns < start_ns || !std::isfinite(duration_s) ||
      duration_s < 0.0) {
    return {};
  }
  const __int128 delta_ns = static_cast<__int128>(stamp_ns) -
                            static_cast<__int128>(start_ns);
  if (stamp_ns >= end_ns) {
    return {stamp_ns > end_ns, duration_s};
  }
  const double elapsed_s = static_cast<double>(delta_ns) * 1.0e-9;
  return {false, elapsed_s};
}

struct HotReplanWindow {
  bool valid{false};
  bool reaches_command_end{false};
  double start_tt_s{0.0};
  double state_tt_s{0.0};
};

// Convert one hot-replan wall-clock instant into the clock of the executable
// command. Planner history may have a different start time and must not be used
// for this conversion.
inline HotReplanWindow hotReplanWindow(double replan_start_wall_time_s,
                                       double command_start_wall_time_s,
                                       double forward_time_s,
                                       double command_duration_s) noexcept {
  if (!std::isfinite(replan_start_wall_time_s) ||
      !std::isfinite(command_start_wall_time_s) ||
      !std::isfinite(forward_time_s) || forward_time_s < 0.0 ||
      !std::isfinite(command_duration_s) || command_duration_s <= 0.0) {
    return {};
  }
  const double start_tt_s =
      replan_start_wall_time_s - command_start_wall_time_s;
  const double state_tt_s = start_tt_s + forward_time_s;
  if (!std::isfinite(start_tt_s) || start_tt_s < 0.0 ||
      !std::isfinite(state_tt_s) || state_tt_s < start_tt_s) {
    return {};
  }
  return {true, state_tt_s >= command_duration_s, start_tt_s, state_tt_s};
}

struct InheritedBackupInterval {
  bool valid{false};
  bool present{false};
  double begin_tt_s{-1.0};
  double end_tt_s{-1.0};
};

// Express the part of an existing BACKUP interval retained by the hot-replan
// prefix in the candidate's local clock. If BACKUP began before the candidate,
// its local begin is zero rather than a negative sentinel-like value.
inline InheritedBackupInterval inheritedBackupInterval(
    double backup_start_tt_s, const HotReplanWindow& window,
    double candidate_duration_s) noexcept {
  if (!window.valid || !std::isfinite(backup_start_tt_s) ||
      !std::isfinite(candidate_duration_s) || candidate_duration_s < 0.0) {
    return {};
  }
  if (backup_start_tt_s < 0.0 || window.state_tt_s <= backup_start_tt_s) {
    return {true, false, -1.0, -1.0};
  }
  const double prefix_duration_s = window.state_tt_s - window.start_tt_s;
  const double begin_tt_s =
      std::max(0.0, backup_start_tt_s - window.start_tt_s);
  const double end_tt_s = prefix_duration_s;
  if (!std::isfinite(prefix_duration_s) || !std::isfinite(begin_tt_s) ||
      !std::isfinite(end_tt_s) || begin_tt_s > end_tt_s ||
      end_tt_s > candidate_duration_s) {
    return {};
  }
  return {true, true, begin_tt_s, end_tt_s};
}

}  // namespace navigation_planning_backend
