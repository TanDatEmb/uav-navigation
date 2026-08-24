#pragma once

namespace super_planner {

struct CommandTrajectoryTime {
  bool finished{false};
  double trajectory_time_s{0.0};
};

// Preserve SUPER's established wall-time to trajectory-time contract: samples
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

}  // namespace super_planner
