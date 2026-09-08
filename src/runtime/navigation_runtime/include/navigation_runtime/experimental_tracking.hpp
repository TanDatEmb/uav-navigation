#pragma once

#include <limits>
#include <tracking_experiment.hpp>
#include <navigation_planning/candidate_bundle.hpp>

namespace navigation_runtime {

struct ExperimentalTrackingResult {
  navigation_contracts::AdaptiveTrackingAssessment current;
  navigation_contracts::AdaptiveTrackingAssessment predicted;
  bool support_valid{false};
  bool accepted{false};
  bool suppression_used{false};
};

// Must run outside owner locks. Experimental tolerance is expressed in metres,
// not a fixed +/-100 ms phase window. It compares analytic samples at source
// and forecast time; tangent-plane normal error on curved paths is an
// approximation, explicitly limited to this unqualified experiment.
inline ExperimentalTrackingResult assessExperimentalTracking(
    const navigation_contracts::TrackingExperimentPolicy& policy,
    const navigation_planning::CandidateBundle& bundle,
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    std::int64_t now_ns, std::int64_t source_ns, double interval_s,
    bool fresh, bool body_known_free, bool path_clear) {
  using navigation_planning::CandidateRole;
  ExperimentalTrackingResult out;
  if (!policy.enabled || !policy.valid() || !fresh || !body_known_free || !path_clear ||
      !bundle.valid() || bundle.terminal_stop || bundle.role != CandidateRole::kMain ||
      source_ns <= 0 || source_ns > now_ns || !std::isfinite(interval_s) || interval_s <= 0.0)
    return out;
  const long double future_ns = static_cast<long double>(now_ns) + interval_s * 1.0e9L;
  if (future_ns > std::numeric_limits<std::int64_t>::max()) return out;
  const auto source = bundle.sampleAtDeclaredStamp(source_ns);
  const auto current = bundle.sample(now_ns);
  const auto future = bundle.sample(static_cast<std::int64_t>(future_ns));
  if (!source || !current || !future || source->role != CandidateRole::kMain ||
      current->role != CandidateRole::kMain || future->role != CandidateRole::kMain)
    return out;
  const double source_local_s = static_cast<double>(
      (static_cast<long double>(source_ns) - bundle.declared_start_ns) * 1.0e-9L);
  const double future_local_s = static_cast<double>(
      (future_ns - bundle.declared_start_ns) * 1.0e-9L);
  bool same_main_interval = false;
  for (const auto& interval : bundle.role_schedule) {
    if (interval.role == CandidateRole::kMain && source_local_s >= interval.begin_time_s &&
        future_local_s < interval.end_time_s) same_main_interval = true;
  }
  if (!same_main_interval) return out;
  out.current = navigation_contracts::assessAdaptiveTracking(
      policy, position, velocity, source->position_world, source->velocity_world);
  const double horizon_s = static_cast<double>(now_ns - source_ns) * 1.0e-9 + interval_s;
  out.predicted = navigation_contracts::assessAdaptiveTracking(
      policy, position + velocity * horizon_s, velocity,
      future->position_world, future->velocity_world);
  out.support_valid = out.current.valid && out.predicted.valid;
  out.accepted = out.support_valid &&
      navigation_contracts::experimentPermitsTracking(policy, out.current) &&
      navigation_contracts::experimentPermitsTracking(policy, out.predicted);
  out.suppression_used = out.accepted && policy.suppress_braking &&
      (!out.current.within_limits || !out.predicted.within_limits);
  return out;
}

}  // namespace navigation_runtime
