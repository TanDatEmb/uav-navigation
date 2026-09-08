#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <Eigen/Core>

namespace navigation_contracts {

// Explicit SITL experiment. These bounds are not part of the planner's
// certified clearance envelope and do not constitute flight qualification.
struct TrackingExperimentPolicy {
  bool enabled{false};
  bool suppress_braking{false};
  double base_m{0.20};
  double lateral_alpha_s{0.05};
  double longitudinal_beta_s{0.15};

  bool valid() const noexcept {
    return std::isfinite(base_m) && base_m > 0.0 &&
        std::isfinite(lateral_alpha_s) && lateral_alpha_s >= 0.0 &&
        std::isfinite(longitudinal_beta_s) && longitudinal_beta_s >= 0.0 &&
        (!suppress_braking || enabled);
  }
};

template<class Node>
TrackingExperimentPolicy loadTrackingExperimentPolicy(Node& node) {
  TrackingExperimentPolicy p;
  p.enabled = node.template declare_parameter<bool>("tracking_experiment.enabled", false);
  p.suppress_braking = node.template declare_parameter<bool>(
      "tracking_experiment.suppress_braking", false);
  p.base_m = node.template declare_parameter<double>("tracking_experiment.base_m", 0.20);
  p.lateral_alpha_s = node.template declare_parameter<double>(
      "tracking_experiment.lateral_alpha_s", 0.05);
  p.longitudinal_beta_s = node.template declare_parameter<double>(
      "tracking_experiment.longitudinal_beta_s", 0.15);
  if (!p.valid() || (p.enabled && !node.get_parameter("use_sim_time").as_bool())) {
    throw std::invalid_argument("tracking experiment requires valid coefficients and use_sim_time=true");
  }
  return p;
}

struct AdaptiveTrackingAssessment {
  bool valid{false};
  bool within_limits{false};
  double speed_mps{0.0};
  double lateral_error_m{0.0};
  double longitudinal_error_m{0.0};
  double lateral_limit_m{0.0};
  double longitudinal_limit_m{0.0};
};

inline double trackingVectorNorm(const Eigen::Vector3d& v) {
  return std::hypot(std::hypot(v.x(), v.y()), v.z());
}

// The normal plane is 3-D and therefore includes vertical deviation. At rest
// there is no defined tangent: use the finite base radius in every direction.
// A nonzero commanded speed remains relevant while a measured vehicle slows.
inline AdaptiveTrackingAssessment assessAdaptiveTracking(
    const TrackingExperimentPolicy& policy,
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& measured_velocity,
    const Eigen::Vector3d& reference_position,
    const Eigen::Vector3d& reference_velocity) {
  AdaptiveTrackingAssessment r;
  if (!policy.valid() || !measured_position.allFinite() ||
      !measured_velocity.allFinite() || !reference_position.allFinite() ||
      !reference_velocity.allFinite()) return r;
  const auto error = (measured_position - reference_position).eval();
  const double measured_speed = trackingVectorNorm(measured_velocity);
  const double reference_speed = trackingVectorNorm(reference_velocity);
  r.speed_mps = std::max(measured_speed, reference_speed);
  r.lateral_limit_m = policy.base_m + policy.lateral_alpha_s * r.speed_mps;
  r.longitudinal_limit_m = policy.base_m + policy.longitudinal_beta_s * r.speed_mps;
  if (!error.allFinite() || !std::isfinite(r.speed_mps) ||
      !std::isfinite(r.lateral_limit_m) || !std::isfinite(r.longitudinal_limit_m)) return r;
  if (r.speed_mps <= 1.0e-6) {
    r.lateral_error_m = trackingVectorNorm(error);
    r.longitudinal_error_m = r.lateral_error_m;
  } else {
    const Eigen::Vector3d tangent = reference_speed > 1.0e-6
        ? (reference_velocity / reference_speed).eval()
        : (measured_velocity / measured_speed).eval();
    const double along = error.dot(tangent);
    r.longitudinal_error_m = std::abs(along);
    r.lateral_error_m = trackingVectorNorm(error - along * tangent);
  }
  r.valid = std::isfinite(r.lateral_error_m) && std::isfinite(r.longitudinal_error_m);
  r.within_limits = r.valid && r.lateral_error_m <= r.lateral_limit_m &&
      r.longitudinal_error_m <= r.longitudinal_limit_m;
  return r;
}

inline bool experimentPermitsTracking(
    const TrackingExperimentPolicy& p, const AdaptiveTrackingAssessment& r) {
  return p.enabled && r.valid && (r.within_limits || p.suppress_braking);
}

}  // namespace navigation_contracts
