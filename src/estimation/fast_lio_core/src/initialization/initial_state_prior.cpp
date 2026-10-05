#include "fast_lio_core/initialization/initial_state_prior.hpp"

#include <cmath>

namespace uav::nav::lio {

const char* toString(InitialStatePriorSource value) noexcept {
  switch (value) {
    case InitialStatePriorSource::kZero: return "zero";
    case InitialStatePriorSource::kFixed: return "fixed";
    case InitialStatePriorSource::kTopic: return "topic";
  }
  return "unknown";
}

const char* toString(InitialStatePriorContext value) noexcept {
  switch (value) {
    case InitialStatePriorContext::kGroundStartup: return "ground_startup";
    case InitialStatePriorContext::kInFlightReinitialization: return "in_flight_reinitialization";
  }
  return "unknown";
}

const char* toString(PriorAttitudeMode value) noexcept {
  switch (value) {
    case PriorAttitudeMode::kNone: return "none";
    case PriorAttitudeMode::kYawOnly: return "yaw_only";
    case PriorAttitudeMode::kFull: return "full";
  }
  return "unknown";
}

const char* toString(InitialPriorStatus value) noexcept {
  switch (value) {
    case InitialPriorStatus::kNotRequired: return "not_required";
    case InitialPriorStatus::kWaiting: return "waiting";
    case InitialPriorStatus::kCandidateAvailable: return "candidate_available";
    case InitialPriorStatus::kApplied: return "applied";
    case InitialPriorStatus::kFallbackApplied: return "fallback_applied";
    case InitialPriorStatus::kRejected: return "rejected";
    case InitialPriorStatus::kClosed: return "closed";
  }
  return "unknown";
}

bool InitialStatePrior::allFinite() const noexcept {
  const bool linear_finite = !linear_velocity_base_m_s.has_value() ||
                             linear_velocity_base_m_s->allFinite();
  const bool angular_finite = !angular_velocity_base_rad_s.has_value() ||
                              angular_velocity_base_rad_s->allFinite();
  const bool covariance_finite = !covariance.has_value() || covariance->allFinite();
  const double orientation_squared_norm = orientation_odom_base.squaredNorm();
  return position_odom_base_m.allFinite() && orientation_odom_base.coeffs().allFinite() &&
         std::isfinite(orientation_squared_norm) &&
         std::abs(orientation_squared_norm - 1.0) <= 1e-6 && linear_finite &&
         angular_finite && covariance_finite;
}

}  // namespace uav::nav::lio
