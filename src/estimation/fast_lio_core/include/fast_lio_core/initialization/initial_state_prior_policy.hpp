#pragma once

#include <cmath>
#include <cstdint>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/initialization/initial_state_prior.hpp"

namespace uav::nav::lio {

struct InitialStatePriorPolicy {
  InitialStatePriorSource source{InitialStatePriorSource::kZero};
  InitialStatePriorContext context{InitialStatePriorContext::kGroundStartup};
  InitialStatePriorMask mask{true, true, PriorAttitudeMode::kYawOnly};
  std::int64_t topic_wait_timeout_ns{2'000'000'000};
  std::int64_t maximum_topic_prior_age_ns{500'000'000};
  InitialPriorFallback ground_fallback{InitialPriorFallback::kZero};
  double maximum_full_attitude_tilt_disagreement_rad{0.17453292519943295};
  InitialStatePrior fixed_prior{};

  [[nodiscard]] Status validate() const {
    switch (source) {
      case InitialStatePriorSource::kZero:
      case InitialStatePriorSource::kFixed:
      case InitialStatePriorSource::kTopic:
        break;
      default:
        return Status(StatusCode::kInvalidArgument,
                      "invalid initial-state prior source");
    }
    switch (context) {
      case InitialStatePriorContext::kGroundStartup:
      case InitialStatePriorContext::kInFlightReinitialization:
        break;
      default:
        return Status(StatusCode::kInvalidArgument,
                      "invalid initial-state prior context");
    }
    switch (mask.attitude) {
      case PriorAttitudeMode::kNone:
      case PriorAttitudeMode::kYawOnly:
      case PriorAttitudeMode::kFull:
        break;
      default:
        return Status(StatusCode::kInvalidArgument,
                      "invalid initial-state prior attitude mode");
    }
    switch (ground_fallback) {
      case InitialPriorFallback::kReject:
      case InitialPriorFallback::kZero:
      case InitialPriorFallback::kFixed:
        break;
      default:
        return Status(StatusCode::kInvalidArgument,
                      "invalid initial-state prior fallback");
    }
    if (topic_wait_timeout_ns < 0 || maximum_topic_prior_age_ns < 0 ||
        !std::isfinite(maximum_full_attitude_tilt_disagreement_rad) ||
        maximum_full_attitude_tilt_disagreement_rad < 0.0 ||
        maximum_full_attitude_tilt_disagreement_rad > M_PI) {
      return Status(StatusCode::kInvalidArgument, "invalid initial-state prior timing/tilt policy");
    }
    if (source == InitialStatePriorSource::kTopic &&
        context == InitialStatePriorContext::kInFlightReinitialization &&
        ground_fallback != InitialPriorFallback::kReject) {
      return Status(StatusCode::kInvalidArgument,
                    "in-flight initial-state prior must reject on topic timeout");
    }
    if (source == InitialStatePriorSource::kFixed &&
        (fixed_prior.source != InitialStatePriorSource::kFixed ||
         fixed_prior.context != context || fixed_prior.reference_frame.empty() ||
         fixed_prior.body_frame.empty() || !fixed_prior.allFinite())) {
      return Status(StatusCode::kInvalidArgument, "fixed initial-state prior has wrong source");
    }
    if (ground_fallback == InitialPriorFallback::kFixed &&
        (fixed_prior.source != InitialStatePriorSource::kFixed ||
         fixed_prior.reference_frame.empty() || fixed_prior.body_frame.empty() ||
         !fixed_prior.allFinite())) {
      return Status(StatusCode::kInvalidArgument,
                    "fixed fallback requires a valid fixed initial-state prior");
    }
    return Status::Ok();
  }
};

}  // namespace uav::nav::lio
