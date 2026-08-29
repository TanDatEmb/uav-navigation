#include "fast_lio_core/navigation/angular_velocity_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uav::nav::lio {
namespace {

void recordFailure(const Status& status, AngularVelocityDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return;
  }
  switch (status.code()) {
    case StatusCode::kMissingStartBracket:
    case StatusCode::kMissingEndBracket:
      ++diagnostics->missing_bracket_count;
      break;
    case StatusCode::kClockDomainMismatch:
    case StatusCode::kTimestampRegression:
      ++diagnostics->timestamp_mismatch_count;
      break;
    case StatusCode::kInvalidArgument:
    case StatusCode::kNumericalFailure:
      ++diagnostics->nonfinite_reject_count;
      break;
    default:
      break;
  }
}

Result<KinematicStateEstimate> failure(StatusCode code, const char* message,
                                       AngularVelocityDiagnostics* diagnostics) {
  const Status status(code, message);
  recordFailure(status, diagnostics);
  return status;
}

Result<KinematicStateEstimate> finish(const StateEstimate& estimate,
                                      const Eigen::Vector3d& raw_omega,
                                      const bool interpolated,
                                      AngularVelocityDiagnostics* diagnostics) {
  if (!estimate.allFinite() || !raw_omega.allFinite() ||
      !estimate.state.gyro_bias_rad_s().allFinite()) {
    return failure(StatusCode::kNumericalFailure,
                   "State, raw angular velocity, or gyro bias is non-finite",
                   diagnostics);
  }
  const Eigen::Vector3d corrected_omega =
      raw_omega - estimate.state.gyro_bias_rad_s();
  if (!corrected_omega.allFinite()) {
    return failure(StatusCode::kNumericalFailure,
                   "Bias-corrected angular velocity is non-finite",
                   diagnostics);
  }
  if (diagnostics != nullptr) {
    diagnostics->angular_velocity_available = true;
    if (interpolated) {
      ++diagnostics->interpolated_count;
    } else {
      ++diagnostics->exact_sample_count;
    }
  }
  return KinematicStateEstimate{estimate, corrected_omega};
}

}  // namespace

Result<KinematicStateEstimate> AngularVelocityResolver::resolveExact(
    const StateEstimate& estimate, const ImuSample& sample,
    AngularVelocityDiagnostics* diagnostics) {
  if (!estimate.time.sameClockDomain(sample.time)) {
    return failure(StatusCode::kClockDomainMismatch,
                   "State and exact IMU sample use different clock domains",
                   diagnostics);
  }
  if (estimate.time != sample.time) {
    return failure(StatusCode::kTimestampRegression,
                   "Exact IMU sample does not match state epoch", diagnostics);
  }
  const Status sample_status = sample.validate();
  if (!sample_status.ok()) {
    recordFailure(sample_status, diagnostics);
    return sample_status;
  }
  return finish(estimate, sample.angular_velocity_imu_rad_s, false,
                diagnostics);
}

Result<KinematicStateEstimate> AngularVelocityResolver::resolve(
    const StateEstimate& estimate, const std::span<const ImuSample> samples,
    AngularVelocityDiagnostics* diagnostics) {
  if (!estimate.allFinite()) {
    return failure(StatusCode::kNumericalFailure,
                   "State estimate is non-finite", diagnostics);
  }
  if (samples.empty()) {
    return failure(StatusCode::kMissingStartBracket,
                   "No IMU samples are available at the state epoch",
                   diagnostics);
  }

  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const Status sample_status = samples[i].validate();
    if (!sample_status.ok()) {
      recordFailure(sample_status, diagnostics);
      return sample_status;
    }
    if (!samples[i].time.sameClockDomain(estimate.time)) {
      return failure(StatusCode::kClockDomainMismatch,
                     "IMU and state use different clock domains", diagnostics);
    }
    if (i > 0U && samples[i - 1U].time.nanoseconds() >=
                     samples[i].time.nanoseconds()) {
      return failure(StatusCode::kTimestampRegression,
                     "IMU samples are not strictly increasing", diagnostics);
    }
  }

  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), estimate.time.nanoseconds(),
      [](const ImuSample& sample, const std::int64_t time_ns) {
        return sample.time.nanoseconds() < time_ns;
      });
  if (upper != samples.end() && upper->time == estimate.time) {
    return finish(estimate, upper->angular_velocity_imu_rad_s, false,
                  diagnostics);
  }
  if (upper == samples.begin()) {
    return failure(StatusCode::kMissingStartBracket,
                   "No IMU sample at or before state epoch", diagnostics);
  }
  if (upper == samples.end()) {
    return failure(StatusCode::kMissingEndBracket,
                   "No IMU sample at or after state epoch", diagnostics);
  }

  const ImuSample& before = *(upper - 1U);
  const ImuSample& after = *upper;
  if (!before.time.sameClockDomain(after.time) ||
      before.time.nanoseconds() >= after.time.nanoseconds() ||
      estimate.time.nanoseconds() < before.time.nanoseconds() ||
      estimate.time.nanoseconds() > after.time.nanoseconds()) {
    return failure(StatusCode::kTimestampRegression,
                   "IMU bracket is not valid for interpolation", diagnostics);
  }
  // Compute integer deltas before converting to floating point.  This keeps
  // nanosecond epochs out of double precision and only converts the bounded
  // bracket ratio.
  const auto numerator_result = checkedDifference(estimate.time, before.time);
  const auto denominator_result = checkedDifference(after.time, before.time);
  if (!numerator_result.ok()) {
    return failure(numerator_result.status().code(),
                   "IMU interpolation numerator overflows", diagnostics);
  }
  if (!denominator_result.ok()) {
    return failure(denominator_result.status().code(),
                   "IMU interpolation denominator overflows", diagnostics);
  }
  const long double numerator =
      static_cast<long double>(numerator_result.value().nanoseconds());
  const long double denominator =
      static_cast<long double>(denominator_result.value().nanoseconds());
  const long double alpha = numerator / denominator;
  if (!std::isfinite(alpha) || alpha < 0.0L || alpha > 1.0L) {
    return failure(StatusCode::kNumericalFailure,
                   "IMU interpolation ratio is non-finite", diagnostics);
  }
  const Eigen::Vector3d raw_omega =
      before.angular_velocity_imu_rad_s +
      static_cast<double>(alpha) *
          (after.angular_velocity_imu_rad_s -
           before.angular_velocity_imu_rad_s);
  return finish(estimate, raw_omega, true, diagnostics);
}

}  // namespace uav::nav::lio
