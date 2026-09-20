#include "fast_lio_core/time/timestamp_validator.hpp"

#include <string>

namespace uav::nav::lio {

TimestampValidator::TimestampValidator(TimestampValidatorConfig config) : config_(config) {}

Status TimestampValidator::validate(const Timestamp& timestamp) {
  if (config_.expected_clock_domain && timestamp.clock_domain() != *config_.expected_clock_domain) {
    ++clock_mismatch_count_;
    return Status(StatusCode::kClockDomainMismatch,
                  "Expected clock domain " + std::string(toString(*config_.expected_clock_domain)) +
                      ", received " + std::string(toString(timestamp.clock_domain())));
  }

  if (last_timestamp_ && !timestamp.sameClockDomain(*last_timestamp_)) {
    ++clock_mismatch_count_;
    return Status(StatusCode::kClockDomainMismatch,
                  "Clock domain changed within one measurement stream");
  }

  if (last_timestamp_) {
    const bool regressed = timestamp.nanoseconds() < last_timestamp_->nanoseconds();
    const bool repeated = timestamp.nanoseconds() == last_timestamp_->nanoseconds();
    if (regressed || (repeated && !config_.allow_equal_timestamps)) {
      ++regression_count_;
      if (config_.reject_regression || repeated) {
        return Status(StatusCode::kTimestampRegression,
                      regressed ? "Timestamp regressed" : "Repeated timestamp is not allowed");
      }
      // Keep the latest frontier when the caller explicitly permits
      // out-of-order arrival. The caller remains responsible for ordered
      // storage.
      return Status::Ok();
    }
  }

  last_timestamp_ = timestamp;
  return Status::Ok();
}

void TimestampValidator::reset() noexcept {
  last_timestamp_.reset();
  regression_count_ = 0;
  clock_mismatch_count_ = 0;
}

const std::optional<Timestamp>& TimestampValidator::lastTimestamp() const noexcept {
  return last_timestamp_;
}

std::size_t TimestampValidator::regressionCount() const noexcept { return regression_count_; }

std::size_t TimestampValidator::clockMismatchCount() const noexcept {
  return clock_mismatch_count_;
}

}  // namespace uav::nav::lio
