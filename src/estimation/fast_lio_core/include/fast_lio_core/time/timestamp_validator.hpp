#pragma once

#include <cstddef>
#include <optional>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/time/clock_domain.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct TimestampValidatorConfig {
  std::optional<ClockDomain> expected_clock_domain;
  bool reject_regression{true};
  bool allow_equal_timestamps{true};
};

class TimestampValidator {
 public:
  explicit TimestampValidator(TimestampValidatorConfig config = {});

  [[nodiscard]] Status validate(const Timestamp& timestamp);
  void reset() noexcept;

  [[nodiscard]] const std::optional<Timestamp>& lastTimestamp() const noexcept;
  [[nodiscard]] std::size_t regressionCount() const noexcept;
  [[nodiscard]] std::size_t clockMismatchCount() const noexcept;

 private:
  TimestampValidatorConfig config_;
  std::optional<Timestamp> last_timestamp_;
  std::size_t regression_count_{0};
  std::size_t clock_mismatch_count_{0};
};

}  // namespace uav::nav::lio
