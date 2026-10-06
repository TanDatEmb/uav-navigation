#include "uavnav/core/time_convert.hpp"

#include <cmath>

namespace uavnav::time {

Result<Duration, TimeError> duration_from_seconds(double s) noexcept {
  if (!std::isfinite(s)) return std::unexpected(TimeError::kNotFinite);
  if (std::fabs(s) > kMaxDurationSeconds) return std::unexpected(TimeError::kOutOfRange);
  // |s| <= 1e6, so |s * 1e9| <= 1e15, far inside int64.
  return Duration{static_cast<std::int64_t>(std::llround(s * 1e9))};
}

}  // namespace uavnav::time
