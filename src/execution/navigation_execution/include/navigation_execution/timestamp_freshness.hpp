#pragma once

#include <cstdint>

namespace navigation_execution {

enum class TimestampFreshness : std::uint8_t { VALID = 0, INVALID, STALE, FUTURE };

inline TimestampFreshness classifyTimestampFreshness(
    std::int64_t now_ns, std::int64_t stamp_ns, std::int64_t maximum_age_ns) noexcept {
  if (stamp_ns <= 0 || maximum_age_ns <= 0) return TimestampFreshness::INVALID;
  const auto age_ns = now_ns - stamp_ns;
  if (age_ns > maximum_age_ns) return TimestampFreshness::STALE;
  if (age_ns < -maximum_age_ns) return TimestampFreshness::FUTURE;
  return TimestampFreshness::VALID;
}

}  // namespace navigation_execution
