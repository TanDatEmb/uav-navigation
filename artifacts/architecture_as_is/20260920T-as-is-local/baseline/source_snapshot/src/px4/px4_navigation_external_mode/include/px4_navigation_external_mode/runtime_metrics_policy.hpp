#pragma once

#include <cstdint>

#include <navigation_common/time.hpp>

namespace px4_navigation_external_mode {

inline bool runtimeMetricsLogDue(const std::int64_t last_log_ns,
                                 const std::int64_t now_ns) noexcept {
  if (now_ns <= 0) return false;
  if (last_log_ns <= 0) return true;
  const auto elapsed = navigation_common::checkedDifference(now_ns, last_log_ns);
  return elapsed.has_value() && *elapsed >= 1'000'000'000LL;
}

}  // namespace px4_navigation_external_mode
