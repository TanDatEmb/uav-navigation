#pragma once

#include <cmath>
#include <cstdint>
#include <limits>


namespace navigation_runtime {

inline bool commitObservedThisCycle(std::uint64_t generation_before,
                                    std::uint64_t generation_after,
                                    std::uint64_t diagnostics_generation) noexcept {
  return generation_after > generation_before &&
         diagnostics_generation == generation_after;
}

inline double executionStateAgeMs(std::int64_t solve_start_ros_ns,
                                  std::int64_t execution_stamp_ns) noexcept {
  const __int128 delta = static_cast<__int128>(solve_start_ros_ns) -
                         static_cast<__int128>(execution_stamp_ns);
  if (delta < static_cast<__int128>(std::numeric_limits<std::int64_t>::min()) ||
      delta > static_cast<__int128>(std::numeric_limits<std::int64_t>::max())) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(static_cast<std::int64_t>(delta)) * 1.0e-6;
}

}  // namespace navigation_runtime
