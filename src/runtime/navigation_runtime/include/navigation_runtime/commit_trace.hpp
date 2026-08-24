#pragma once

#include <cstdint>

namespace navigation_runtime {

inline bool commitObservedThisCycle(std::uint64_t generation_before,
                                    std::uint64_t generation_after,
                                    std::uint64_t diagnostics_generation) noexcept {
  return generation_after > generation_before &&
         diagnostics_generation == generation_after;
}

inline double executionStateAgeMs(std::int64_t solve_start_ros_ns,
                                  std::int64_t execution_stamp_ns) noexcept {
  return static_cast<double>(solve_start_ros_ns - execution_stamp_ns) * 1.0e-6;
}

}  // namespace navigation_runtime
