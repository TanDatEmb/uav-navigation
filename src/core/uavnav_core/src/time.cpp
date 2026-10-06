#include "uavnav/core/time.hpp"

#include <chrono>

namespace uavnav::time {

SteadyTime steady_now() noexcept {
  const auto since_epoch = std::chrono::steady_clock::now().time_since_epoch();
  return SteadyTime{std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch).count()};
}

}  // namespace uavnav::time
