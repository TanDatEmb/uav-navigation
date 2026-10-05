#pragma once

#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace navigation_runtime {

// The ingress owner serializes epoch transitions and has already published
// the new epoch/unready state and revoked the old timeline. Mapping needs this
// lifecycle mutex to finish publication; never wait for its drain while
// holding that mutex. No other lifecycle lock may be held across the drain.
template <typename Drain>
void drainMappingForLocalizationReset(
    std::unique_lock<std::mutex>& localization_lock, Drain&& drain) {
  if (!localization_lock.owns_lock()) {
    throw std::invalid_argument("localization reset must own its lifecycle lock");
  }
  localization_lock.unlock();
  try {
    std::invoke(std::forward<Drain>(drain));
  } catch (...) {
    localization_lock.lock();
    throw;
  }
  localization_lock.lock();
}

}  // namespace navigation_runtime
