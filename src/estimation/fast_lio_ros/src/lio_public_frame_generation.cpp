#include "fast_lio_ros/lio_public_frame_generation.hpp"

#include <limits>
#include <utility>

namespace uav::nav::lio {

void LioPublicFrameGeneration::observe(const PublicFrameEvent event,
                                       const std::uint64_t event_token,
                                       std::string reason) {
  if (event != PublicFrameEvent::kPublicFrameDiscontinuity) {
    return;
  }
  if (event_token == 0) {
    return;
  }
  std::lock_guard lock(mutex_);
  if (event_token == last_event_token_) {
    return;
  }
  if (snapshot_.generation == std::numeric_limits<std::uint64_t>::max() ||
      snapshot_.discontinuity_count == std::numeric_limits<std::uint64_t>::max()) {
    last_event_token_ = event_token;
    snapshot_.valid = false;
    snapshot_.last_event = "PUBLIC_FRAME_GENERATION_EXHAUSTED";
    return;
  }
  last_event_token_ = event_token;
  ++snapshot_.generation;
  ++snapshot_.discontinuity_count;
  snapshot_.last_event = reason.empty() ? "PUBLIC_FRAME_DISCONTINUITY"
                                        : std::move(reason);
}

}  // namespace uav::nav::lio
