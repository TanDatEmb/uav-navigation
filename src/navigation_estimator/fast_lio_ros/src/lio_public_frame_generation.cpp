#include "fast_lio_ros/lio_public_frame_generation.hpp"

#include <utility>

namespace uav::nav::lio {

void LioPublicFrameGeneration::observe(const PublicFrameEvent event,
                                       const std::uint64_t event_token,
                                       std::string reason) {
  if (event != PublicFrameEvent::kPublicFrameDiscontinuity) {
    return;
  }
  if (event_token == 0 || event_token == last_event_token_) {
    return;
  }
  last_event_token_ = event_token;
  ++snapshot_.generation;
  ++snapshot_.discontinuity_count;
  snapshot_.last_event = reason.empty() ? "PUBLIC_FRAME_DISCONTINUITY"
                                        : std::move(reason);
}

}  // namespace uav::nav::lio
