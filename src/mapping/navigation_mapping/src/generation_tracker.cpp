#include "navigation_mapping/generation_tracker.hpp"

namespace navigation_mapping {

GenerationDecision GenerationTracker::decide(std::uint64_t observed_generation) const noexcept {
  if (!has_generation_) {
    return GenerationDecision::kResetAndAdoptNewGeneration;
  }
  if (observed_generation == current_generation_) {
    return GenerationDecision::kAcceptCurrentGeneration;
  }
  if (observed_generation > current_generation_) {
    return GenerationDecision::kResetAndAdoptNewGeneration;
  }
  return GenerationDecision::kRejectStaleGeneration;
}

void GenerationTracker::adopt(std::uint64_t observed_generation) {
  has_generation_ = true;
  current_generation_ = observed_generation;
  ++reset_count_;
}

}  // namespace navigation_mapping
