#pragma once

#include <cstdint>
#include <string>

namespace navigation_mapping {

// Encapsulates the P1 public-frame-generation contract (see
// docs/architecture/navigation_layers.md and
// fast_lio_ros/lio_public_frame_generation.hpp, which owns the authoritative
// generation counter). This class does not invent a second generation
// mechanism: it only decides what a *newly observed* generation value means
// for the navigation world model's lifecycle.
enum class GenerationDecision {
  // observation.generation == current tracked generation: process normally.
  kAcceptCurrentGeneration,
  // observation.generation > current tracked generation: the public frame
  // had a discontinuity. The map must be reset before this observation (or
  // any observation belonging to the new generation) is processed.
  kResetAndAdoptNewGeneration,
  // observation.generation < current tracked generation: stale data from a
  // frame that no longer exists. Must be rejected, never processed.
  kRejectStaleGeneration,
};

class GenerationTracker {
 public:
  // The tracker starts with no adopted generation. The very first observed
  // generation is always adopted (treated as a reset-and-adopt) so the map
  // has a well-defined generation before any observation is processed.
  GenerationTracker() = default;

  [[nodiscard]] GenerationDecision decide(std::uint64_t observed_generation) const noexcept;

  // Must be called exactly once after acting on a kResetAndAdoptNewGeneration
  // decision (i.e. after the map has actually been reset), so the tracker's
  // notion of "current" only advances once the reset has really happened.
  void adopt(std::uint64_t observed_generation);

  [[nodiscard]] bool hasGeneration() const noexcept { return has_generation_; }
  [[nodiscard]] std::uint64_t currentGeneration() const noexcept { return current_generation_; }
  [[nodiscard]] std::uint64_t resetCount() const noexcept { return reset_count_; }
  [[nodiscard]] std::uint64_t staleRejectedCount() const noexcept { return stale_rejected_count_; }

  // Test/diagnostic-only: records a rejection without adopting.
  void recordStaleRejection() noexcept { ++stale_rejected_count_; }

 private:
  bool has_generation_{false};
  std::uint64_t current_generation_{0};
  std::uint64_t reset_count_{0};
  std::uint64_t stale_rejected_count_{0};
};

}  // namespace navigation_mapping
