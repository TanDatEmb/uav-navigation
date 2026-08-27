#pragma once

#include <cstdint>
#include <string>

namespace uav::nav::lio {

// This generation belongs to the public LIO odometry contract. It is
// intentionally independent from estimator/control generations: a
// corrected-to-propagated handoff and an internal restart are not public
// frame changes when the published lio_odom stream remains continuous.
enum class PublicFrameEvent {
  kContinuous,
  kInternalLioGenerationChange,
  kCorrectedPropagatedHandoff,
  kPx4Reset,
  kPublicFrameDiscontinuity,
};

struct LioPublicFrameGenerationSnapshot {
  std::uint64_t generation{1};
  bool valid{true};
  std::uint64_t discontinuity_count{0};
  std::string last_event{"INITIAL_PUBLIC_FRAME"};
};

class LioPublicFrameGeneration final {
 public:
  explicit LioPublicFrameGeneration(std::uint64_t initial_generation = 1U) noexcept {
    snapshot_.generation = initial_generation == 0U ? 1U : initial_generation;
  }

  [[nodiscard]] LioPublicFrameGenerationSnapshot snapshot() const noexcept {
    return snapshot_;
  }

  // event_token is the producer-owned identity of the discontinuity. A
  // repeated observation of the same event is deliberately idempotent.
  void observe(PublicFrameEvent event, std::uint64_t event_token = 0,
               std::string reason = {});

 private:
  LioPublicFrameGenerationSnapshot snapshot_{};
  std::uint64_t last_event_token_{0};
};

}  // namespace uav::nav::lio
