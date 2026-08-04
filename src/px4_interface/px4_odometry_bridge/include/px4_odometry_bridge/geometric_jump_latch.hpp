#pragma once

#include <cstdint>

namespace px4_odometry_bridge {

class GeometricJumpLatch final {
 public:
  // Returns true only for the first jump in the current latch epoch.
  bool observeGeometricJump(bool jump);

  // PX4 reset/frame generations and gate transitions intentionally have no
  // method here. Only a new producer-owned public LIO generation may recover.
  bool observePublicFrameGeneration(bool valid, std::uint64_t generation);
  bool observeOperatorReset(bool requested, std::uint64_t sequence);

  [[nodiscard]] bool latched() const noexcept { return latched_; }
  [[nodiscard]] std::uint64_t count() const noexcept { return count_; }

 private:
  bool latched_{false};
  std::uint64_t count_{0};
  std::uint64_t last_generation_{0};
  std::uint64_t last_operator_sequence_{0};
  bool generation_known_{false};
  bool operator_sequence_known_{false};
};

[[nodiscard]] constexpr std::uint8_t public_frame_generation_to_reset_counter(
    std::uint64_t generation) noexcept {
  return static_cast<std::uint8_t>(generation % 256U);
}

}  // namespace px4_odometry_bridge
