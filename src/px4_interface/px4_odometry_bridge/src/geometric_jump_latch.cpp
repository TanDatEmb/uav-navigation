#include "px4_odometry_bridge/geometric_jump_latch.hpp"

namespace px4_odometry_bridge {

bool GeometricJumpLatch::observeGeometricJump(const bool jump) {
  if (!jump || latched_) {
    return false;
  }
  latched_ = true;
  ++count_;
  return true;
}

bool GeometricJumpLatch::observePublicFrameGeneration(
    const bool valid, const std::uint64_t generation) {
  const bool changed = generation_known_ && valid && generation != last_generation_;
  if (valid && generation != 0) {
    generation_known_ = true;
    last_generation_ = generation;
  }
  if (changed) {
    latched_ = false;
  }
  return changed;
}

bool GeometricJumpLatch::observeOperatorReset(const bool requested,
                                              const std::uint64_t sequence) {
  const bool changed = requested && sequence != 0 &&
                       (!operator_sequence_known_ ||
                        sequence != last_operator_sequence_);
  if (sequence != 0) {
    operator_sequence_known_ = true;
    last_operator_sequence_ = sequence;
  }
  if (changed) {
    latched_ = false;
  }
  return changed;
}

}  // namespace px4_odometry_bridge
