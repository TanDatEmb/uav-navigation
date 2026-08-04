#include "odometry_supervisor/lio_lifecycle_coordinator.hpp"

namespace odometry_supervisor {

void LioLifecycleCoordinator::observe(const LioLifecycleObservation& observation) {
  if (generation_ != 0 && observation.generation != generation_) {
    snapshot_.reset();
    state_ = LioLifecycleState::kStartup;
  }
  generation_ = observation.generation;
  if (observation.resetting) {
    state_ = LioLifecycleState::kResetting;
    return;
  }
  if (observation.continuity_unrecoverable || !observation.valid) {
    state_ = LioLifecycleState::kLost;
    return;
  }
  if (observation.corrected.has_value() && observation.corrected->valid) {
    snapshot_ = LioReinitializationSnapshot{observation.generation, *observation.corrected, true};
    if (state_ == LioLifecycleState::kStartup || state_ == LioLifecycleState::kReinitializing) {
      state_ = LioLifecycleState::kTracking;
    }
  }
}

bool LioLifecycleCoordinator::requestReinitialization() {
  if (!snapshot_.has_value() || !snapshot_->valid || state_ != LioLifecycleState::kLost) {
    return false;
  }
  state_ = LioLifecycleState::kReinitializing;
  ++reinitialization_count_;
  return true;
}

void LioLifecycleCoordinator::acceptReinitialization(const std::uint64_t new_generation) {
  if (state_ != LioLifecycleState::kReinitializing || new_generation == generation_) return;
  generation_ = new_generation;
  snapshot_.reset();
  state_ = LioLifecycleState::kStartup;
}

void LioLifecycleCoordinator::clear() {
  state_ = LioLifecycleState::kStartup;
  snapshot_.reset();
  generation_ = 0;
  reinitialization_count_ = 0;
}

}  // namespace odometry_supervisor
