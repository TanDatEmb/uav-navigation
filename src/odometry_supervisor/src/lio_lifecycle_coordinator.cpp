#include "odometry_supervisor/lio_lifecycle_coordinator.hpp"

namespace odometry_supervisor {

void LioLifecycleCoordinator::observe(const LioLifecycleObservation& observation) {
  if (generation_ != 0 && observation.generation != generation_) {
    snapshot_.reset();
    state_ = LioLifecycleState::kStartup;
    tracking_ever_confirmed_ = false;
  }
  generation_ = observation.generation;
  if (observation.resetting) {
    state_ = LioLifecycleState::kResetting;
    return;
  }
  if (observation.continuity_unrecoverable) {
    state_ = LioLifecycleState::kLost;
    return;
  }
  if (!observation.valid) {
    state_ = tracking_ever_confirmed_ ? LioLifecycleState::kLost
                                      : LioLifecycleState::kStartup;
    return;
  }
  tracking_ever_confirmed_ = true;
  if (observation.corrected.has_value() && observation.corrected->valid) {
    snapshot_ = LioOdometryOnlySnapshot{observation.generation, *observation.corrected, true};
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
  tracking_ever_confirmed_ = false;
  state_ = LioLifecycleState::kStartup;
}

void LioLifecycleCoordinator::clear() {
  state_ = LioLifecycleState::kStartup;
  snapshot_.reset();
  generation_ = 0;
  reinitialization_count_ = 0;
  tracking_ever_confirmed_ = false;
}

}  // namespace odometry_supervisor
