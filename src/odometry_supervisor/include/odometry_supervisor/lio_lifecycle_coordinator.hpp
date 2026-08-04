#pragma once

#include <cstdint>
#include <optional>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

enum class LioLifecycleState : std::uint8_t {
  kStartup = 0,
  kTracking,
  kResetting,
  kLost,
  kReinitializing,
};

struct LioLifecycleObservation {
  std::uint64_t generation{0};
  bool valid{false};
  bool resetting{false};
  bool continuity_unrecoverable{false};
  std::optional<OdometryState> corrected;
};

// Last-good odometry-only evidence. It is not a warm-start state and is never
// applied to the estimator by this coordinator.
struct LioOdometryOnlySnapshot {
  std::uint64_t generation{0};
  OdometryState corrected;
  bool valid{false};
};

class LioLifecycleCoordinator {
 public:
  void observe(const LioLifecycleObservation& observation);
  [[nodiscard]] bool requestReinitialization();
  void acceptReinitialization(std::uint64_t new_generation);
  void clear();

  [[nodiscard]] LioLifecycleState state() const noexcept { return state_; }
  [[nodiscard]] const std::optional<LioOdometryOnlySnapshot>& snapshot() const noexcept {
    return snapshot_;
  }
  [[nodiscard]] bool trackingEverConfirmed() const noexcept {
    return tracking_ever_confirmed_;
  }
  [[nodiscard]] std::uint64_t reinitialization_count() const noexcept {
    return reinitialization_count_;
  }

 private:
  LioLifecycleState state_{LioLifecycleState::kStartup};
  std::optional<LioOdometryOnlySnapshot> snapshot_;
  std::uint64_t generation_{0};
  std::uint64_t reinitialization_count_{0};
  bool tracking_ever_confirmed_{false};
};

}  // namespace odometry_supervisor
