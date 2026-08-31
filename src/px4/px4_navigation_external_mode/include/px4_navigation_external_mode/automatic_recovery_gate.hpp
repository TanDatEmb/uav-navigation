#pragma once

#include <cmath>
#include <cstdint>

namespace px4_navigation_external_mode {

inline constexpr double kAutomaticRecoveryStationarySpeedMps = 0.15;
inline constexpr std::int64_t kAutomaticRecoveryStationaryDurationNs =
    5'000'000'000LL;

class AutomaticRecoveryGate {
 public:
  [[nodiscard]] bool arm() {
    if (attempt_count_ >= 1U) {
      cancel();
      return false;
    }
    pending_ = true;
    hold_confirmed_ = false;
    stationary_since_ns_ = 0;
    return true;
  }

  void cancel() {
    pending_ = false;
    hold_confirmed_ = false;
    stationary_since_ns_ = 0;
  }

  [[nodiscard]] bool pending() const { return pending_; }
  [[nodiscard]] std::uint32_t attemptCount() const { return attempt_count_; }

  void resetBudget() {
    cancel();
    attempt_count_ = 0U;
  }

  void setHoldConfirmed(const bool confirmed, const std::int64_t now_steady_ns) {
    if (!pending_) return;
    if (!confirmed || now_steady_ns <= 0) {
      hold_confirmed_ = false;
      stationary_since_ns_ = 0;
      return;
    }
    if (!hold_confirmed_) {
      hold_confirmed_ = true;
      stationary_since_ns_ = 0;
    }
  }

  [[nodiscard]] bool consumeIfReady(const bool armed, const bool state_fresh,
                                    const bool health_fresh, const double speed_mps,
                                    const std::int64_t now_steady_ns) {
    if (!pending_) return false;
    const bool observation_valid = armed && hold_confirmed_ && state_fresh &&
        health_fresh && std::isfinite(speed_mps) && speed_mps >= 0.0 &&
        speed_mps <= kAutomaticRecoveryStationarySpeedMps && now_steady_ns > 0;
    if (!observation_valid) {
      stationary_since_ns_ = 0;
      return false;
    }
    if (stationary_since_ns_ == 0 || now_steady_ns < stationary_since_ns_) {
      stationary_since_ns_ = now_steady_ns;
      return false;
    }
    if (now_steady_ns - stationary_since_ns_ <
        kAutomaticRecoveryStationaryDurationNs) {
      return false;
    }
    ++attempt_count_;
    cancel();
    return true;
  }

 private:
  bool pending_{false};
  bool hold_confirmed_{false};
  std::int64_t stationary_since_ns_{0};
  std::uint32_t attempt_count_{0U};
};

}  // namespace px4_navigation_external_mode
