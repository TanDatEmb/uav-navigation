#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace navigation_runtime {

// PropagatedOdometry currently transports position and velocity, not
// acceleration or jerk.  This estimator makes that boundary explicit: the
// first finite difference is an estimate, never a measured value, and a time
// or localization discontinuity invalidates the derivative history.
struct KinematicDerivativeEstimate {
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_world{Eigen::Vector3d::Zero()};
  bool acceleration_estimated{false};
  bool jerk_estimated{false};
};

class KinematicDerivativeEstimator final {
 public:
  KinematicDerivativeEstimator() = default;

  void reset() noexcept {
    have_velocity_ = false;
    have_acceleration_ = false;
    previous_stamp_ns_ = 0;
    previous_epoch_ = 0;
    previous_velocity_.setZero();
    previous_acceleration_.setZero();
  }

  [[nodiscard]] KinematicDerivativeEstimate update(
      const std::int64_t stamp_ns,
      const std::uint64_t localization_epoch,
      const Eigen::Vector3d& velocity_world) noexcept {
    KinematicDerivativeEstimate result;
    if (stamp_ns <= 0 || localization_epoch == 0U || !velocity_world.allFinite()) {
      reset();
      return result;
    }

    const bool continuous_sample =
        have_velocity_ && previous_epoch_ == localization_epoch &&
        stamp_ns > previous_stamp_ns_ &&
        stamp_ns - previous_stamp_ns_ <= kMaximumDerivativeGapNs;
    if (continuous_sample) {
      const double dt_s = static_cast<double>(stamp_ns - previous_stamp_ns_) * 1.0e-9;
      const Eigen::Vector3d acceleration =
          (velocity_world - previous_velocity_) / dt_s;
      if (acceleration.allFinite()) {
        result.acceleration_world = acceleration;
        result.acceleration_estimated = true;
        if (have_acceleration_) {
          const Eigen::Vector3d jerk =
              (acceleration - previous_acceleration_) / dt_s;
          if (jerk.allFinite()) {
            result.jerk_world = jerk;
            result.jerk_estimated = true;
          }
        }
        previous_acceleration_ = acceleration;
        have_acceleration_ = true;
      } else {
        have_acceleration_ = false;
        previous_acceleration_.setZero();
      }
    } else {
      have_acceleration_ = false;
      previous_acceleration_.setZero();
    }

    previous_stamp_ns_ = stamp_ns;
    previous_epoch_ = localization_epoch;
    previous_velocity_ = velocity_world;
    have_velocity_ = true;
    return result;
  }

 private:
  // A gap longer than the runtime's default freshness window cannot support a
  // meaningful finite difference.  The next sample starts a new derivative
  // history; it does not manufacture acceleration from stale state.
  static constexpr std::int64_t kMaximumDerivativeGapNs = 500'000'000;

  bool have_velocity_{false};
  bool have_acceleration_{false};
  std::int64_t previous_stamp_ns_{0};
  std::uint64_t previous_epoch_{0U};
  Eigen::Vector3d previous_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_acceleration_{Eigen::Vector3d::Zero()};
};

}  // namespace navigation_runtime
