#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <navigation_planning_backend/route_yaw_reference.hpp>

namespace navigation_runtime {

struct HeadingTargetIdentity final {
  std::string mission_id;
  std::uint64_t route_revision{0U};
  std::uint32_t waypoint_index{0U};
  std::uint64_t request_id{0U};
  std::uint64_t localization_epoch{0U};

  [[nodiscard]] bool valid() const noexcept {
    return !mission_id.empty() && route_revision != 0U && request_id != 0U &&
           localization_epoch != 0U;
  }

  friend bool operator==(const HeadingTargetIdentity&,
                         const HeadingTargetIdentity&) = default;
};

struct HeadingTrackerSample final {
  bool valid{false};
  double yaw_rad{0.0};
  double yaw_rate_rad_s{0.0};
  double yaw_acceleration_rad_s2{0.0};
  double target_yaw_rad{0.0};
};

// Command-loop-owned heading state. The route reference is replaced when the
// active goal identity changes, but the published yaw/rate state is retained
// across ordinary waypoint handoffs. No planner bundle sample is used as the
// next state, so a stale solve cannot reset turn progress.
class HeadingTracker final {
 public:
  void reset() noexcept {
    initialized_ = false;
    target_valid_ = false;
    target_identity_ = {};
    last_command_stamp_ns_ = 0;
    yaw_rad_ = 0.0;
    yaw_rate_rad_s_ = 0.0;
    target_yaw_rad_ = 0.0;
  }

  [[nodiscard]] HeadingTrackerSample step(
      const HeadingTargetIdentity& identity, double target_yaw_rad,
      double measured_yaw_rad, std::int64_t command_stamp_ns,
      double maximum_yaw_rate_rad_s,
      double maximum_yaw_acceleration_rad_s2) noexcept {
    HeadingTrackerSample output;
    if (!identity.valid() || !std::isfinite(target_yaw_rad) ||
        !std::isfinite(measured_yaw_rad) || command_stamp_ns <= 0) {
      return output;
    }

    if (!initialized_) {
      initialized_ = true;
      yaw_rad_ = measured_yaw_rad;
      yaw_rate_rad_s_ = 0.0;
      last_command_stamp_ns_ = command_stamp_ns;
    } else if (command_stamp_ns <= last_command_stamp_ns_) {
      // A ROS clock reset or duplicate stamp must not create a negative dt.
      // Preserve the last command state and re-anchor the next valid tick.
      last_command_stamp_ns_ = command_stamp_ns;
      target_identity_ = identity;
      target_yaw_rad_ = target_yaw_rad;
      target_valid_ = true;
      output.valid = true;
      output.yaw_rad = yaw_rad_;
      output.yaw_rate_rad_s = yaw_rate_rad_s_;
      output.target_yaw_rad = target_yaw_rad_;
      return output;
    }

    const double dt_s = static_cast<double>(
        command_stamp_ns - last_command_stamp_ns_) * 1.0e-9;
    last_command_stamp_ns_ = command_stamp_ns;
    target_identity_ = identity;
    target_yaw_rad_ = target_yaw_rad;
    target_valid_ = true;
    if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > kMaximumClockGapS) {
      output.valid = true;
      output.yaw_rad = yaw_rad_;
      output.yaw_rate_rad_s = yaw_rate_rad_s_;
      output.target_yaw_rad = target_yaw_rad_;
      return output;
    }

    const auto next = navigation_planning_backend::stepBoundedHeading(
        yaw_rad_, yaw_rate_rad_s_, target_yaw_rad_, dt_s,
        maximum_yaw_rate_rad_s, maximum_yaw_acceleration_rad_s2);
    if (!next.valid) {
      output.valid = true;
      output.yaw_rad = yaw_rad_;
      output.yaw_rate_rad_s = yaw_rate_rad_s_;
      output.target_yaw_rad = target_yaw_rad_;
      return output;
    }
    yaw_rad_ = next.yaw_rad;
    yaw_rate_rad_s_ = next.yaw_rate_rad_s;
    output.valid = true;
    output.yaw_rad = yaw_rad_;
    output.yaw_rate_rad_s = yaw_rate_rad_s_;
    output.yaw_acceleration_rad_s2 = next.yaw_acceleration_rad_s2;
    output.target_yaw_rad = target_yaw_rad_;
    return output;
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] const HeadingTargetIdentity& targetIdentity() const noexcept {
    return target_identity_;
  }
  [[nodiscard]] double yawRad() const noexcept { return yaw_rad_; }
  [[nodiscard]] double yawRateRadS() const noexcept { return yaw_rate_rad_s_; }

 private:
  static constexpr double kMaximumClockGapS = 0.25;
  bool initialized_{false};
  bool target_valid_{false};
  HeadingTargetIdentity target_identity_{};
  std::int64_t last_command_stamp_ns_{0};
  double yaw_rad_{0.0};
  double yaw_rate_rad_s_{0.0};
  double target_yaw_rad_{0.0};
};

}  // namespace navigation_runtime
