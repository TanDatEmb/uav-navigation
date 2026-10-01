#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <navigation_mission/mission.hpp>
#include <navigation_mission/route_progress.hpp>

namespace navigation_runtime {

// Mission policy has one writer: the Core callback transaction. PX4 lifecycle,
// execution role and command admission are observations supplied by their
// respective owners, never copied into this state as a second authority.
struct MissionRouteIdentity {
  std::string mission_id;
  std::uint64_t revision{1U};
  std::uint64_t localization_epoch{0U};
};

struct MissionGateIdentity {
  std::uint32_t waypoint_index{0U};
  std::uint64_t request_id{0U};
};

struct MissionMeasuredSample {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  std::int64_t source_stamp_ns{0};
  std::uint64_t sequence{0U};
  std::uint64_t localization_epoch{0U};

  [[nodiscard]] bool valid() const noexcept;
};

struct MissionCrossingWitness {
  MissionRouteIdentity route;
  MissionGateIdentity gate;
  MissionMeasuredSample before;
  MissionMeasuredSample after;
  navigation_mission::RouteProjection before_cursor;
  navigation_mission::RouteProjection after_cursor;
  double crossing_error_m{0.0};
};

struct MissionContinuationWitness {
  enum class Kind { Main, TerminalHold, SafetySuffixStop };
  MissionRouteIdentity route;
  MissionGateIdentity gate;
  Kind kind{Kind::Main};
  std::uint64_t bundle_generation{0U};
  std::int64_t boundary_stamp_ns{0};
  std::int64_t valid_until_ns{0};
};

struct MissionProgressDecision {
  enum class Kind { None, Goal, Complete } kind{Kind::None};
  MissionGateIdentity gate;
  std::optional<MissionGateIdentity> accepted;
  double acceptance_error_m{0.0};
  double acceptance_speed_mps{0.0};
};

class MissionProgress final {
 public:
  explicit MissionProgress(navigation_mission::Mission mission);

  // Only the Core owner transaction invokes mutations. A new localization
  // epoch or route revision discards physical observations and certificates.
  void resetIdentity(std::uint64_t route_revision,
                     std::uint64_t localization_epoch);
  [[nodiscard]] MissionProgressDecision activate();
  void deactivate();
  [[nodiscard]] MissionProgressDecision observeContinuation(
      const MissionContinuationWitness& witness, std::int64_t now_ns);
  [[nodiscard]] MissionProgressDecision observeMeasured(
      const MissionMeasuredSample& sample, std::int64_t now_ns);

  [[nodiscard]] const navigation_mission::Mission& mission() const noexcept {
    return mission_;
  }
  [[nodiscard]] const MissionRouteIdentity& routeIdentity() const noexcept {
    return identity_;
  }
  [[nodiscard]] MissionGateIdentity currentGate() const noexcept { return gate_; }
  [[nodiscard]] std::optional<MissionGateIdentity> acceptedGate() const noexcept {
    return accepted_;
  }
  [[nodiscard]] std::optional<MissionCrossingWitness> crossing() const {
    return crossing_;
  }
  [[nodiscard]] bool complete() const noexcept {
    return accepted_.has_value() &&
        accepted_->waypoint_index + 1U == mission_.waypoints.size();
  }
  [[nodiscard]] navigation_mission::ImmutableRouteSnapshot routeSnapshot() const;

 private:
  [[nodiscard]] MissionProgressDecision accept(double error_m, double speed_mps);
  [[nodiscard]] bool continuationMatches() const noexcept;
  [[nodiscard]] bool isCoincidentTerminalStop() const;

  const navigation_mission::Mission mission_;
  navigation_mission::RouteProgress measured_route_;
  MissionRouteIdentity identity_;
  MissionGateIdentity gate_{};
  std::optional<MissionGateIdentity> accepted_;
  std::optional<MissionMeasuredSample> previous_;
  std::optional<MissionCrossingWitness> crossing_;
  std::optional<MissionContinuationWitness> continuation_;
  std::optional<MissionMeasuredSample> latest_;
  // A STOP requires continuous measured confirmation followed by its hold.
  // These source timestamps preserve temporal information across callbacks.
  std::optional<std::int64_t> stop_confirmation_start_ns_;
  std::optional<std::int64_t> stop_hold_start_ns_;
  bool active_{false};
};

}  // namespace navigation_runtime
