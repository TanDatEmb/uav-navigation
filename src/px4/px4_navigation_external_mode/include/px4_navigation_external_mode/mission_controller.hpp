#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

#include <Eigen/Core>

#include "px4_navigation_external_mode/mission.hpp"
#include "navigation_mission/route_progress.hpp"

namespace px4_navigation_external_mode {

enum class MissionControllerState {
  Idle,
  WaitingForAirborne,
  ExecutingWaypoint,
  Braking,
  Paused,
  Holding,
  Complete,
  Failed
};

struct MissionControllerEvent {
  enum class Type { None, PublishGoal, RequestPositionControl, Complete, Failure };
  Type type{Type::None};
  std::size_t waypoint_index{0U};
  std::uint64_t request_id{0U};
  bool waypoint_accepted{false};
  std::size_t accepted_waypoint_index{0U};
  double acceptance_position_error_m{0.0};
  double acceptance_speed_mps{0.0};
};

class MissionController final {
 public:
  explicit MissionController(Mission mission);

  void activate(double now_s);
  void deactivate();
  void onTrajectory(bool success, double now_s);
  void onTrajectory(bool success, std::uint8_t trajectory_role, double now_s);
  void onTrajectory(bool success, std::uint8_t trajectory_role,
                    std::uint8_t safety_plan_kind, double now_s,
                    double duration_s = 0.0);
  // Native planner backend trajectories do not use the legacy PlannedTrajectory
  // lifecycle. Latch readiness independently of the airborne transition so a
  // callback that races activate()/update() cannot strand the mission on the
  // first waypoint.
  void onNativeTrajectoryReady();
  // A completed native MAIN trajectory may end inside a corner waypoint's
  // acceptance ball while the measured vehicle velocity is still non-zero.
  // Keep the certified terminal hold until the measured state satisfies the
  // normal acceptance gate; do not re-publish the same goal and restart the
  // planner in the meantime.
  void onNativeTerminalHoldObserved();
  // A completed certified command may stop while the measured vehicle remains
  // outside the waypoint acceptance ball. Re-issue the same mission checkpoint
  // once so runtime plans a measured-state terminal connector; the executor
  // keeps the endpoint hold during its existing bounded recovery window. A
  // waypoint can request this retry only once.
  void requestNativeTerminalRecovery(double now_s);

  [[nodiscard]] MissionControllerEvent update(double now_s,
                                               const std::optional<Eigen::Vector3d>& position,
                                               bool airborne = true,
                                               const std::optional<Eigen::Vector3d>& velocity =
                                                   std::nullopt);
  [[nodiscard]] MissionControllerState state() const;
  [[nodiscard]] bool holding() const;
  [[nodiscard]] bool waitingForAirborne() const;
  [[nodiscard]] std::size_t activeWaypointIndex() const;
  [[nodiscard]] std::uint64_t activeRequestId() const;
  [[nodiscard]] std::optional<MissionWaypoint> activeWaypoint() const;
  [[nodiscard]] std::optional<MissionWaypoint> waypointAt(std::size_t index) const;
  [[nodiscard]] std::optional<MissionWaypoint> nextWaypoint() const;
  [[nodiscard]] bool nativeTrajectoryReady() const;
  [[nodiscard]] bool terminalHoldPending() const;
  [[nodiscard]] double acceptanceSpeedMps() const;
  [[nodiscard]] navigation_mission::ImmutableRouteSnapshot routeSnapshot() const;
  [[nodiscard]] bool activePassThroughHasCoincidentStop() const;

 private:
  static constexpr double kSafetyStopSpeedMps = 0.15;
  static constexpr double kSafetyStopConfirmationS = 0.5;
  static constexpr double kSafetyStopTimeoutS = 5.0;
  static constexpr double kMaximumPassThroughSampleGapS = 0.25;
  static constexpr std::uint8_t kSafetyTrajectoryRole = 1U;
  static constexpr std::uint8_t kSafetyRouteKind = 1U;
  static constexpr std::uint8_t kSafetyStopKind = 2U;

  [[nodiscard]] bool advanceRequestId() noexcept;

  Mission mission_;
  navigation_mission::RouteProgress route_progress_;
  mutable std::mutex mutex_;
  MissionControllerState state_{MissionControllerState::Idle};
  std::size_t active_waypoint_index_{0U};
  double next_goal_time_s_{0.0};
  double hold_start_time_s_{0.0};
  std::uint64_t request_id_{0U};
  std::optional<double> braking_start_time_s_;
  std::optional<double> braking_end_time_s_;
  std::optional<double> stopped_start_time_s_;
  std::optional<double> arrival_start_time_s_;
  bool pending_position_control_{false};
  bool checkpoint_valid_{false};
  bool trajectory_ready_{false};
  bool terminal_hold_pending_{false};
  bool terminal_recovery_requested_{false};
  std::optional<Eigen::Vector3d> previous_position_;
  double previous_position_time_s_{0.0};
};

}  // namespace px4_navigation_external_mode
