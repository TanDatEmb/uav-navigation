#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

#include <Eigen/Core>

#include "px4_navigation_external_mode/mission.hpp"

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
  [[nodiscard]] MissionWaypoint activeWaypoint() const;
  [[nodiscard]] std::optional<MissionWaypoint> nextWaypoint() const;

 private:
  static constexpr double kSafetyStopSpeedMps = 0.15;
  static constexpr double kSafetyStopConfirmationS = 0.5;
  static constexpr double kSafetyStopTimeoutS = 5.0;
  static constexpr std::uint8_t kSafetyTrajectoryRole = 1U;
  static constexpr std::uint8_t kSafetyRouteKind = 1U;
  static constexpr std::uint8_t kSafetyStopKind = 2U;

  Mission mission_;
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
};

}  // namespace px4_navigation_external_mode
