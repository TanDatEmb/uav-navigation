#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "navigation_mission/mission.hpp"

namespace navigation_mission {

struct RouteProgressConfig {
  // Position noise may project a vehicle a small distance behind its previous
  // route position. Keep the reported route progress monotonic while exposing
  // larger reversals to the caller for policy decisions.
  double backtrack_tolerance_m{0.5};
};

struct RouteSegment {
  std::size_t start_waypoint_index{0U};
  std::size_t end_waypoint_index{0U};
  double start_arc_m{0.0};
  double end_arc_m{0.0};
  double length_m{0.0};
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::Zero()};
};

struct RouteProjection {
  bool valid{false};
  std::size_t segment_index{std::numeric_limits<std::size_t>::max()};
  double arc_length_m{0.0};
  double lateral_error_m{0.0};
  double segment_fraction{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::Zero()};
};

struct RouteProgressState {
  bool valid{false};
  bool backtracking_exceeded{false};
  RouteProjection projection{};
  double progress_arc_m{0.0};
};

// Pure route geometry/progress owner shared by mission and planner-facing
// adapters. It never accepts a waypoint; acceptance remains a measured-state
// policy owned by MissionController.
class RouteProgress final {
 public:
  explicit RouteProgress(const Mission& mission,
                          RouteProgressConfig config = {});

  void reset() noexcept;

  [[nodiscard]] RouteProjection project(const Eigen::Vector3d& position) const noexcept;
  [[nodiscard]] RouteProgressState update(const Eigen::Vector3d& position) noexcept;
  [[nodiscard]] RouteProgressState state() const noexcept { return state_; }

  [[nodiscard]] const std::vector<RouteSegment>& segments() const noexcept {
    return segments_;
  }
  [[nodiscard]] double totalLengthM() const noexcept { return total_length_m_; }
  [[nodiscard]] double waypointArcLengthM(std::size_t waypoint_index) const;
  [[nodiscard]] std::optional<Eigen::Vector3d> pointAtArc(double arc_length_m) const noexcept;
  [[nodiscard]] double altitudeAtArc(double arc_length_m) const noexcept;
  [[nodiscard]] std::optional<Eigen::Vector3d> incomingTangent(
      std::size_t waypoint_index) const noexcept;
  [[nodiscard]] std::optional<Eigen::Vector3d> outgoingTangent(
      std::size_t waypoint_index) const noexcept;
  [[nodiscard]] bool insideAcceptance(std::size_t waypoint_index,
                                      const Eigen::Vector3d& position) const noexcept;

 private:
  [[nodiscard]] std::size_t segmentForArc(double arc_length_m) const noexcept;

  std::vector<MissionWaypoint> waypoints_;
  std::vector<RouteSegment> segments_;
  std::vector<double> waypoint_arc_lengths_;
  RouteProgressConfig config_{};
  RouteProgressState state_{};
  double total_length_m_{0.0};
};

}  // namespace navigation_mission
