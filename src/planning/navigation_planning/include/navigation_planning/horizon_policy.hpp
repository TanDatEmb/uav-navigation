#pragma once

#include <limits>
#include <optional>

namespace navigation_planning {

struct RouteProgress {
  double projected_arc_length_m{0.0};
  double route_length_m{0.0};
  double usable_forward_distance_m{std::numeric_limits<double>::infinity()};
  std::optional<double> previous_projected_arc_length_m;
};

struct HorizonPolicyConfig {
  double minimum_distance_m{10.0};
  double maximum_distance_m{30.0};
  double preview_time_s{5.0};
  double stopping_distance_factor{2.0};
  double map_boundary_margin_m{2.0};
  double projection_tolerance_m{1e-6};
};

struct HorizonRequest {
  RouteProgress route;
  double speed_mps{0.0};
  double max_deceleration_mps2{1.0};
  double pipeline_latency_s{0.0};
  double stop_margin_m{0.0};
  std::optional<double> terminal_waypoint_arc_length_m;
};

enum class HorizonFailureCode {
  None,
  InvalidInput,
  BackwardProjection,
  TerminalBehindProjection,
  NoUsableForwardDistance,
};

struct PlanningHorizon {
  bool success{false};
  HorizonFailureCode failure{HorizonFailureCode::None};
  double start_arc_length_m{0.0};
  double endpoint_arc_length_m{0.0};
  double forward_distance_m{0.0};
  double unconstrained_distance_m{0.0};
  double usable_forward_distance_m{0.0};
  bool shortened_by_usable_space{false};
  bool terminal_waypoint_clamped{false};
};

class HorizonPolicy final {
 public:
  explicit HorizonPolicy(HorizonPolicyConfig config = {});

  [[nodiscard]] const HorizonPolicyConfig& config() const noexcept { return config_; }
  [[nodiscard]] PlanningHorizon compute(const HorizonRequest& request) const noexcept;

 private:
  HorizonPolicyConfig config_;
};

}  // namespace navigation_planning
