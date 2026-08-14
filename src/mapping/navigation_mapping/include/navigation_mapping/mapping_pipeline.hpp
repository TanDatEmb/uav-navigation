#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "navigation_mapping/generation_tracker.hpp"
#include "navigation_mapping/collision_clearance.hpp"
#include "navigation_mapping/mapping_diagnostics.hpp"
#include "navigation_mapping/mapping_point_filter.hpp"
#include "navigation_mapping/observation_validator.hpp"
#include "navigation_mapping/rog_map_adapter.hpp"

namespace navigation_mapping {

struct MappingPipelineConfig {
  ObservationContract contract{};
  MappingPointFilterConfig point_filter{};
  RogMapProductConfig rog{};
  CollisionClearanceConfig collision{};
};

struct ObservationInput {
  std::string header_frame_id;
  builtin_interfaces::msg::Time header_stamp;
  std::string points_frame_id;
  builtin_interfaces::msg::Time points_stamp;
  geometry_msgs::msg::Pose sensor_pose;
  // Already extracted XYZ points, in the sensor (lidar) frame. Extraction
  // from sensor_msgs::msg::PointCloud2 is a ROS-boundary concern and lives in
  // NavigationMappingNode, not here (mirrors fast_lio_core/fast_lio_ros split).
  std::vector<Point3f> points_lidar_m;
  std::uint64_t public_frame_generation{0};
};

// ROS-independent mapping pipeline: validation, generation handling, mapping
// downsample, transform, and ROG update. NavigationMappingNode only adapts
// ROS messages to/from this class, matching the repository's estimator-core
// / estimator-ROS split (see docs/architecture/repository_layout.md).
class MappingPipeline {
 public:
  MappingPipeline(MappingPipelineConfig config, std::function<double()> wall_clock_seconds,
                  std::string generated_config_directory);

  // Processes exactly one observation to completion with no internal queue.
  // Never throws on malformed input; all rejection paths
  // are diagnostic-counted instead.
  void process(const ObservationInput& input);

  [[nodiscard]] const MappingDiagnostics& diagnostics() const noexcept { return diagnostics_; }
  [[nodiscard]] MappingDiagnostics& diagnostics() noexcept { return diagnostics_; }
  [[nodiscard]] RogMapAdapter& adapter() noexcept { return adapter_; }
  [[nodiscard]] const RogMapAdapter& adapter() const noexcept { return adapter_; }

 private:
  MappingPipelineConfig config_;
  ObservationValidator validator_;
  MappingPointFilter point_filter_;
  GenerationTracker generation_tracker_;
  RogMapAdapter adapter_;
  MappingDiagnostics diagnostics_;
};

}  // namespace navigation_mapping
