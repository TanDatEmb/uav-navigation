#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <nav_msgs/msg/odometry.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_mapping {

struct MappingObservation {
  // The actor owns the only mutable backend map.  The input cloud is
  // immutable after admission so no producer can race the mapping callback.
  // Unique ownership makes the immutable handoff enforceable at the type
  // boundary: a producer cannot retain a mutable alias after admission.
  std::unique_ptr<const PointCloud> cloud;
  nav_msgs::msg::Odometry corrected_odometry;
  std::uint64_t localization_epoch{0};
  std::uint64_t scan_sequence{0};
  std::int64_t stamp_ns{0};
  std::int64_t pointcloud_decode_us{0};
  // Optional explicit no-return endpoints.  These are already registered in
  // the outer/world frame and share stamp_ns with cloud.  Empty/absent means
  // no visibility evidence, never blanket free space.
  std::unique_ptr<const PointCloud> free_space_endpoints;
  // Registered endpoints are expressed in the outer frame.  Keep the base
  // pose and the actual sensor ray origin separate so a non-zero lever arm
  // cannot silently become a map-ray origin.
  std::optional<navigation_world_model::Point3> sensor_origin_world;
  // These identity fields are part of the same accepted scan contract; a
  // producer must not attach an origin from another epoch or timestamp.
  std::uint64_t sensor_origin_localization_epoch{0};
  std::int64_t sensor_origin_stamp_ns{0};
};

}  // namespace navigation_mapping
