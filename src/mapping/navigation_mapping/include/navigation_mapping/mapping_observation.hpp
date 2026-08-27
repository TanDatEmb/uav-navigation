#pragma once

#include <cstdint>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>

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
};

}  // namespace navigation_mapping
