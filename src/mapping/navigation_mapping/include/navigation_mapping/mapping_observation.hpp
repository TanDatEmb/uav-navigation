#pragma once

#include <cstdint>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>

namespace navigation_mapping {

struct MappingObservation {
  std::shared_ptr<PointCloud> cloud;
  nav_msgs::msg::Odometry corrected_odometry;
  std::uint64_t localization_epoch{0};
  std::uint64_t scan_sequence{0};
  std::int64_t stamp_ns{0};
  std::int64_t pointcloud_decode_us{0};
  std::int64_t pair_wait_us{0};
};

}  // namespace navigation_mapping
