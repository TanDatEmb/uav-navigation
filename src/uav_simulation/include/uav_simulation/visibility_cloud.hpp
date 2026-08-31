#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <gz/msgs/laserscan.pb.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace uav::simulation {

struct VisibilityEndpoint {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct VisibilityCloud {
  std::string frame_id;
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0U};
  std::vector<VisibilityEndpoint> endpoints;
  std::uint32_t source_ray_count{0U};
};

struct OrganizedVisibilityConfig {
  std::uint32_t horizontal_count{0U};
  std::uint32_t vertical_count{0U};
  double horizontal_angle_min_rad{0.0};
  double horizontal_angle_max_rad{0.0};
  double vertical_angle_min_rad{0.0};
  double vertical_angle_max_rad{0.0};
  double range_max_m{0.0};
};

// Converts only explicit no-return rays to endpoints. A malformed message,
// incomplete 3D array or missing timestamp returns nullopt. A valid scan with
// no explicit no-return ray returns a present cloud with zero endpoints.
[[nodiscard]] std::optional<VisibilityCloud> makeVisibilityCloud(
    const gz::msgs::LaserScan& scan, std::string_view expected_frame,
    std::size_t maximum_endpoints = 4096U);

// Gazebo's organized PointCloudPacked conversion preserves explicit no-return
// rays as signed infinities. Reconstruct their directions from the immutable
// scan grid so hit and visibility evidence share one transport stream.
[[nodiscard]] std::optional<VisibilityCloud> makeVisibilityCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    const OrganizedVisibilityConfig& config, std::string_view expected_frame,
    std::size_t maximum_endpoints = 4096U);

}  // namespace uav::simulation
