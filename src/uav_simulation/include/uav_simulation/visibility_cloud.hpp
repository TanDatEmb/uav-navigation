#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <gz/msgs/laserscan.pb.h>

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
};

// Converts only explicit no-return rays to endpoints. A malformed message,
// incomplete 3D array or missing timestamp returns nullopt: the consumer must
// then treat the update as having no visibility evidence.
[[nodiscard]] std::optional<VisibilityCloud> makeVisibilityCloud(
    const gz::msgs::LaserScan& scan, std::string_view expected_frame,
    std::size_t maximum_endpoints = 4096U);

}  // namespace uav::simulation
