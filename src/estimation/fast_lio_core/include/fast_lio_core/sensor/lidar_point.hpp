#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace uav::nav::lio {

struct LidarPoint {
  Eigen::Vector3f position_lidar_m{Eigen::Vector3f::Zero()};
  std::uint32_t relative_time_ns{0};
  std::uint8_t reflectivity{0};
  std::uint8_t tag{0};
  std::uint8_t line{0};

  [[nodiscard]] bool allFinite() const noexcept { return position_lidar_m.allFinite(); }
};

}  // namespace uav::nav::lio
