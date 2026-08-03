#pragma once

#include <cstdint>
#include <Eigen/Geometry>
#include <optional>

#include "px4_odometry_bridge/frame_converter.hpp"

namespace px4_odometry_bridge {

struct DetailedResetMetadata {
  std::int64_t timestamp_ns{0};
  bool available{false};
  bool position_xy_reset{false};
  bool position_z_reset{false};
  bool velocity_xy_reset{false};
  bool velocity_z_reset{false};
  bool heading_reset{false};
  bool attitude_reset{false};

  Eigen::Vector3d position_delta_source{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_delta_source{Eigen::Vector3d::Zero()};
  double heading_delta_rad{0.0};
  Eigen::Quaterniond attitude_delta{Eigen::Quaterniond::Identity()};

  std::uint8_t xy_reset_counter{0};
  std::uint8_t z_reset_counter{0};
  std::uint8_t vxy_reset_counter{0};
  std::uint8_t vz_reset_counter{0};
  std::uint8_t heading_reset_counter{0};
  std::uint8_t attitude_reset_counter{0};

  [[nodiscard]] bool hasReset() const noexcept {
    return position_xy_reset || position_z_reset || velocity_xy_reset ||
           velocity_z_reset || heading_reset || attitude_reset;
  }
};

class ResetCompensator {
 public:
  std::optional<ConvertedOdometry> observe(ConvertedOdometry sample,
                                           DetailedResetMetadata metadata = {});
  void clear();
  std::uint64_t reset_generation() const { return reset_generation_; }

 private:
  bool initialized_{false};
  std::uint8_t last_counter_{0};
  std::uint64_t reset_generation_{0};
  Eigen::Matrix3d continuity_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d continuity_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d continuity_velocity_translation_{Eigen::Vector3d::Zero()};
  std::optional<ConvertedOdometry> last_output_;
};

}  // namespace px4_odometry_bridge
