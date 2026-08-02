#pragma once

#include <cstdint>
#include <optional>

#include "px4_odometry_bridge/frame_converter.hpp"

namespace px4_odometry_bridge {

struct DetailedResetMetadata {
  bool available{false};
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
  std::optional<ConvertedOdometry> last_output_;
};

}  // namespace px4_odometry_bridge
