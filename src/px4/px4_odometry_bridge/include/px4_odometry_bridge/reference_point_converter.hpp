#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "px4_odometry_bridge/frame_converter.hpp"

namespace px4_odometry_bridge {

struct ReferencePointConfig {
  std::string source_body_frame{"base_link"};
  std::string output_body_frame{"base_link"};
  Eigen::Isometry3d base_from_source{Eigen::Isometry3d::Identity()};
};

class ReferencePointConverter {
 public:
  explicit ReferencePointConverter(ReferencePointConfig config)
      : config_(std::move(config)) {}

  ConvertedOdometry convert(const ConvertedOdometry &source) const;

 private:
  ReferencePointConfig config_;
};

}  // namespace px4_odometry_bridge
