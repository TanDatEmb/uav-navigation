#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace uav::nav::lio {

struct Plane {
  Eigen::Vector3d centroid_odom_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_odom{Eigen::Vector3d::UnitZ()};
  double rms_error_m{0.0};
  double maximum_error_m{0.0};
  double planarity{0.0};
};

struct Correspondence {
  std::size_t point_index{0};
  Eigen::Vector3d point_lidar_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d point_odom_m{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> neighbors_odom_m;
  Plane plane{};
  double signed_distance_m{0.0};
};

}  // namespace uav::nav::lio
