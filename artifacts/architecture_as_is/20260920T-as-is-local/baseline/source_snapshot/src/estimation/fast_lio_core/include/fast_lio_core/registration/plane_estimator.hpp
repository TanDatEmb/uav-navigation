#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <span>

namespace uav::nav::lio {

struct Plane {
  Eigen::Vector3d centroid_odom_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_odom{Eigen::Vector3d::UnitZ()};
  double rms_error_m{0.0};
  double maximum_error_m{0.0};
  double planarity{0.0};
};

struct PlaneEstimatorConfig {
  std::size_t minimum_neighbor_count{5};
  double maximum_rms_error_m{0.10};
  double maximum_point_error_m{0.20};
  double maximum_smallest_eigenvalue_ratio{0.10};
  double minimum_second_eigenvalue_ratio{0.01};
};

class PlaneEstimator {
 public:
  explicit PlaneEstimator(PlaneEstimatorConfig config = {});

  [[nodiscard]] std::optional<Plane> estimate(std::span<const Eigen::Vector3d> points_odom_m) const;

 private:
  PlaneEstimatorConfig config_;
};

}  // namespace uav::nav::lio
