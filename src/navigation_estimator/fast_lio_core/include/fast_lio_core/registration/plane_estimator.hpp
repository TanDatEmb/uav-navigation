#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <span>

#include "fast_lio_core/registration/correspondence.hpp"

namespace uav::nav::lio {

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
