#include "fast_lio_core/registration/plane_estimator.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav::nav::lio {

PlaneEstimator::PlaneEstimator(PlaneEstimatorConfig config) : config_(config) {
  if (config_.minimum_neighbor_count < 3U || !(config_.maximum_rms_error_m > 0.0) ||
      !(config_.maximum_point_error_m > 0.0) ||
      !(config_.maximum_smallest_eigenvalue_ratio > 0.0) ||
      config_.maximum_smallest_eigenvalue_ratio >= 1.0 ||
      !(config_.minimum_second_eigenvalue_ratio > 0.0) ||
      config_.minimum_second_eigenvalue_ratio >= 1.0) {
    throw std::invalid_argument("invalid plane estimator configuration");
  }
}

std::optional<Plane> PlaneEstimator::estimate(
    std::span<const Eigen::Vector3d> points_odom_m) const {
  if (points_odom_m.size() < config_.minimum_neighbor_count) {
    return std::nullopt;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& point : points_odom_m) {
    if (!point.allFinite()) {
      return std::nullopt;
    }
    centroid += point;
  }
  centroid /= static_cast<double>(points_odom_m.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d& point : points_odom_m) {
    const Eigen::Vector3d centered = point - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(points_odom_m.size());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    return std::nullopt;
  }

  const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const double eigenvalue_sum = eigenvalues.sum();
  if (!(eigenvalue_sum > 1e-12)) {
    return std::nullopt;
  }
  const double smallest_eigenvalue_ratio = eigenvalues.x() / eigenvalue_sum;
  const double second_eigenvalue_ratio = eigenvalues.y() / eigenvalue_sum;
  if (smallest_eigenvalue_ratio > config_.maximum_smallest_eigenvalue_ratio ||
      second_eigenvalue_ratio < config_.minimum_second_eigenvalue_ratio) {
    return std::nullopt;
  }

  Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  // Resolve the otherwise arbitrary eigenvector sign deterministically.
  Eigen::Index dominant_axis = 0;
  normal.cwiseAbs().maxCoeff(&dominant_axis);
  if (normal[dominant_axis] < 0.0) {
    normal = -normal;
  }

  double squared_error_sum = 0.0;
  double maximum_error = 0.0;
  for (const Eigen::Vector3d& point : points_odom_m) {
    const double error = std::abs(normal.dot(point - centroid));
    squared_error_sum += error * error;
    maximum_error = std::max(maximum_error, error);
  }
  const double rms_error = std::sqrt(squared_error_sum / static_cast<double>(points_odom_m.size()));
  if (rms_error > config_.maximum_rms_error_m || maximum_error > config_.maximum_point_error_m) {
    return std::nullopt;
  }

  Plane plane;
  plane.centroid_odom_m = centroid;
  plane.normal_odom = normal;
  plane.rms_error_m = rms_error;
  plane.maximum_error_m = maximum_error;
  plane.planarity = 1.0 - smallest_eigenvalue_ratio;
  return plane;
}

}  // namespace uav::nav::lio
