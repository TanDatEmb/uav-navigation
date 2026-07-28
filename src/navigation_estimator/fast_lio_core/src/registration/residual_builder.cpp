#include "fast_lio_core/registration/residual_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace uav::nav::lio {
namespace {

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(), 0.0;
  return skew;
}

}  // namespace

ResidualBuilder::ResidualBuilder(ResidualBuilderConfig config)
    : config_(config),
      correspondence_search_(config_.correspondence_search),
      plane_estimator_(config_.plane_estimator),
      residual_gate_(config_.residual_gate) {
  if (!(config_.point_measurement_standard_deviation_m > 0.0) ||
      !std::isfinite(config_.point_measurement_standard_deviation_m)) {
    throw std::invalid_argument("point measurement standard deviation must be positive");
  }
}

ResidualBuildResult ResidualBuilder::build(std::span<const Eigen::Vector3d> points_lidar_m,
                                           const ManifoldState& state,
                                           const RegistrationMap& map) const {
  ResidualBuildResult result;
  result.diagnostics.input_point_count = points_lidar_m.size();
  if (!state.allFinite() || points_lidar_m.empty()) {
    return result;
  }

  std::vector<Eigen::Vector3d> points_odom_m;
  points_odom_m.reserve(points_lidar_m.size());
  for (const Eigen::Vector3d& point_lidar_m : points_lidar_m) {
    points_odom_m.push_back(state.transformLidarPointToOdom(point_lidar_m));
  }

  CorrespondenceSearchResult search =
      correspondence_search_.search(points_lidar_m, points_odom_m, map);
  result.diagnostics.query_count = search.query_count;
  result.diagnostics.insufficient_neighbor_count = search.insufficient_neighbor_count;

  using JacobianRow = Eigen::Matrix<double, 1, ManifoldState::kErrorStateDimension>;
  std::vector<JacobianRow> jacobian_rows;
  std::vector<double> residuals;
  std::vector<double> variances;
  jacobian_rows.reserve(search.correspondences.size());
  residuals.reserve(search.correspondences.size());
  variances.reserve(search.correspondences.size());
  result.accepted_correspondences.reserve(search.correspondences.size());

  const Eigen::Matrix3d rotation_odom_imu = state.orientation_odom_imu().toRotationMatrix();
  const Eigen::Matrix3d rotation_imu_lidar = state.rotation_imu_lidar().toRotationMatrix();
  const double sensor_variance = config_.point_measurement_standard_deviation_m *
                                 config_.point_measurement_standard_deviation_m;

  for (Correspondence& correspondence : search.correspondences) {
    const std::optional<Plane> plane = plane_estimator_.estimate(correspondence.neighbors_odom_m);
    if (!plane.has_value()) {
      ++result.diagnostics.rejected_plane_count;
      continue;
    }
    ++result.diagnostics.valid_plane_count;
    correspondence.plane = *plane;
    correspondence.signed_distance_m =
        plane->normal_odom.dot(correspondence.point_odom_m - plane->centroid_odom_m);
    const ResidualGateDecision decision =
        residual_gate_.evaluate(*plane, correspondence.signed_distance_m);
    if (!decision.accepted) {
      ++result.diagnostics.rejected_residual_count;
      continue;
    }

    const Eigen::Vector3d point_imu_m =
        rotation_imu_lidar * correspondence.point_lidar_m + state.position_imu_lidar_m();
    JacobianRow jacobian = JacobianRow::Zero();
    jacobian.segment<3>(ManifoldState::kPositionOffset) = plane->normal_odom.transpose();
    jacobian.segment<3>(ManifoldState::kOrientationOffset) =
        plane->normal_odom.transpose() * (-rotation_odom_imu * skewSymmetric(point_imu_m));
    if (config_.estimate_extrinsic) {
      jacobian.segment<3>(ManifoldState::kExtrinsicRotationOffset) =
          plane->normal_odom.transpose() *
          (-rotation_odom_imu * rotation_imu_lidar * skewSymmetric(correspondence.point_lidar_m));
      jacobian.segment<3>(ManifoldState::kExtrinsicPositionOffset) =
          plane->normal_odom.transpose() * rotation_odom_imu;
    }

    jacobian_rows.push_back(jacobian);
    residuals.push_back(correspondence.signed_distance_m);
    const double plane_variance = plane->rms_error_m * plane->rms_error_m;
    variances.push_back((sensor_variance + plane_variance) /
                        std::max(decision.robust_weight, 1e-6));
    result.accepted_correspondences.push_back(std::move(correspondence));
  }

  result.diagnostics.accepted_residual_count = residuals.size();
  result.diagnostics.rejected_residual_count +=
      result.diagnostics.insufficient_neighbor_count + result.diagnostics.rejected_plane_count;
  result.measurement.rejected_residual_count = result.diagnostics.rejected_residual_count;

  const Eigen::Index row_count = static_cast<Eigen::Index>(jacobian_rows.size());
  result.measurement.jacobian.resize(row_count, ManifoldState::kErrorStateDimension);
  result.measurement.residual_m.resize(row_count);
  result.measurement.variance_m2.resize(row_count);
  for (Eigen::Index row = 0; row < row_count; ++row) {
    result.measurement.jacobian.row(row) = jacobian_rows[static_cast<std::size_t>(row)];
    result.measurement.residual_m[row] = residuals[static_cast<std::size_t>(row)];
    result.measurement.variance_m2[row] = variances[static_cast<std::size_t>(row)];
  }
  result.diagnostics.residual_rms_m = result.measurement.residualRmsM();
  return result;
}

}  // namespace uav::nav::lio
