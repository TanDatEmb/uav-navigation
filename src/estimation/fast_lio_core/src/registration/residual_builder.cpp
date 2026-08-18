#include "fast_lio_core/registration/residual_builder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace uav::nav::lio {
namespace {

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

}  // namespace

void ResidualWorkspace::ensureCapacity(const std::size_t point_count) {
  if (candidates.capacity() < point_count) {
    candidates.reserve(point_count);
  }
  candidates.resize(point_count);
  if (H.rows() < static_cast<Eigen::Index>(point_count) ||
      H.cols() != ManifoldState::kErrorStateDimension) {
    H.resize(static_cast<Eigen::Index>(point_count),
             ManifoldState::kErrorStateDimension);
  }
  if (residual.size() < static_cast<Eigen::Index>(point_count)) {
    residual.resize(static_cast<Eigen::Index>(point_count));
  }
  if (variance.size() < static_cast<Eigen::Index>(point_count)) {
    variance.resize(static_cast<Eigen::Index>(point_count));
  }
}

ResidualBuilder::ResidualBuilder(ResidualBuilderConfig config)
    : config_(config),
      plane_estimator_(config_.plane_estimator),
      residual_gate_(config_.residual_gate) {
  if (!(config_.correspondence_search.maximum_neighbor_distance_m > 0.0) ||
      !std::isfinite(config_.correspondence_search.maximum_neighbor_distance_m) ||
      !(config_.point_measurement_standard_deviation_m > 0.0) ||
      !std::isfinite(config_.point_measurement_standard_deviation_m) ||
      config_.parallel_thread_count == 0U ||
      config_.parallel_thread_count > 32U) {
    throw std::invalid_argument("invalid residual builder configuration");
  }
}

ResidualBuildView ResidualBuilder::buildInto(
    std::span<const Eigen::Vector3d> points_lidar_m,
    const ManifoldState& state, const RegistrationMap& map) {
  ResidualBuildDiagnostics diagnostics;
  diagnostics.input_point_count = points_lidar_m.size();
  workspace_.ensureCapacity(points_lidar_m.size());
  last_row_count_ = 0U;

  if (!state.allFinite() || points_lidar_m.empty()) {
    last_diagnostics_ = diagnostics;
    return {&workspace_.H, &workspace_.residual, &workspace_.variance, 0U,
            diagnostics};
  }

  const Eigen::Matrix3d rotation_odom_imu =
      state.orientation_odom_imu().toRotationMatrix();
  const Eigen::Matrix3d rotation_imu_lidar =
      state.rotation_imu_lidar().toRotationMatrix();
  const std::int64_t point_count =
      static_cast<std::int64_t>(points_lidar_m.size());
#pragma omp parallel for schedule(static) \
    num_threads(config_.parallel_thread_count) if (point_count >= 256)
  for (std::int64_t signed_index = 0; signed_index < point_count;
       ++signed_index) {
    const std::size_t index = static_cast<std::size_t>(signed_index);
    const Eigen::Vector3d& point_lidar_m = points_lidar_m[index];
    ResidualWorkspace::Candidate& candidate = workspace_.candidates[index];
    candidate.outcome =
        ResidualWorkspace::CandidateOutcome::kInsufficientNeighbors;
    candidate.neighbors_valid = false;
    candidate.point_odom_m =
        state.transformLidarPointToOdom(point_lidar_m);
    const Eigen::Vector3d& point_odom_m = candidate.point_odom_m;
    if (!point_lidar_m.allFinite() || !point_odom_m.allFinite()) {
      continue;
    }
  }

  std::size_t query_count = 0U;
  const auto nearest_search_started = std::chrono::steady_clock::now();
#pragma omp parallel for schedule(static) \
    num_threads(config_.parallel_thread_count) if (point_count >= 256) \
    reduction(+ : query_count)
  for (std::int64_t signed_index = 0; signed_index < point_count;
       ++signed_index) {
    const std::size_t index = static_cast<std::size_t>(signed_index);
    ResidualWorkspace::Candidate& candidate = workspace_.candidates[index];
    if (!candidate.point_odom_m.allFinite()) {
      continue;
    }
    ++query_count;
    candidate.neighbors_valid =
        map.nearestSearch(
            candidate.point_odom_m,
            config_.correspondence_search.maximum_neighbor_distance_m,
            candidate.neighbors) &&
        candidate.neighbors.complete();
  }
  diagnostics.nearest_search_runtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - nearest_search_started)
          .count();
  diagnostics.query_count = query_count;

  const auto plane_and_gate_started = std::chrono::steady_clock::now();
#pragma omp parallel for schedule(static) \
    num_threads(config_.parallel_thread_count) if (point_count >= 256)
  for (std::int64_t signed_index = 0; signed_index < point_count;
       ++signed_index) {
    const std::size_t index = static_cast<std::size_t>(signed_index);
    ResidualWorkspace::Candidate& candidate = workspace_.candidates[index];
    if (!candidate.neighbors_valid ||
        !candidate.point_odom_m.allFinite()) {
      candidate.outcome =
          ResidualWorkspace::CandidateOutcome::kInsufficientNeighbors;
      continue;
    }
    const std::span<const Eigen::Vector3d> neighbor_span(
        candidate.neighbors.points.data(), NeighborSet::kCapacity);
    const std::optional<Plane> plane = plane_estimator_.estimate(neighbor_span);
    if (!plane.has_value()) {
      candidate.outcome = ResidualWorkspace::CandidateOutcome::kRejectedPlane;
      continue;
    }
    candidate.signed_distance_m =
        plane->normal_odom.dot(candidate.point_odom_m - plane->centroid_odom_m);
    const ResidualGateDecision decision =
        residual_gate_.evaluate(*plane, candidate.signed_distance_m);
    if (!decision.accepted) {
      candidate.outcome =
          ResidualWorkspace::CandidateOutcome::kRejectedResidual;
      continue;
    }
    candidate.normal_odom = plane->normal_odom;
    candidate.plane_variance_m2 =
        plane->rms_error_m * plane->rms_error_m;
    candidate.robust_weight = decision.robust_weight;
    candidate.outcome = ResidualWorkspace::CandidateOutcome::kAccepted;
  }
  diagnostics.plane_and_gate_runtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - plane_and_gate_started)
          .count();

  const auto jacobian_build_started = std::chrono::steady_clock::now();
  const double sensor_variance =
      config_.point_measurement_standard_deviation_m *
      config_.point_measurement_standard_deviation_m;
  for (std::size_t index = 0U; index < points_lidar_m.size(); ++index) {
    const ResidualWorkspace::Candidate& candidate =
        workspace_.candidates[index];
    if (candidate.outcome ==
        ResidualWorkspace::CandidateOutcome::kInsufficientNeighbors) {
      ++diagnostics.insufficient_neighbor_count;
      continue;
    }
    if (candidate.outcome ==
        ResidualWorkspace::CandidateOutcome::kRejectedPlane) {
      ++diagnostics.rejected_plane_count;
      continue;
    }
    ++diagnostics.valid_plane_count;
    if (candidate.outcome ==
        ResidualWorkspace::CandidateOutcome::kRejectedResidual) {
      ++diagnostics.rejected_residual_count;
      continue;
    }

    const Eigen::Vector3d& point_lidar_m = points_lidar_m[index];
    const Eigen::Vector3d point_imu_m =
        rotation_imu_lidar * point_lidar_m + state.position_imu_lidar_m();
    auto jacobian = workspace_.H.row(static_cast<Eigen::Index>(last_row_count_));
    jacobian.setZero();
    jacobian.segment<3>(ManifoldState::kPositionOffset) =
        candidate.normal_odom.transpose();
    jacobian.segment<3>(ManifoldState::kOrientationOffset) =
        candidate.normal_odom.transpose() *
        (-rotation_odom_imu * skewSymmetric(point_imu_m));
    workspace_.residual[static_cast<Eigen::Index>(last_row_count_)] =
        candidate.signed_distance_m;
    workspace_.variance[static_cast<Eigen::Index>(last_row_count_)] =
        (sensor_variance + candidate.plane_variance_m2) /
        std::max(candidate.robust_weight, 1e-6);
    ++last_row_count_;
  }

  diagnostics.accepted_residual_count = last_row_count_;
  diagnostics.rejected_residual_count +=
      diagnostics.insufficient_neighbor_count + diagnostics.rejected_plane_count;
  if (last_row_count_ > 0U) {
    diagnostics.residual_rms_m =
        std::sqrt(workspace_.residual
                      .head(static_cast<Eigen::Index>(last_row_count_))
                      .squaredNorm() /
                  static_cast<double>(last_row_count_));
  }
  diagnostics.jacobian_build_runtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - jacobian_build_started)
          .count();
  last_diagnostics_ = diagnostics;
  return {&workspace_.H, &workspace_.residual, &workspace_.variance,
          last_row_count_, diagnostics};
}

ResidualBuildResult ResidualBuilder::snapshotLast() const {
  ResidualBuildResult result;
  result.diagnostics = last_diagnostics_;
  const Eigen::Index rows = static_cast<Eigen::Index>(last_row_count_);
  result.measurement.jacobian = workspace_.H.topRows(rows);
  result.measurement.residual_m = workspace_.residual.head(rows);
  result.measurement.variance_m2 = workspace_.variance.head(rows);
  result.measurement.rejected_residual_count =
      last_diagnostics_.rejected_residual_count;
  return result;
}

ResidualBuildResult ResidualBuilder::build(
    std::span<const Eigen::Vector3d> points_lidar_m,
    const ManifoldState& state, const RegistrationMap& map) {
  static_cast<void>(buildInto(points_lidar_m, state, map));
  return snapshotLast();
}

}  // namespace uav::nav::lio
