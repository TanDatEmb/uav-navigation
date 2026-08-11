#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <span>
#include <vector>

#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/mapping/registration_map.hpp"
#include "fast_lio_core/registration/linearized_measurement.hpp"
#include "fast_lio_core/registration/plane_estimator.hpp"
#include "fast_lio_core/registration/residual_gate.hpp"

namespace uav::nav::lio {

struct ResidualBuilderConfig {
  // Kept as a configuration grouping for YAML/API compatibility. The search
  // abstraction itself is intentionally absent from the hot path.
  struct SearchConfig {
    std::size_t neighbor_count{5};
    double maximum_neighbor_distance_m{2.0};
  } correspondence_search{};
  PlaneEstimatorConfig plane_estimator{};
  ResidualGateConfig residual_gate{};
  double point_measurement_standard_deviation_m{0.03};
  bool estimate_extrinsic{false};
};

struct ResidualBuildDiagnostics {
  std::size_t input_point_count{0};
  std::size_t query_count{0};
  std::size_t insufficient_neighbor_count{0};
  std::size_t valid_plane_count{0};
  std::size_t rejected_plane_count{0};
  std::size_t accepted_residual_count{0};
  std::size_t rejected_residual_count{0};
  double residual_rms_m{0.0};
};

struct ResidualBuildResult {
  LinearizedMeasurement measurement;
  ResidualBuildDiagnostics diagnostics;
};

struct ResidualBuildView {
  const Eigen::MatrixXd* jacobian{nullptr};
  const Eigen::VectorXd* residual_m{nullptr};
  const Eigen::VectorXd* variance_m2{nullptr};
  std::size_t row_count{0};
  ResidualBuildDiagnostics diagnostics;

  [[nodiscard]] bool valid() const noexcept {
    return jacobian != nullptr && residual_m != nullptr &&
           variance_m2 != nullptr && row_count > 0U;
  }
};

// Persistent numerical storage used by every IKFoM measurement callback.
// Capacity grows only when a larger scan arrives and is then reused.
class ResidualWorkspace {
 public:
  std::vector<Eigen::Vector3d> points_odom;
  Eigen::MatrixXd H;
  Eigen::VectorXd residual;
  Eigen::VectorXd variance;

  void ensureCapacity(std::size_t point_count);
};

class ResidualBuilder {
 public:
  explicit ResidualBuilder(ResidualBuilderConfig config = {});

  [[nodiscard]] ResidualBuildResult build(
      std::span<const Eigen::Vector3d> points_lidar_m,
      const ManifoldState& state, const RegistrationMap& map);

  [[nodiscard]] ResidualBuildView buildInto(
      std::span<const Eigen::Vector3d> points_lidar_m,
      const ManifoldState& state, const RegistrationMap& map);

  [[nodiscard]] ResidualBuildResult snapshotLast() const;

 private:
  ResidualBuilderConfig config_;
  PlaneEstimator plane_estimator_;
  ResidualGate residual_gate_;
  ResidualWorkspace workspace_;
  ResidualBuildDiagnostics last_diagnostics_{};
  std::size_t last_row_count_{0};
};

}  // namespace uav::nav::lio
