#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <span>
#include <vector>

#include "fast_lio_core/estimation/iterated_kalman_filter.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/mapping/registration_map.hpp"
#include "fast_lio_core/registration/correspondence.hpp"
#include "fast_lio_core/registration/correspondence_search.hpp"
#include "fast_lio_core/registration/plane_estimator.hpp"
#include "fast_lio_core/registration/residual_gate.hpp"

namespace uav::nav::lio {

struct ResidualBuilderConfig {
  CorrespondenceSearchConfig correspondence_search{};
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
  std::vector<Correspondence> accepted_correspondences;
  ResidualBuildDiagnostics diagnostics;
};

class ResidualBuilder {
 public:
  explicit ResidualBuilder(ResidualBuilderConfig config = {});

  [[nodiscard]] ResidualBuildResult build(std::span<const Eigen::Vector3d> points_lidar_m,
                                          const ManifoldState& state,
                                          const RegistrationMap& map) const;

 private:
  ResidualBuilderConfig config_;
  CorrespondenceSearch correspondence_search_;
  PlaneEstimator plane_estimator_;
  ResidualGate residual_gate_;
};

}  // namespace uav::nav::lio
