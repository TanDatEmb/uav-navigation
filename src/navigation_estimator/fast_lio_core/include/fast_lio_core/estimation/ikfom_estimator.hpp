#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/estimation/ikfom_state.hpp"
#include "fast_lio_core/estimation/imu_trajectory.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/mapping/registration_map.hpp"
#include "fast_lio_core/registration/residual_builder.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"

namespace uav::nav::lio {

struct IkfomEstimatorConfig {
  std::size_t maximum_iterations{4};
  std::size_t minimum_accepted_residuals{20};
  std::int64_t maximum_integration_step_ns{20'000'000};
  double convergence_limit{1e-3};
  double gyro_noise_standard_deviation{0.01};
  double accel_noise_standard_deviation{0.1};
  double gyro_bias_random_walk_standard_deviation{1e-4};
  double accel_bias_random_walk_standard_deviation{1e-4};
  double initial_covariance{1e-3};
  bool estimate_extrinsic{false};
};

struct IkfomCorrectionResult {
  bool successful{false};
  bool finite{false};
  bool converged{false};
  std::string reason;
  ManifoldState predicted_state;
  ManifoldState corrected_state;
  ManifoldState::Covariance corrected_covariance{
      ManifoldState::Covariance::Identity()};
  ResidualBuildResult residual_build;
  std::size_t iteration_count{0};
  double final_increment_norm{0.0};
  double correction_translation_norm_m{0.0};
  double correction_rotation_norm_rad{0.0};
  std::size_t measurement_callback_count{0};
  std::int64_t measurement_model_runtime_us{0};
  std::int64_t solver_only_runtime_us{0};
  std::int64_t ikfom_total_runtime_us{0};
  std::int64_t residual_build_runtime_us{0};
  std::int64_t ikfom_update_runtime_us{0};
};

class IkfomEstimator {
 public:
  IkfomEstimator(IkfomEstimatorConfig config,
                 ResidualBuilderConfig residual_config);

  void initialize(const ManifoldState& state);
  void reset(const ManifoldState& state);
  void rebase(const ManifoldState& state,
              const ManifoldState::Covariance& covariance);

  [[nodiscard]] Result<ImuTrajectory> predict(
      std::span<const ImuSample> samples, const Timestamp& start_time,
      const Timestamp& end_time);
  [[nodiscard]] IkfomCorrectionResult correct(
      std::span<const Eigen::Vector3d> points_lidar_m,
      const RegistrationMap& map);

  [[nodiscard]] ManifoldState stateView() const;
  [[nodiscard]] ManifoldState::Covariance covariance() const;
  [[nodiscard]] const IkfomFilter& upstreamFilter() const noexcept;

 private:
  static Eigen::VectorXd measurementModel(
      IkfomState& state, esekfom::dyn_share_datastruct<double>& data);
  [[nodiscard]] Eigen::VectorXd buildMeasurement(
      IkfomState& state, esekfom::dyn_share_datastruct<double>& data);
  [[nodiscard]] IkfomFilter::processnoisecovariance processNoise() const;

  IkfomEstimatorConfig config_;
  ResidualBuilder residual_builder_;
  IkfomFilter filter_;
  std::span<const Eigen::Vector3d> active_points_;
  const RegistrationMap* active_map_{nullptr};
  ResidualBuildResult last_residual_build_;
  ResidualBuildDiagnostics last_residual_diagnostics_;
  std::size_t measurement_call_count_{0};
  std::int64_t active_residual_runtime_us_{0};
  Eigen::Quaterniond fixed_rotation_imu_lidar_{
      Eigen::Quaterniond::Identity()};
  Eigen::Vector3d fixed_position_imu_lidar_m_{Eigen::Vector3d::Zero()};

  static thread_local IkfomEstimator* active_estimator_;
};

}  // namespace uav::nav::lio
