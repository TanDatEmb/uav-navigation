#include "fast_lio_core/estimation/ikfom_estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <Eigen/Eigenvalues>

namespace uav::nav::lio {
namespace {

[[nodiscard]] bool covarianceValid(const IkfomFilter::cov& covariance) {
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8) {
    return false;
  }
  const IkfomFilter::cov symmetric =
      0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<IkfomFilter::cov> solver(
      symmetric, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1e-10;
}

[[nodiscard]] bool covarianceSaneDuringPrediction(
    const IkfomFilter::cov& covariance) {
  if (!covariance.allFinite()) {
    return false;
  }
  const double symmetry_error =
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
  return std::isfinite(symmetry_error) && symmetry_error <= 1e-8 &&
         covariance.diagonal().minCoeff() > -1e-10;
}

[[nodiscard]] Result<ImuSample> interpolateSample(
    std::span<const ImuSample> samples, const Timestamp& time) {
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), time.nanoseconds(),
      [](const ImuSample& sample, std::int64_t nanoseconds) {
        return sample.time.nanoseconds() < nanoseconds;
      });
  if (upper == samples.end()) {
    return Status(StatusCode::kInsufficientData,
                  "No IMU sample at or after IKFoM prediction boundary");
  }
  if (upper->time.nanoseconds() == time.nanoseconds()) {
    return *upper;
  }
  if (upper == samples.begin()) {
    return Status(StatusCode::kInsufficientData,
                  "No IMU sample at or before IKFoM prediction boundary");
  }
  const auto lower = std::prev(upper);
  const std::int64_t interval_ns =
      upper->time.nanoseconds() - lower->time.nanoseconds();
  if (interval_ns <= 0) {
    return Status(StatusCode::kTimestampRegression,
                  "IMU timestamps must be strictly increasing");
  }
  const double alpha =
      static_cast<double>(time.nanoseconds() -
                          lower->time.nanoseconds()) /
      static_cast<double>(interval_ns);
  return ImuSample{
      time,
      (1.0 - alpha) * lower->angular_velocity_imu_rad_s +
          alpha * upper->angular_velocity_imu_rad_s,
      (1.0 - alpha) * lower->linear_acceleration_imu_m_s2 +
          alpha * upper->linear_acceleration_imu_m_s2};
}

[[nodiscard]] IkfomInput toIkfomInput(const ImuSample& previous,
                                      const ImuSample& current) {
  IkfomInput input;
  input.gyro =
      0.5 * (previous.angular_velocity_imu_rad_s +
             current.angular_velocity_imu_rad_s);
  input.acc =
      0.5 * (previous.linear_acceleration_imu_m_s2 +
             current.linear_acceleration_imu_m_s2);
  return input;
}

[[nodiscard]] ImuTrajectoryState trajectoryState(
    const ManifoldState& state, const Timestamp& time) {
  return ImuTrajectoryState{time, state.orientation_odom_imu(),
                            state.position_odom_imu_m(),
                            state.velocity_odom_imu_m_s()};
}

[[nodiscard]] IkfomState toIkfomState(const ManifoldState& state) {
  IkfomState output;
  output.pos = state.position_odom_imu_m();
  output.rot = IkfomSo3{state.orientation_odom_imu()};
  output.offset_R_L_I = IkfomSo3{state.rotation_imu_lidar()};
  output.offset_T_L_I = state.position_imu_lidar_m();
  output.vel = state.velocity_odom_imu_m_s();
  output.bg = state.gyro_bias_rad_s();
  output.ba = state.accel_bias_m_s2();
  output.grav = IkfomGravity{IkfomVector3{state.gravity_odom_m_s2()}};
  return output;
}

[[nodiscard]] ManifoldState fromIkfomState(const IkfomState& state) {
  ManifoldState output;
  output.set_position_odom_imu_m(state.pos);
  output.set_orientation_odom_imu(
      Eigen::Quaterniond{state.rot.w(), state.rot.x(), state.rot.y(),
                         state.rot.z()});
  output.set_rotation_imu_lidar(
      Eigen::Quaterniond{state.offset_R_L_I.w(),
                         state.offset_R_L_I.x(),
                         state.offset_R_L_I.y(),
                         state.offset_R_L_I.z()});
  output.set_position_imu_lidar_m(state.offset_T_L_I);
  output.set_velocity_odom_imu_m_s(state.vel);
  output.set_gyro_bias_rad_s(state.bg);
  output.set_accel_bias_m_s2(state.ba);
  output.set_gravity_odom_m_s2(state.grav.get_vect());
  output.normalize();
  return output;
}

}  // namespace

thread_local IkfomEstimator* IkfomEstimator::active_estimator_ = nullptr;

IkfomEstimator::IkfomEstimator(IkfomEstimatorConfig config,
                               ResidualBuilderConfig residual_config)
    : config_(config), residual_builder_(residual_config) {
  if (config_.maximum_iterations == 0U ||
      config_.minimum_accepted_residuals == 0U ||
      config_.maximum_integration_step_ns <= 0 ||
      !(config_.convergence_limit > 0.0) ||
      !(config_.initial_covariance > 0.0)) {
    throw std::invalid_argument("invalid IKFoM runtime configuration");
  }
  std::array<double, IkfomState::DOF> limits{};
  limits.fill(config_.convergence_limit);
  filter_.init_dyn_share(ikfomProcessModel, ikfomProcessJacobianState,
                         ikfomProcessJacobianNoise, measurementModel,
                         static_cast<int>(config_.maximum_iterations),
                         limits.data());
}

void IkfomEstimator::initialize(const ManifoldState& state) {
  fixed_rotation_imu_lidar_ = state.rotation_imu_lidar();
  fixed_position_imu_lidar_m_ = state.position_imu_lidar_m();
  IkfomState upstream_state = toIkfomState(state);
  filter_.change_x(upstream_state);
  IkfomFilter::cov covariance =
      config_.initial_covariance * IkfomFilter::cov::Identity();
  covariance.block<3, 3>(6, 6) =
      1e-12 * Eigen::Matrix3d::Identity();
  covariance.block<3, 3>(9, 9) =
      1e-12 * Eigen::Matrix3d::Identity();
  filter_.change_P(covariance);
}

void IkfomEstimator::reset(const ManifoldState& state) { initialize(state); }

void IkfomEstimator::rebase(
    const ManifoldState& state,
    const ManifoldState::Covariance& covariance) {
  IkfomState upstream_state = toIkfomState(state);
  IkfomFilter::cov upstream_covariance = covariance;
  filter_.change_x(upstream_state);
  filter_.change_P(upstream_covariance);
}

Result<ImuTrajectory> IkfomEstimator::predict(
    std::span<const ImuSample> samples, const Timestamp& start_time,
    const Timestamp& end_time) {
  const auto duration = checkedDifference(end_time, start_time);
  if (!duration.ok()) {
    return duration.status();
  }
  if (duration.value().nanoseconds() < 0) {
    return Status(StatusCode::kTimestampRegression,
                  "IKFoM prediction end precedes start");
  }
  if (samples.empty()) {
    return Status(StatusCode::kInsufficientData,
                  "No IMU samples supplied to IKFoM");
  }
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const Status validation = samples[index].validate();
    if (!validation.ok()) {
      return validation;
    }
    if (!samples[index].time.sameClockDomain(start_time)) {
      return Status(StatusCode::kClockDomainMismatch,
                    "IMU and IKFoM prediction use different clocks");
    }
    if (index > 0 &&
        samples[index].time.nanoseconds() <=
            samples[index - 1].time.nanoseconds()) {
      return Status(StatusCode::kTimestampRegression,
                    "IMU timestamps must be strictly increasing");
    }
  }

  const auto start_sample = interpolateSample(samples, start_time);
  const auto end_sample = interpolateSample(samples, end_time);
  if (!start_sample.ok()) {
    return start_sample.status();
  }
  if (!end_sample.ok()) {
    return end_sample.status();
  }

  std::vector<ImuSample> integration_samples;
  integration_samples.reserve(samples.size() + 2U);
  integration_samples.push_back(start_sample.value());
  for (const auto& sample : samples) {
    if (sample.time.nanoseconds() > start_time.nanoseconds() &&
        sample.time.nanoseconds() < end_time.nanoseconds()) {
      integration_samples.push_back(sample);
    }
  }
  if (end_time.nanoseconds() > start_time.nanoseconds()) {
    integration_samples.push_back(end_sample.value());
  }

  for (std::size_t index = 1; index < integration_samples.size(); ++index) {
    const std::int64_t dt_ns =
        integration_samples[index].time.nanoseconds() -
        integration_samples[index - 1].time.nanoseconds();
    if (dt_ns <= 0 || dt_ns > config_.maximum_integration_step_ns) {
      return Status(StatusCode::kInsufficientData,
                    "IKFoM integration step is zero, negative or too large");
    }
    const IkfomInput input =
        toIkfomInput(integration_samples[index - 1],
                     integration_samples[index]);
    if (!input.gyro.allFinite() || !input.acc.allFinite()) {
      return Status(StatusCode::kInvalidArgument,
                    "IKFoM integration input is non-finite");
    }
  }

  ImuTrajectory trajectory;
  Status trajectory_status =
      trajectory.addState(trajectoryState(stateView(), start_time));
  if (!trajectory_status.ok()) {
    return trajectory_status;
  }
  IkfomState state_before_prediction = filter_.get_x();
  IkfomFilter::cov covariance_before_prediction = filter_.get_P();
  bool filter_mutated = false;
  const auto rollback = [&]() {
    if (filter_mutated) {
      filter_.change_x(state_before_prediction);
      filter_.change_P(covariance_before_prediction);
    }
  };
  auto noise = processNoise();
  for (std::size_t index = 1; index < integration_samples.size(); ++index) {
    const auto& previous = integration_samples[index - 1];
    const auto& current = integration_samples[index];
    const std::int64_t dt_ns =
        current.time.nanoseconds() - previous.time.nanoseconds();
    double dt_seconds = static_cast<double>(dt_ns) * 1e-9;
    filter_.predict(dt_seconds, noise, toIkfomInput(previous, current));
    filter_mutated = true;
    const ManifoldState predicted = stateView();
    if (!predicted.allFinite() ||
        !covarianceSaneDuringPrediction(filter_.get_P())) {
      rollback();
      return Status(StatusCode::kNumericalFailure,
                    "IKFoM prediction produced non-finite state");
    }
    trajectory_status =
        trajectory.addState(trajectoryState(predicted, current.time));
    if (!trajectory_status.ok()) {
      rollback();
      return trajectory_status;
    }
  }
  return trajectory;
}

IkfomCorrectionResult IkfomEstimator::correct(
    std::span<const Eigen::Vector3d> points_lidar_m,
    const RegistrationMap& map) {
  IkfomCorrectionResult result;
  result.predicted_state = stateView();
  IkfomState predicted_upstream_state = filter_.get_x();
  IkfomFilter::cov predicted_covariance = filter_.get_P();
  active_points_ = points_lidar_m;
  active_map_ = &map;
  measurement_call_count_ = 0U;
  active_nearest_search_query_count_ = 0U;
  active_residual_runtime_us_ = 0;
  active_nearest_search_runtime_us_ = 0;
  active_plane_and_gate_runtime_us_ = 0;
  active_jacobian_build_runtime_us_ = 0;
  last_residual_diagnostics_ = {};

  struct ActiveGuard {
    explicit ActiveGuard(IkfomEstimator* estimator) {
      if (IkfomEstimator::active_estimator_ != nullptr) {
        throw std::logic_error("IKFoM measurement callback is not reentrant");
      }
      IkfomEstimator::active_estimator_ = estimator;
    }
    ~ActiveGuard() { IkfomEstimator::active_estimator_ = nullptr; }
  } guard{this};

  const auto update_started = std::chrono::steady_clock::now();
  const auto update = filter_.update_iterated_dyn_share();
  result.ikfom_update_runtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - update_started)
          .count();
  result.ikfom_total_runtime_us = result.ikfom_update_runtime_us;
  result.measurement_model_runtime_us = active_residual_runtime_us_;
  result.residual_build_runtime_us = result.measurement_model_runtime_us;
  result.solver_only_runtime_us = std::max<std::int64_t>(
      0, result.ikfom_total_runtime_us - result.measurement_model_runtime_us);
  result.measurement_callback_count = measurement_call_count_;
  result.nearest_search_query_count = active_nearest_search_query_count_;
  result.nearest_search_runtime_us = active_nearest_search_runtime_us_;
  result.plane_and_gate_runtime_us = active_plane_and_gate_runtime_us_;
  result.jacobian_build_runtime_us = active_jacobian_build_runtime_us_;
  IkfomState fixed_state = filter_.get_x();
  fixed_state.offset_R_L_I = IkfomSo3{fixed_rotation_imu_lidar_};
  fixed_state.offset_T_L_I = fixed_position_imu_lidar_m_;
  filter_.change_x(fixed_state);
  result.corrected_state = stateView();
  result.corrected_covariance = covariance();
  result.residual_diagnostics = last_residual_diagnostics_;
  result.residual_diagnostics.query_count =
      result.nearest_search_query_count;
  result.iteration_count =
      static_cast<std::size_t>(std::max(update.iteration_count, 0));
  result.final_increment_norm = update.final_increment_norm;
  result.finite = result.corrected_state.allFinite() &&
                  covarianceValid(result.corrected_covariance);
  const bool measurement_usable =
      update.measurement_valid && !update.numerical_failure &&
      result.residual_diagnostics.accepted_residual_count >=
          config_.minimum_accepted_residuals;
  result.converged = measurement_usable && update.converged;
  // FAST-LIO applies the finite terminal IKFoM iterate when the configured
  // iteration budget is exhausted. Convergence remains a diagnostic; it must
  // not roll back an otherwise usable correction or prevent the registration
  // map from following the accepted state.
  result.successful = result.finite && measurement_usable;
  result.reason =
      result.successful
          ? (result.converged
                 ? "IKFOM_LIDAR_UPDATE_CONVERGED"
                 : "IKFOM_LIDAR_UPDATE_ACCEPTED_AT_ITERATION_LIMIT")
          : (update.numerical_failure
                 ? "IKFOM_LIDAR_UPDATE_NUMERICAL_FAILURE"
                 : (result.finite ? "IKFOM_LIDAR_UPDATE_NOT_CONVERGED"
                                  : "IKFOM_LIDAR_UPDATE_NON_FINITE"));
  result.correction_translation_norm_m =
      (result.corrected_state.position_odom_imu_m() -
       result.predicted_state.position_odom_imu_m())
          .norm();
  result.correction_rotation_norm_rad =
      result.predicted_state.orientation_odom_imu().angularDistance(
          result.corrected_state.orientation_odom_imu());
  if (!result.successful) {
    // A rejected LiDAR update must not become the prior for the next IMU
    // prediction. IKFoM updates in place, so restore the transactional prior.
    filter_.change_x(predicted_upstream_state);
    filter_.change_P(predicted_covariance);
  }
  active_points_ = {};
  active_map_ = nullptr;
  return result;
}

ManifoldState IkfomEstimator::stateView() const {
  return fromIkfomState(filter_.get_x());
}

ManifoldState::Covariance IkfomEstimator::covariance() const {
  return filter_.get_P();
}

const IkfomFilter& IkfomEstimator::upstreamFilter() const noexcept {
  return filter_;
}

Eigen::VectorXd IkfomEstimator::measurementModel(
    IkfomState& state, esekfom::dyn_share_datastruct<double>& data) {
  if (active_estimator_ == nullptr) {
    data.valid = false;
    return {};
  }
  return active_estimator_->buildMeasurement(state, data);
}

Eigen::VectorXd IkfomEstimator::buildMeasurement(
    IkfomState& state, esekfom::dyn_share_datastruct<double>& data) {
  ++measurement_call_count_;
  if (active_map_ == nullptr) {
    data.valid = false;
    return {};
  }
  const ManifoldState view = fromIkfomState(state);
  const auto residual_started = std::chrono::steady_clock::now();
  const ResidualBuildView measurement_view =
      residual_builder_.buildInto(active_points_, view, *active_map_);
  last_residual_diagnostics_ = measurement_view.diagnostics;
  active_nearest_search_query_count_ +=
      measurement_view.diagnostics.query_count;
  active_nearest_search_runtime_us_ +=
      measurement_view.diagnostics.nearest_search_runtime_us;
  active_plane_and_gate_runtime_us_ +=
      measurement_view.diagnostics.plane_and_gate_runtime_us;
  active_jacobian_build_runtime_us_ +=
      measurement_view.diagnostics.jacobian_build_runtime_us;
  active_residual_runtime_us_ +=
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - residual_started)
          .count();
  if (!measurement_view.valid() ||
      measurement_view.row_count < config_.minimum_accepted_residuals) {
    data.valid = false;
    return {};
  }
  const Eigen::Index rows = static_cast<Eigen::Index>(measurement_view.row_count);
  data.valid = true;
  data.h_x = measurement_view.jacobian->topRows(rows);
  if (rows >= IkfomState::DOF) {
    // This production measurement has independent scalar noise. Store only
    // its diagonal; the patched IKFoM normal-equation branch consumes this
    // compact representation without allocating two rows-by-rows matrices.
    data.h_v.resize(0, 0);
    data.R = measurement_view.variance_m2->head(rows);
  } else {
    data.h_v = Eigen::MatrixXd::Identity(rows, rows);
    data.R = measurement_view.variance_m2->head(rows).asDiagonal();
  }
  data.z = Eigen::VectorXd::Zero(rows);
  return measurement_view.residual_m->head(rows);
}

IkfomFilter::processnoisecovariance IkfomEstimator::processNoise() const {
  IkfomFilter::processnoisecovariance covariance =
      IkfomFilter::processnoisecovariance::Zero();
  covariance.block<3, 3>(0, 0).diagonal().setConstant(
      config_.gyro_noise_standard_deviation *
      config_.gyro_noise_standard_deviation);
  covariance.block<3, 3>(3, 3).diagonal().setConstant(
      config_.accel_noise_standard_deviation *
      config_.accel_noise_standard_deviation);
  covariance.block<3, 3>(6, 6).diagonal().setConstant(
      config_.gyro_bias_random_walk_standard_deviation *
      config_.gyro_bias_random_walk_standard_deviation);
  covariance.block<3, 3>(9, 9).diagonal().setConstant(
      config_.accel_bias_random_walk_standard_deviation *
      config_.accel_bias_random_walk_standard_deviation);
  return covariance;
}

}  // namespace uav::nav::lio
