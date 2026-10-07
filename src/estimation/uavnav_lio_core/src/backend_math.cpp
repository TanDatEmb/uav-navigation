#include "backend_math.hpp"

#include <Eigen/Eigenvalues>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "fast_lio_core/initialization/initial_state_prior.hpp"
#include "fast_lio_core/navigation/kinematic_state_estimate.hpp"
#include "uavnav/lio/limits.hpp"

namespace uavnav::lio::backend_math {

namespace {

/// imu_T_lidar's translation and the config's extrinsic_imu_lidar_*_m must agree to this (one source).
constexpr double kExtrinsicToleranceM = 1e-6;
/// In-flight restart: the seed attitude is taken as is (§3.4: T^-1 x PX4 pose); this tilt limit is not
/// used by InitialStatePriorApplicator for the in-flight context.
constexpr double kUnusedTiltLimitRad = 0.0;

}  // namespace

EstimatorSnapshot snapshot(const fl::ManifoldState& s, time::SensorTime t) {
  return EstimatorSnapshot{t,
                           s.orientation_odom_imu(),
                           s.velocity_odom_imu_m_s(),
                           s.position_odom_imu_m(),
                           s.gyro_bias_rad_s(),
                           s.accel_bias_m_s2(),
                           s.gravity_odom_m_s2()};
}

Result<void, config::ConfigError> check_frames(const LioConfig& config, const Eigen::Isometry3d& base_T_imu,
                                               const Eigen::Isometry3d& imu_T_lidar) {
  using Kind = config::ConfigError::Kind;
  if (!base_T_imu.matrix().allFinite()) {
    return std::unexpected(config::ConfigError{Kind::kNotFinite, "base_T_imu", "not finite"});
  }
  if (!imu_T_lidar.matrix().allFinite()) {
    return std::unexpected(config::ConfigError{Kind::kNotFinite, "imu_T_lidar", "not finite"});
  }
  constexpr std::array<std::string_view, 3> kKeys{"extrinsic_imu_lidar_x_m", "extrinsic_imu_lidar_y_m",
                                                  "extrinsic_imu_lidar_z_m"};
  for (int a = 0; a < 3; ++a) {
    if (std::abs(imu_T_lidar.translation()[a] - config.math.t_imu_lidar_m[a]) > kExtrinsicToleranceM) {
      const std::string key(kKeys[static_cast<std::size_t>(a)]);
      return std::unexpected(config::ConfigError{Kind::kOutOfRange, key,
                                                 key + " disagrees with imu_T_lidar (" +
                                                     std::to_string(config.math.t_imu_lidar_m[a]) + " vs " +
                                                     std::to_string(imu_T_lidar.translation()[a]) + ")"});
    }
  }
  return {};
}

fl::ManifoldState::Covariance inflated_for_rebase(const fl::ManifoldState::Covariance& p) {
  const fl::ManifoldState::Covariance sym = 0.5 * (p + p.transpose());
  Eigen::SelfAdjointEigenSolver<fl::ManifoldState::Covariance> solver(sym);
  fl::ManifoldState::Covariance inflated = limits::kRebaseCovarianceInflation * sym;
  if (solver.info() == Eigen::Success && solver.eigenvalues().allFinite()) {
    const auto values = (solver.eigenvalues().cwiseMax(0.0) * limits::kRebaseCovarianceInflation)
                            .cwiseMin(limits::kRebaseMaxCovarianceEigenvalue);
    inflated = solver.eigenvectors() * values.asDiagonal() * solver.eigenvectors().transpose();
    inflated = 0.5 * (inflated + inflated.transpose());
  }
  return inflated;
}

std::vector<fl::ImuSample> prediction_span(const std::deque<fl::ImuSample>& history, time::SensorTime start,
                                           time::SensorTime end) {
  std::vector<fl::ImuSample> span;
  for (std::size_t i = 0; i < history.size(); ++i) {
    const std::int64_t ti = history[i].time.nanoseconds();
    const bool start_bracket =
        ti <= start.ns && (i + 1 == history.size() || history[i + 1].time.nanoseconds() > start.ns);
    if (start_bracket || (!span.empty() && ti > start.ns)) span.push_back(history[i]);
    if (!span.empty() && ti >= end.ns) break;
  }
  if (span.empty() || span.front().time.nanoseconds() > start.ns || span.back().time.nanoseconds() < end.ns) {
    span.clear();
  }
  return span;
}

void drop_points_before(fl::LidarScan& scan, time::SensorTime scan_start, time::SensorTime eskf_t) {
  const std::int64_t shift = (eskf_t - scan_start).ns;
  std::erase_if(scan.points,
                [&](const fl::LidarPoint& p) { return static_cast<std::int64_t>(p.relative_time_ns) < shift; });
  // Every kept point has relative_time_ns >= shift, so the difference fits the uint32 field.
  for (auto& p : scan.points) p.relative_time_ns = static_cast<std::uint32_t>(p.relative_time_ns - shift);
  scan.start_time = stamp(eskf_t);
}

std::optional<BaseOdometry> base_odometry(const fl::BaseLinkStateConverter& converter,
                                          const fl::BaseLinkCovarianceProjector& projector,
                                          const fl::StateEstimate& est, const Eigen::Vector3d& omega) {
  const auto base = converter.convert(est, omega);
  if (!base.ok()) return std::nullopt;
  const auto cov = projector.project(fl::KinematicStateEstimate{est, omega}, base.value());
  if (!cov.ok()) return std::nullopt;
  const Eigen::Matrix3d r = base.value().orientation_reference_body.toRotationMatrix();
  BaseOdometry o{base.value().position_reference_body_m, base.value().orientation_reference_body,
                 base.value().linear_velocity_reference_body_m_s, cov.value().pose_covariance_odom,
                 r * cov.value().twist_covariance_base.topLeftCorner<3, 3>() * r.transpose()};
  if (!o.p_world_m.allFinite() || !o.q_world_base.coeffs().allFinite() || !o.v_world_mps.allFinite() ||
      !o.pose_cov.allFinite() || !o.vel_cov.allFinite()) {
    return std::nullopt;
  }
  return o;
}

std::optional<fl::ManifoldState> seed_state(const fl::InitialStatePriorApplicator& applicator,
                                            fl::ManifoldState carried, const RestartCommand& c,
                                            const Eigen::Quaterniond& q_base_imu) {
  const Eigen::Quaterniond q_seed = c.seed.q_world_base.normalized();
  const Eigen::Quaterniond q_new_imu = q_seed * q_base_imu;
  carried.set_velocity_odom_imu_m_s((q_new_imu * c.q_world_imu_old.normalized().conjugate()) * c.v_world_mps_old);

  fl::InitialStatePrior prior;
  prior.sample_time = stamp(c.t);
  prior.source = fl::InitialStatePriorSource::kTopic;
  prior.context = fl::InitialStatePriorContext::kInFlightReinitialization;
  prior.mask = fl::InitialStatePriorMask{true, false, fl::PriorAttitudeMode::kFull};
  prior.position_odom_base_m = c.seed.p_world_m;
  prior.orientation_odom_base = q_seed;
  prior.generation = c.new_epoch;
  prior.provenance = "restart";
  fl::ManifoldState seeded;
  if (!applicator.apply(prior, carried, kUnusedTiltLimitRad, seeded).ok()) return std::nullopt;
  return seeded;
}

}  // namespace uavnav::lio::backend_math
