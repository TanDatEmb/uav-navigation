#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>

#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/lio/degeneracy.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"

// Typed tier-(b) configuration of the LIO core (SYSTEM_DESIGN §6.2). Later tasks append
// their keys to kLioSpecs (N is derived from the initializer) and their section to
// LioConfig / load_lio_config.
namespace uavnav::lio {

/// Values handed to the reused FAST-LIO math (fast_lio_core) by LioEstimator. Beta values (from the survey
/// of `main`, config/runtime/sim.yaml) in parentheses. Every other fast_lio_core knob keeps the default of
/// its own config struct (single source: fast_lio_core), see estimator.cpp.
struct MathConfig {
  Eigen::Vector3d t_imu_lidar_m;         ///< IMU -> LiDAR translation, IMU frame ((-0.011, -0.02329, 0.04412))
  double preprocess_min_range_m;         ///< points closer than this are dropped (0.5)
  double preprocess_max_range_m;         ///< points farther than this are dropped (40)
  double preprocess_voxel_m;             ///< scan voxel filter edge (0.2)
  double map_voxel_m;                    ///< ikd-tree downsample voxel edge (0.3)
  Eigen::Vector3d map_half_extent_m;     ///< local map half extents x, y, z ((30, 30, 15))
  std::uint32_t registration_max_iterations;  ///< IKFoM iterations per scan (4)
  std::uint32_t imu_init_min_samples;    ///< stationary samples for gravity + bias init (200)
  time::Duration imu_max_gap;            ///< largest IMU step integrated / reported as a gap (0.02 s)
};

struct LioConfig {
  LifecycleConfig lifecycle;
  DegeneracyConfig degeneracy;
  PredictorConfig predictor;
  MathConfig math;
};

/// The single definition of every LIO key, its unit and its inclusive bounds. Beta values:
/// confirm 5, degenerate 3, gap_degraded 0.25 s, gap_lost 0.5 s, degeneracy_lost 1.0 s, sigma_lost 0.5 m,
/// degeneracy_translation_min_info 1.1e5, degeneracy_rotation_min_info 2.8e6,
/// predictor_tau_vel_s 0.25 s, predictor_tau_pos_s 0.25 s (SYSTEM_DESIGN §3.3: τ_vel = τ_pos = 0.25 s).
///
/// Effective predictor time constant: the vel/pos gain is dt_corr_avg / τ, and (as in PX4) the averaged
/// correction interval is clamped to kPredictorDtMax = 0.03 s. At scan period T > 0.03 s the per-scan step
/// is therefore ≈ 0.03 / τ · err, an effective time constant of about τ · T / 0.03 (≈ 0.83 s for τ = 0.25 s
/// at 10 Hz), not τ. This keeps each correction step small (no visible jump). S1b chooses from SITL logs
/// between this clamp and the literal τ.
///
/// Derivation of the two degeneracy thresholds (smallest eigenvalue of the translation / rotation block of
/// the per-scan information matrix; see evaluate_degeneracy). Information from n points with weak-direction
/// measurement noise sigma is about n / sigma^2:
///   translation 1.1e5 ~= 100 points at sigma = 0.03 m along the weakest direction: 100 / 0.03^2 = 1.11e5.
///   rotation    2.8e6 ~= the same 100 points at a 5 m lever arm: 100 * 5^2 / 0.03^2 = 2.78e6.
/// These are beta starting values; S1b recalibrates both from SITL logs. The default YAML
/// (config/lio/sim.yaml) repeats the values and points back here ("see config.hpp").
///
/// Math keys (MathConfig): beta values from the survey of `main` (config/runtime/sim.yaml); bounds are about
/// 1/4x to 4x the beta value (for the negative extrinsic components: 4x .. 1/4x), except
/// registration_max_iterations [1, 10]. imu_init_min_samples is capped at 800 so it stays below
/// fast_lio_core's ImuInitializerConfig::maximum_imu_samples (1000). The range bounds do not overlap
/// (min <= 2 m < 10 m <= max), so preprocess_max_range_m > preprocess_min_range_m needs no cross check.
inline constexpr auto kLioSpecs = std::to_array<config::ParamSpec>({
    {"lifecycle_confirm_scans", config::Unit::kNone, 1.0, 50.0},
    {"lifecycle_degenerate_scans", config::Unit::kNone, 1.0, 50.0},
    {"lifecycle_gap_degraded_s", config::Unit::kSeconds, 0.05, 2.0},
    {"lifecycle_gap_lost_s", config::Unit::kSeconds, 0.1, 5.0},
    {"lifecycle_degeneracy_lost_s", config::Unit::kSeconds, 0.1, 10.0},
    {"lifecycle_position_sigma_lost_m", config::Unit::kMeters, 0.05, 5.0},
    {"degeneracy_translation_min_info", config::Unit::kNone, 1.0, 1e9},
    {"degeneracy_rotation_min_info", config::Unit::kNone, 1.0, 1e12},
    {"predictor_tau_vel_s", config::Unit::kSeconds, 0.05, 5.0},
    {"predictor_tau_pos_s", config::Unit::kSeconds, 0.05, 5.0},
    {"extrinsic_imu_lidar_x_m", config::Unit::kMeters, -0.044, -0.00275},
    {"extrinsic_imu_lidar_y_m", config::Unit::kMeters, -0.09316, -0.0058225},
    {"extrinsic_imu_lidar_z_m", config::Unit::kMeters, 0.01103, 0.17648},
    {"preprocess_min_range_m", config::Unit::kMeters, 0.125, 2.0},
    {"preprocess_max_range_m", config::Unit::kMeters, 10.0, 160.0},
    {"preprocess_voxel_m", config::Unit::kMeters, 0.05, 0.8},
    {"map_voxel_m", config::Unit::kMeters, 0.075, 1.2},
    {"map_half_extent_x_m", config::Unit::kMeters, 7.5, 120.0},
    {"map_half_extent_y_m", config::Unit::kMeters, 7.5, 120.0},
    {"map_half_extent_z_m", config::Unit::kMeters, 3.75, 60.0},
    {"registration_max_iterations", config::Unit::kNone, 1.0, 10.0},
    {"imu_init_min_samples", config::Unit::kNone, 50.0, 800.0},
    {"imu_max_gap_s", config::Unit::kSeconds, 0.005, 0.08},
});

/// Builds a LioConfig from values already loaded against kLioSpecs (config::load_params).
/// Fails with kMissingKey when a key is absent. Beyond the per-key bounds it rejects, as
/// kOutOfRange naming BOTH keys (first in `key`, both in `detail`):
///   lifecycle_gap_lost_s <= lifecycle_gap_degraded_s
///   lifecycle_degenerate_scans > lifecycle_confirm_scans * 10
/// A count key holding a non-integer value is kWrongType.
Result<LioConfig, config::ConfigError> load_lio_config(const config::ParamValues& values);

}  // namespace uavnav::lio
