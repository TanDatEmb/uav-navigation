#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>

#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/lio/backend_config.hpp"
#include "uavnav/lio/degeneracy.hpp"
#include "uavnav/lio/frontend_config.hpp"
#include "uavnav/lio/limits.hpp"

// Typed tier-(b) configuration of the LIO core (SYSTEM_DESIGN §6.2). Later tasks append
// their keys to kLioSpecs (N is derived from the initializer) and their section to
// LioConfig / load_lio_config.
namespace uavnav::lio {

/// The config is split by block (SYSTEM_DESIGN §3.5, D32): backend_config.hpp (EstimatorMathConfig,
/// DegeneracyConfig: the estimator thread) and frontend_config.hpp (LifecycleConfig, PredictorConfig: the
/// ingest thread). This header merges them with the one key table and the loader; ONLY the two-thread
/// assembly (LioPipeline, LioRunner), the node and tests include it, never a backend or frontend file.
/// LioConfig keeps its four members (lifecycle, degeneracy, predictor, math); backend_config() and
/// frontend_config() hand each block its part.
using MathConfig = EstimatorMathConfig;  ///< the S1a name, kept for load_math and test_lio_config

struct LioConfig {
  LifecycleConfig lifecycle;
  DegeneracyConfig degeneracy;
  PredictorConfig predictor;
  EstimatorMathConfig math;
};

inline BackendConfig backend_config(const LioConfig& c) { return BackendConfig{c.math, c.degeneracy}; }
inline FrontendConfig frontend_config(const LioConfig& c) {
  return FrontendConfig{c.lifecycle, c.predictor, c.math.imu_max_gap};
}

/// The single definition of every LIO key, its unit and its inclusive bounds. Beta values:
/// confirm 5, degenerate 3, gap_degraded 0.25 s, gap_lost 0.5 s, degeneracy_lost 1.0 s, sigma_lost 0.5 m, restart_dwell 2.0 s,
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
/// the per-scan information matrix; see evaluate_degeneracy). The pair comes from analysis (S1a): information from
/// n points with weak-direction measurement noise sigma is about n / sigma^2, with n and sigma ASSUMED, not measured:
///   translation 1.1e5 ~= 100 points at sigma = 0.03 m along the weakest direction: 100 / 0.03^2 = 1.11e5.
///   rotation    2.8e6 ~= the same 100 points at a 5 m lever arm: 100 * 5^2 / 0.03^2 = 2.78e6.
/// The S1b SITL altitude sweep could not calibrate the pair (D36;
/// docs/analysis/2026-10-s1b-degeneracy-calibration.md): in every 1 s window before the first LOST the drift stayed
/// <= 0.05 m, so the data has no degenerate region to place a threshold (the scans below the pair are not applied,
/// D31, and LOST follows within 1 s). The values are kept, not validated. Measured there: hover at 2.3 m
/// translation p5 4.9e5, rotation p5 2.5e7; walls only (4.75 m, floor out of range) translation ~9e2, rotation
/// ~1.1e5; the synthetic floor-only scan translation 1.7e3. The report names the flight that would give support.
/// The default YAML (config/lio/sim.yaml) repeats the values and points back here ("see config.hpp").
///
/// Math keys (EstimatorMathConfig): beta values from the survey of `main` (config/runtime/sim.yaml); bounds are about
/// 1/4x to 4x the beta value, except: the IMU -> LiDAR extrinsic components use the physical bound +-0.5 m
/// (an IMU-LiDAR offset on one airframe; 0 allowed), registration_max_iterations [1, 10], and imu_max_gap_s
/// [0.01, 0.08] (at least two 200 Hz periods, so one late sample is not a gap). imu_init_min_samples is
/// capped at 800 so it stays below fast_lio_core's ImuInitializerConfig::maximum_imu_samples (1000). The range bounds do not overlap
/// (min <= 2 m < 10 m <= max), so preprocess_max_range_m > preprocess_min_range_m needs no cross check.
///
/// lifecycle_restart_dwell_s (D41, AR4): the one wait of the recovery chain, LOST -> RESTARTING after this long of
/// non-empty scans without a gap, and INITIALIZING (map ready) -> LOST without a confirmation after this long. A fixed
/// value (D44.7), bounds [0.5, 30] s, at most what a hover can wait without a decision. Cross-checked at load (review
/// T3 I1): restart_dwell >= (confirm_scans - 1) x kScanPeriod + gap_degraded (0.65 s at beta), the longest a run of
/// fresh confirming scans can take after kMapReady; below it TRACKING is unreachable.
///
/// lifecycle_gap_degraded_s: the lower bound is limits::kGapDegradedMin (0.2 s, two scan periods; D40, AR4), the bound
/// the lio node's future-stamp limit is asserted against.
inline constexpr auto kLioSpecs = std::to_array<config::ParamSpec>({
    {"lifecycle_confirm_scans", config::Unit::kNone, 1.0, 50.0},
    {"lifecycle_degenerate_scans", config::Unit::kNone, 1.0, 50.0},
    {"lifecycle_gap_degraded_s", config::Unit::kSeconds, time::to_seconds(limits::kGapDegradedMin), 2.0},
    {"lifecycle_gap_lost_s", config::Unit::kSeconds, 0.1, 5.0},
    {"lifecycle_degeneracy_lost_s", config::Unit::kSeconds, 0.1, 10.0},
    {"lifecycle_position_sigma_lost_m", config::Unit::kMeters, 0.05, 5.0},
    {"lifecycle_restart_dwell_s", config::Unit::kSeconds, 0.5, 30.0},
    {"degeneracy_translation_min_info", config::Unit::kNone, 1.0, 1e9},
    {"degeneracy_rotation_min_info", config::Unit::kNone, 1.0, 1e12},
    {"predictor_tau_vel_s", config::Unit::kSeconds, 0.05, 5.0},
    {"predictor_tau_pos_s", config::Unit::kSeconds, 0.05, 5.0},
    {"extrinsic_imu_lidar_x_m", config::Unit::kMeters, -0.5, 0.5},
    {"extrinsic_imu_lidar_y_m", config::Unit::kMeters, -0.5, 0.5},
    {"extrinsic_imu_lidar_z_m", config::Unit::kMeters, -0.5, 0.5},
    {"preprocess_min_range_m", config::Unit::kMeters, 0.125, 2.0},
    {"preprocess_max_range_m", config::Unit::kMeters, 10.0, 160.0},
    {"preprocess_voxel_m", config::Unit::kMeters, 0.05, 0.8},
    {"map_voxel_m", config::Unit::kMeters, 0.075, 1.2},
    {"map_half_extent_x_m", config::Unit::kMeters, 7.5, 120.0},
    {"map_half_extent_y_m", config::Unit::kMeters, 7.5, 120.0},
    {"map_half_extent_z_m", config::Unit::kMeters, 3.75, 60.0},
    {"registration_max_iterations", config::Unit::kNone, 1.0, 10.0},
    {"imu_init_min_samples", config::Unit::kNone, 50.0, 800.0},
    {"imu_max_gap_s", config::Unit::kSeconds, 0.01, 0.08},
});

/// Builds a LioConfig from values already loaded against kLioSpecs (config::load_params).
/// Fails with kMissingKey when a key is absent. Beyond the per-key bounds it rejects, as
/// kOutOfRange naming BOTH keys (first in `key`, both in `detail`):
///   lifecycle_gap_lost_s <= lifecycle_gap_degraded_s
///   lifecycle_degenerate_scans > lifecycle_confirm_scans * 10
///   lifecycle_restart_dwell_s < (lifecycle_confirm_scans - 1) * kScanPeriod + lifecycle_gap_degraded_s
/// A count key holding a non-integer value is kWrongType.
Result<LioConfig, config::ConfigError> load_lio_config(const config::ParamValues& values);

}  // namespace uavnav::lio
