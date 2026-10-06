#pragma once

#include <array>

#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/lio/degeneracy.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"

// Typed tier-(b) configuration of the LIO core (SYSTEM_DESIGN §6.2). Later tasks append
// their keys to kLioSpecs (N is derived from the initializer) and their section to
// LioConfig / load_lio_config.
namespace uavnav::lio {

struct LioConfig {
  LifecycleConfig lifecycle;
  DegeneracyConfig degeneracy;
  PredictorConfig predictor;
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
/// These are beta starting values; S1b recalibrates both from SITL logs. The default YAML (Task 8) repeats
/// the values and points back here ("see config.hpp").
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
});

/// Builds a LioConfig from values already loaded against kLioSpecs (config::load_params).
/// Fails with kMissingKey when a key is absent. Beyond the per-key bounds it rejects, as
/// kOutOfRange naming BOTH keys (first in `key`, both in `detail`):
///   lifecycle_gap_lost_s <= lifecycle_gap_degraded_s
///   lifecycle_degenerate_scans > lifecycle_confirm_scans * 10
/// A count key holding a non-integer value is kWrongType.
Result<LioConfig, config::ConfigError> load_lio_config(const config::ParamValues& values);

}  // namespace uavnav::lio
