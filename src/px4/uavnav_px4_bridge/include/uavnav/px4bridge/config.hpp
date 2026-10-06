#pragma once

#include <array>

#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/px4bridge/alignment.hpp"
#include "uavnav/px4bridge/limits.hpp"

// Typed tier-(b) configuration of the PX4 bridge core (SYSTEM_DESIGN §6.2, §4.1): the alignment keys.
namespace uavnav::px4bridge {

/// The single definition of every alignment key, its unit and its inclusive bounds (task-10 brief).
/// Beta values: tau 2.0 s, consistent_pairs 20, jump_position 0.5 m, jump_yaw 0.0873 rad (5 deg),
/// max_rate 0.5 m/s, max_yaw_rate 0.0873 rad/s, valid_stale 1.0 s, frozen_max 10.0 s
/// (= lio_recovery_timeout_s, D18). Index order is relied on by the static_asserts below.
inline constexpr auto kAlignmentSpecs = std::to_array<config::ParamSpec>({
    {"alignment_tau_s", config::Unit::kSeconds, 0.2, 20.0},
    {"alignment_consistent_pairs", config::Unit::kNone, 1.0, 200.0},
    {"alignment_jump_position_m", config::Unit::kMeters, 0.05, 5.0},
    {"alignment_jump_yaw_rad", config::Unit::kRadians, 0.01, 0.5},
    {"alignment_max_rate_mps", config::Unit::kMetersPerSecond, 0.01, 5.0},
    {"alignment_max_yaw_rate_rad_s", config::Unit::kRadiansPerSecond, 0.001, 0.5},
    {"alignment_valid_stale_s", config::Unit::kSeconds, 0.2, 10.0},
    {"alignment_frozen_max_s", config::Unit::kSeconds, 1.0, 120.0},
});

// Invariants that hold for every loadable config, so they need no runtime cross check:
//  - alpha = dt / tau <= 1: the filter dt is clamped to kFilterDtMax and tau >= its spec min.
static_assert(kAlignmentSpecs[0].min * 1e9 >= static_cast<double>(limits::kFilterDtMax.ns));
//  - the INIT accumulation array holds the largest consistent_pairs.
static_assert(kAlignmentSpecs[1].max == static_cast<double>(limits::kMaxConsistentPairs));
//  - valid_stale covers one missed 10 Hz scan plus the pairing window on both sides (0.1 + 2 x 0.05 s),
//    so one unpaired scan never invalidates T.
static_assert(kAlignmentSpecs[6].min * 1e9 >=
              static_cast<double>(time::milliseconds(100).ns + 2 * limits::kPairingWindow.ns));

/// Builds an AlignmentConfig from values already loaded against kAlignmentSpecs (config::load_params).
/// Every value is re-checked against its spec (a hand-built ParamValues is not trusted): kMissingKey,
/// kNotFinite, kOutOfRange; alignment_consistent_pairs must be a whole number (kWrongType). Seconds go
/// through time::duration_from_seconds. Cross-field rule, kOutOfRange naming both keys (first in `key`):
///   alignment_frozen_max_s <= alignment_valid_stale_s
/// FROZEN is the tolerance for a LIO loss (D18 recovery timeout, 10 s); it must be longer than the VALID
/// staleness (no pair, 1 s), otherwise a LIO loss with T held would be invalidated sooner than a mere
/// pairing gap, inverting the §4.1 intent.
Result<AlignmentConfig, config::ConfigError> load_alignment_config(const config::ParamValues& values);

}  // namespace uavnav::px4bridge
