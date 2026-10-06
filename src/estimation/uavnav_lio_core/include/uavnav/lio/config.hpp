#pragma once

#include <array>

#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/lio/lifecycle.hpp"

// Typed tier-(b) configuration of the LIO core (SYSTEM_DESIGN §6.2). Later tasks append
// their keys to kLioSpecs (N is derived from the initializer) and their section to
// LioConfig / load_lio_config.
namespace uavnav::lio {

struct LioConfig {
  LifecycleConfig lifecycle;
};

/// The single definition of every LIO key, its unit and its inclusive bounds. Beta values:
/// confirm 5, degenerate 3, gap_degraded 0.25 s, gap_lost 0.5 s, degeneracy_lost 1.0 s, sigma_lost 0.5 m.
inline constexpr auto kLioSpecs = std::to_array<config::ParamSpec>({
    {"lifecycle_confirm_scans", config::Unit::kNone, 1.0, 50.0},
    {"lifecycle_degenerate_scans", config::Unit::kNone, 1.0, 50.0},
    {"lifecycle_gap_degraded_s", config::Unit::kSeconds, 0.05, 2.0},
    {"lifecycle_gap_lost_s", config::Unit::kSeconds, 0.1, 5.0},
    {"lifecycle_degeneracy_lost_s", config::Unit::kSeconds, 0.1, 10.0},
    {"lifecycle_position_sigma_lost_m", config::Unit::kMeters, 0.05, 5.0},
});

/// Builds a LioConfig from values already loaded against kLioSpecs (config::load_params).
/// Fails with kMissingKey when a key is absent. Beyond the per-key bounds it rejects, as
/// kOutOfRange naming BOTH keys (first in `key`, both in `detail`):
///   lifecycle_gap_lost_s <= lifecycle_gap_degraded_s
///   lifecycle_degenerate_scans > lifecycle_confirm_scans * 10
/// A count key holding a non-integer value is kWrongType.
Result<LioConfig, config::ConfigError> load_lio_config(const config::ParamValues& values);

}  // namespace uavnav::lio
