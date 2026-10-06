#include "uavnav/px4bridge/config.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

#include "uavnav/core/time_convert.hpp"

namespace uavnav::px4bridge {

namespace {

using config::ConfigError;
using Kind = ConfigError::Kind;

std::unexpected<ConfigError> fail(Kind kind, std::string_view key, std::string detail) {
  return std::unexpected(ConfigError{kind, std::string(key), std::move(detail)});
}

// The value of `key`, re-checked against its kAlignmentSpecs entry. load_alignment_config is public and
// may be handed ParamValues that never went through load_params, so nothing is assumed.
Result<double, ConfigError> checked(const config::ParamValues& values, std::string_view key) {
  const auto v = config::value(values, key);
  if (!v) return std::unexpected(v.error());
  if (!std::isfinite(*v)) return fail(Kind::kNotFinite, key, std::string(key) + " must be finite");
  for (const config::ParamSpec& spec : kAlignmentSpecs) {
    if (spec.key == key && (*v < spec.min || *v > spec.max)) {
      return fail(Kind::kOutOfRange, key,
                  std::string(key) + " = " + std::to_string(*v) + " is outside [" + std::to_string(spec.min) +
                      ", " + std::to_string(spec.max) + "]");
    }
  }
  return *v;
}

// A count key: a whole number within the spec bounds, so the cast below cannot be undefined.
Result<std::uint32_t, ConfigError> count(const config::ParamValues& values, std::string_view key) {
  const auto v = checked(values, key);
  if (!v) return std::unexpected(v.error());
  if (std::floor(*v) != *v) return fail(Kind::kWrongType, key, std::string(key) + " must be a whole number");
  return static_cast<std::uint32_t>(*v);
}

Result<time::Duration, ConfigError> seconds_key(const config::ParamValues& values, std::string_view key) {
  const auto v = checked(values, key);
  if (!v) return std::unexpected(v.error());
  const auto d = time::duration_from_seconds(*v);
  if (!d) {
    return fail(Kind::kOutOfRange, key,
                std::string(key) + " is not a valid duration: " + std::string(to_string(d.error())));
  }
  return *d;
}

}  // namespace

Result<AlignmentConfig, ConfigError> load_alignment_config(const config::ParamValues& values) {
  const auto tau = seconds_key(values, "alignment_tau_s");
  if (!tau) return std::unexpected(tau.error());
  const auto pairs = count(values, "alignment_consistent_pairs");
  if (!pairs) return std::unexpected(pairs.error());
  const auto jump_position = checked(values, "alignment_jump_position_m");
  if (!jump_position) return std::unexpected(jump_position.error());
  const auto jump_yaw = checked(values, "alignment_jump_yaw_rad");
  if (!jump_yaw) return std::unexpected(jump_yaw.error());
  const auto max_rate = checked(values, "alignment_max_rate_mps");
  if (!max_rate) return std::unexpected(max_rate.error());
  const auto max_yaw_rate = checked(values, "alignment_max_yaw_rate_rad_s");
  if (!max_yaw_rate) return std::unexpected(max_yaw_rate.error());
  const auto valid_stale = seconds_key(values, "alignment_valid_stale_s");
  if (!valid_stale) return std::unexpected(valid_stale.error());
  const auto frozen_max = seconds_key(values, "alignment_frozen_max_s");
  if (!frozen_max) return std::unexpected(frozen_max.error());

  if (*frozen_max <= *valid_stale) {
    return fail(Kind::kOutOfRange, "alignment_frozen_max_s",
                "alignment_frozen_max_s (" + std::to_string(time::to_seconds(*frozen_max)) +
                    ") must be greater than alignment_valid_stale_s (" +
                    std::to_string(time::to_seconds(*valid_stale)) + ")");
  }

  return AlignmentConfig{*tau,          *pairs,       *jump_position, *jump_yaw,
                         *max_rate,     *max_yaw_rate, *valid_stale,   *frozen_max};
}

}  // namespace uavnav::px4bridge
