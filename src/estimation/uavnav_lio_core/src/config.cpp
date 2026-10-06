#include "uavnav/lio/config.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

#include "uavnav/core/time_convert.hpp"

namespace uavnav::lio {

namespace {

using config::ConfigError;
using Kind = ConfigError::Kind;

std::unexpected<ConfigError> fail(Kind kind, std::string_view key, std::string detail) {
  return std::unexpected(ConfigError{kind, std::string(key), std::move(detail)});
}

// The value of `key`, re-checked against its kLioSpecs entry. load_lio_config is public and may be
// handed hand-built ParamValues that never went through load_params, so nothing is assumed.
Result<double, ConfigError> checked(const config::ParamValues& values, std::string_view key) {
  const auto v = config::value(values, key);
  if (!v) return std::unexpected(v.error());
  if (!std::isfinite(*v)) return fail(Kind::kNotFinite, key, std::string(key) + " must be finite");
  for (const config::ParamSpec& spec : kLioSpecs) {
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

Result<LioConfig, ConfigError> load_lio_config(const config::ParamValues& values) {
  const auto confirm = count(values, "lifecycle_confirm_scans");
  if (!confirm) return std::unexpected(confirm.error());
  const auto degenerate = count(values, "lifecycle_degenerate_scans");
  if (!degenerate) return std::unexpected(degenerate.error());
  const auto gap_degraded = seconds_key(values, "lifecycle_gap_degraded_s");
  if (!gap_degraded) return std::unexpected(gap_degraded.error());
  const auto gap_lost = seconds_key(values, "lifecycle_gap_lost_s");
  if (!gap_lost) return std::unexpected(gap_lost.error());
  const auto degeneracy_lost = seconds_key(values, "lifecycle_degeneracy_lost_s");
  if (!degeneracy_lost) return std::unexpected(degeneracy_lost.error());
  const auto sigma = checked(values, "lifecycle_position_sigma_lost_m");
  if (!sigma) return std::unexpected(sigma.error());
  const auto translation_min_info = checked(values, "degeneracy_translation_min_info");
  if (!translation_min_info) return std::unexpected(translation_min_info.error());
  const auto rotation_min_info = checked(values, "degeneracy_rotation_min_info");
  if (!rotation_min_info) return std::unexpected(rotation_min_info.error());

  if (*gap_lost <= *gap_degraded) {
    return fail(Kind::kOutOfRange, "lifecycle_gap_lost_s",
                "lifecycle_gap_lost_s (" + std::to_string(time::to_seconds(*gap_lost)) +
                    ") must be greater than lifecycle_gap_degraded_s (" +
                    std::to_string(time::to_seconds(*gap_degraded)) + ")");
  }
  if (static_cast<std::uint64_t>(*degenerate) > static_cast<std::uint64_t>(*confirm) * 10U) {
    return fail(Kind::kOutOfRange, "lifecycle_degenerate_scans",
                "lifecycle_degenerate_scans (" + std::to_string(*degenerate) +
                    ") must not exceed 10 x lifecycle_confirm_scans (" + std::to_string(*confirm) + ")");
  }

  return LioConfig{LifecycleConfig{*confirm, *degenerate, *gap_degraded, *gap_lost, *degeneracy_lost, *sigma},
                   DegeneracyConfig{*translation_min_info, *rotation_min_info}};
}

}  // namespace uavnav::lio
