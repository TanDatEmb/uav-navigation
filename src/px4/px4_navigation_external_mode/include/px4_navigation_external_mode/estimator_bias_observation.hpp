#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <optional>

#include <Eigen/Core>

namespace px4_navigation_external_mode {

struct EstimatorBiasObservation final {
  std::uint64_t timestamp_us{0U};
  std::uint64_t timestamp_sample_us{0U};
  std::uint32_t device_id{0U};
  Eigen::Vector3d bias_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_var_m2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d innov_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d innov_var_m2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d innov_test_ratio{Eigen::Vector3d::Zero()};
};

template <typename Array>
[[nodiscard]] inline bool finiteArray(const Array& values) noexcept {
  for (const auto value : values) {
    if (!std::isfinite(static_cast<double>(value))) return false;
  }
  return true;
}

template <typename Array>
[[nodiscard]] inline bool nonNegativeFiniteArray(const Array& values) noexcept {
  for (const auto value : values) {
    if (!std::isfinite(static_cast<double>(value)) || value < 0.0F) return false;
  }
  return true;
}

[[nodiscard]] inline std::optional<EstimatorBiasObservation> makeEstimatorBiasObservation(
    const std::uint64_t timestamp_us, const std::uint64_t timestamp_sample_us,
    const std::uint32_t device_id, const std::array<float, 3>& bias,
    const std::array<float, 3>& bias_var, const std::array<float, 3>& innov,
    const std::array<float, 3>& innov_var,
    const std::array<float, 3>& innov_test_ratio) noexcept {
  if (timestamp_us == 0U || timestamp_sample_us == 0U ||
      timestamp_sample_us > timestamp_us || device_id == 0U ||
      !finiteArray(bias) || !nonNegativeFiniteArray(bias_var) ||
      !finiteArray(innov) || !nonNegativeFiniteArray(innov_var) ||
      !nonNegativeFiniteArray(innov_test_ratio)) {
    return std::nullopt;
  }
  EstimatorBiasObservation observation;
  observation.timestamp_us = timestamp_us;
  observation.timestamp_sample_us = timestamp_sample_us;
  observation.device_id = device_id;
  observation.bias_m = Eigen::Map<const Eigen::Vector3f>(bias.data()).cast<double>();
  observation.bias_var_m2 = Eigen::Map<const Eigen::Vector3f>(bias_var.data()).cast<double>();
  observation.innov_m = Eigen::Map<const Eigen::Vector3f>(innov.data()).cast<double>();
  observation.innov_var_m2 = Eigen::Map<const Eigen::Vector3f>(innov_var.data()).cast<double>();
  observation.innov_test_ratio =
      Eigen::Map<const Eigen::Vector3f>(innov_test_ratio.data()).cast<double>();
  if (!observation.bias_m.allFinite() || !observation.bias_var_m2.allFinite() ||
      !observation.innov_m.allFinite() || !observation.innov_var_m2.allFinite() ||
      !observation.innov_test_ratio.allFinite()) {
    return std::nullopt;
  }
  return observation;
}

}  // namespace px4_navigation_external_mode
