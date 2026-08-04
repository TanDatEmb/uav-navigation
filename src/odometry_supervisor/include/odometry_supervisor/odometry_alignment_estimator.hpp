#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

struct AlignmentSample {
  std::int64_t timestamp_ns{0};
  Eigen::Vector3d lio_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond lio_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d px4_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond px4_orientation{Eigen::Quaterniond::Identity()};
  double weight{1.0};
  std::uint64_t lio_generation{0};
  std::uint64_t px4_reset_generation{0};
  std::uint64_t px4_time_generation{0};
  bool lio_tracking{false};
  bool px4_continuity_valid{false};
  bool yaw_authoritative{false};
};

enum class AlignmentEstimateStatus : std::uint8_t {
  kValid = 0,
  kInsufficientSamples,
  kGenerationChanged,
  kInvalidSample,
  kLioNotTracking,
  kPx4ContinuityInvalid,
  kYawUnobservable,
  kRollPitchDisagreement,
  kDispersionExceeded,
  kResidualTrendExceeded,
  kCovarianceInvalid,
};

[[nodiscard]] const char* toString(AlignmentEstimateStatus status) noexcept;

struct OdometryAlignmentEstimatorConfig {
  std::size_t window_size{32};
  std::size_t minimum_samples{8};
  double minimum_horizontal_excitation_m{0.20};
  double maximum_roll_pitch_disagreement_rad{0.17453292519943295};
  double maximum_translation_dispersion_m{0.50};
  double maximum_yaw_dispersion_rad{0.20};
  double maximum_translation_residual_trend_m_s{0.20};
  double maximum_yaw_residual_trend_rad_s{0.20};
  double minimum_outlier_gate_m{0.10};
  double minimum_outlier_gate_rad{0.05};
  double covariance_floor{1e-6};
};

struct AlignmentEstimate {
  AlignmentEstimateStatus status{AlignmentEstimateStatus::kInsufficientSamples};
  WorldAlignment alignment{};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Zero()};
  double translation_dispersion_m{0.0};
  double yaw_dispersion_rad{0.0};
  double roll_pitch_disagreement_rad{0.0};
  double excitation_metric_m{0.0};
  double translation_residual_trend_m_s{0.0};
  double yaw_residual_trend_rad_s{0.0};
  std::size_t sample_count{0};
  std::size_t rejected_outlier_count{0};
  std::int64_t epoch_start_ns{0};
  std::int64_t epoch_end_ns{0};
  std::uint64_t lio_generation{0};
  std::uint64_t px4_reset_generation{0};
  std::uint64_t px4_time_generation{0};
  std::string rejection_reason;

  [[nodiscard]] bool valid() const noexcept {
    return status == AlignmentEstimateStatus::kValid && alignment.valid;
  }
};

class OdometryAlignmentEstimator {
 public:
  explicit OdometryAlignmentEstimator(OdometryAlignmentEstimatorConfig config = {});

  [[nodiscard]] static bool validSample(const AlignmentSample& sample) noexcept;
  void reset();
  [[nodiscard]] bool addSample(AlignmentSample sample);
  [[nodiscard]] AlignmentEstimate estimate() const;
  [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }

 private:
  OdometryAlignmentEstimatorConfig config_;
  std::deque<AlignmentSample> samples_;
};

}  // namespace odometry_supervisor
