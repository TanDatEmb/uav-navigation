#include "odometry_supervisor/odometry_alignment_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace odometry_supervisor {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double wrap(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double heading(const Eigen::Quaterniond& orientation) {
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double tiltDisagreement(const Eigen::Quaterniond& lio,
                        const Eigen::Quaterniond& px4) {
  const Eigen::Vector3d lio_z = lio.normalized().toRotationMatrix().col(2);
  const Eigen::Vector3d px4_z = px4.normalized().toRotationMatrix().col(2);
  return std::acos(std::clamp(lio_z.dot(px4_z), -1.0, 1.0));
}

template <typename Value>
Value median(std::vector<Value> values) {
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2U != 0U) return *middle;
  const auto lower = std::max_element(values.begin(), middle);
  return (*lower + *middle) / static_cast<Value>(2);
}

double weightedCircularMean(const std::vector<double>& angles,
                            const std::vector<double>& weights) {
  double sine = 0.0;
  double cosine = 0.0;
  for (std::size_t index = 0; index < angles.size(); ++index) {
    sine += weights[index] * std::sin(angles[index]);
    cosine += weights[index] * std::cos(angles[index]);
  }
  return std::atan2(sine, cosine);
}

double motionObservedYaw(const std::deque<AlignmentSample>& samples,
                         const std::vector<double>& weights) {
  Eigen::Vector2d mean_lio = Eigen::Vector2d::Zero();
  Eigen::Vector2d mean_px4 = Eigen::Vector2d::Zero();
  double weight_sum = 0.0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    mean_lio += weights[index] * samples[index].lio_position.head<2>();
    mean_px4 += weights[index] * samples[index].px4_position.head<2>();
    weight_sum += weights[index];
  }
  if (!(weight_sum > 0.0)) return 0.0;
  mean_lio /= weight_sum;
  mean_px4 /= weight_sum;
  double sine = 0.0;
  double cosine = 0.0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const Eigen::Vector2d source = samples[index].px4_position.head<2>() - mean_px4;
    const Eigen::Vector2d target = samples[index].lio_position.head<2>() - mean_lio;
    cosine += weights[index] * source.dot(target);
    sine += weights[index] * (source.x() * target.y() - source.y() * target.x());
  }
  return std::atan2(sine, cosine);
}

double weightedSlope(const std::vector<double>& times,
                     const std::vector<double>& values,
                     const std::vector<double>& weights) {
  double weight_sum = 0.0;
  double time_mean = 0.0;
  double value_mean = 0.0;
  for (std::size_t index = 0; index < times.size(); ++index) {
    weight_sum += weights[index];
    time_mean += weights[index] * times[index];
    value_mean += weights[index] * values[index];
  }
  if (!(weight_sum > 0.0)) return std::numeric_limits<double>::infinity();
  time_mean /= weight_sum;
  value_mean /= weight_sum;
  double numerator = 0.0;
  double denominator = 0.0;
  for (std::size_t index = 0; index < times.size(); ++index) {
    const double dt = times[index] - time_mean;
    numerator += weights[index] * dt * (values[index] - value_mean);
    denominator += weights[index] * dt * dt;
  }
  return denominator > 1e-12 ? numerator / denominator : 0.0;
}

AlignmentEstimate invalid(AlignmentEstimateStatus status, const char* reason,
                          std::size_t count) {
  AlignmentEstimate result;
  result.status = status;
  result.rejection_reason = reason;
  result.sample_count = count;
  result.alignment.valid = false;
  return result;
}

}  // namespace

const char* toString(const AlignmentEstimateStatus status) noexcept {
  switch (status) {
    case AlignmentEstimateStatus::kValid: return "valid";
    case AlignmentEstimateStatus::kInsufficientSamples: return "insufficient_samples";
    case AlignmentEstimateStatus::kGenerationChanged: return "generation_changed";
    case AlignmentEstimateStatus::kInvalidSample: return "invalid_sample";
    case AlignmentEstimateStatus::kLioNotTracking: return "lio_not_tracking";
    case AlignmentEstimateStatus::kPx4ContinuityInvalid: return "px4_continuity_invalid";
    case AlignmentEstimateStatus::kYawUnobservable: return "yaw_unobservable";
    case AlignmentEstimateStatus::kRollPitchDisagreement: return "roll_pitch_disagreement";
    case AlignmentEstimateStatus::kDispersionExceeded: return "dispersion_exceeded";
    case AlignmentEstimateStatus::kResidualTrendExceeded: return "residual_trend_exceeded";
    case AlignmentEstimateStatus::kCovarianceInvalid: return "covariance_invalid";
  }
  return "unknown";
}

OdometryAlignmentEstimator::OdometryAlignmentEstimator(
    OdometryAlignmentEstimatorConfig config)
    : config_(std::move(config)) {}

bool OdometryAlignmentEstimator::validSample(const AlignmentSample& sample) noexcept {
  return sample.timestamp_ns > 0 && sample.lio_position.allFinite() &&
         sample.px4_position.allFinite() && sample.lio_orientation.coeffs().allFinite() &&
         sample.px4_orientation.coeffs().allFinite() &&
         std::isfinite(sample.lio_orientation.norm()) &&
         std::isfinite(sample.px4_orientation.norm()) &&
         sample.lio_orientation.norm() > 1e-9 && sample.px4_orientation.norm() > 1e-9 &&
         std::isfinite(sample.weight) && sample.weight > 0.0;
}

void OdometryAlignmentEstimator::reset() { samples_.clear(); }

bool OdometryAlignmentEstimator::addSample(AlignmentSample sample) {
  if (!validSample(sample)) return false;
  if (!samples_.empty()) {
    const auto& previous = samples_.back();
    if (sample.timestamp_ns <= previous.timestamp_ns ||
        sample.lio_generation != previous.lio_generation ||
        sample.px4_reset_generation != previous.px4_reset_generation ||
        sample.px4_frame_generation != previous.px4_frame_generation ||
        sample.px4_time_generation != previous.px4_time_generation) {
      samples_.clear();
    }
  }
  samples_.push_back(std::move(sample));
  while (samples_.size() > config_.window_size) samples_.pop_front();
  return true;
}

AlignmentEstimate OdometryAlignmentEstimator::estimate() const {
  if (samples_.size() < config_.minimum_samples) {
    return invalid(AlignmentEstimateStatus::kInsufficientSamples,
                   "alignment window has too few paired samples", samples_.size());
  }
  const AlignmentSample& first = samples_.front();
  for (const auto& sample : samples_) {
    if (!validSample(sample)) {
      return invalid(AlignmentEstimateStatus::kInvalidSample, "alignment sample is invalid",
                     samples_.size());
    }
    if (sample.lio_generation != first.lio_generation ||
        sample.px4_reset_generation != first.px4_reset_generation ||
        sample.px4_frame_generation != first.px4_frame_generation ||
        sample.px4_time_generation != first.px4_time_generation) {
      return invalid(AlignmentEstimateStatus::kGenerationChanged,
                     "alignment window crosses a generation", samples_.size());
    }
    if (!sample.lio_tracking) {
      return invalid(AlignmentEstimateStatus::kLioNotTracking,
                     "LIO is not tracking during alignment window", samples_.size());
    }
    if (!sample.px4_continuity_valid) {
      return invalid(AlignmentEstimateStatus::kPx4ContinuityInvalid,
                     "PX4 continuity is invalid during alignment window", samples_.size());
    }
  }

  std::vector<double> angles;
  std::vector<double> weights;
  std::vector<double> tilts;
  std::vector<double> authoritative_angles;
  std::vector<double> authoritative_weights;
  angles.reserve(samples_.size());
  weights.reserve(samples_.size());
  tilts.reserve(samples_.size());
  authoritative_angles.reserve(samples_.size());
  authoritative_weights.reserve(samples_.size());
  bool yaw_authoritative = false;
  for (const auto& sample : samples_) {
    angles.push_back(wrap(heading(sample.lio_orientation) - heading(sample.px4_orientation)));
    weights.push_back(sample.weight);
    tilts.push_back(tiltDisagreement(sample.lio_orientation, sample.px4_orientation));
    yaw_authoritative = yaw_authoritative || sample.yaw_authoritative;
    if (sample.yaw_authoritative) {
      authoritative_angles.push_back(angles.back());
      authoritative_weights.push_back(sample.weight);
    }
  }
  const double preliminary_yaw = yaw_authoritative
                                     ? weightedCircularMean(authoritative_angles,
                                                            authoritative_weights)
                                     : motionObservedYaw(samples_, weights);
  std::vector<Eigen::Vector3d> translations;
  translations.reserve(samples_.size());
  Eigen::Vector3d preliminary_translation = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;
  for (std::size_t index = 0; index < samples_.size(); ++index) {
    const Eigen::Vector3d translation =
        samples_[index].lio_position -
        Eigen::AngleAxisd(preliminary_yaw, Eigen::Vector3d::UnitZ()) *
            samples_[index].px4_position;
    translations.push_back(translation);
    preliminary_translation += weights[index] * translation;
    weight_sum += weights[index];
  }
  preliminary_translation /= weight_sum;

  std::vector<double> position_residuals;
  std::vector<double> yaw_residuals;
  position_residuals.reserve(samples_.size());
  yaw_residuals.reserve(samples_.size());
  for (std::size_t index = 0; index < samples_.size(); ++index) {
    position_residuals.push_back((translations[index] - preliminary_translation).norm());
    yaw_residuals.push_back(yaw_authoritative && samples_[index].yaw_authoritative
                                ? std::abs(wrap(angles[index] - preliminary_yaw))
                                : 0.0);
  }
  const double position_median = median(position_residuals);
  const double yaw_median = median(yaw_residuals);
  std::vector<double> position_deviations;
  std::vector<double> yaw_deviations;
  for (std::size_t index = 0; index < samples_.size(); ++index) {
    position_deviations.push_back(std::abs(position_residuals[index] - position_median));
    yaw_deviations.push_back(std::abs(yaw_residuals[index] - yaw_median));
  }
  const double position_gate = std::max(config_.minimum_outlier_gate_m,
                                        position_median + 3.0 * median(position_deviations));
  const double yaw_gate = std::max(config_.minimum_outlier_gate_rad,
                                   yaw_median + 3.0 * median(yaw_deviations));
  std::vector<std::size_t> inliers;
  for (std::size_t index = 0; index < samples_.size(); ++index) {
    if (position_residuals[index] <= position_gate && yaw_residuals[index] <= yaw_gate) {
      inliers.push_back(index);
    }
  }
  if (inliers.size() < config_.minimum_samples) {
    return invalid(AlignmentEstimateStatus::kDispersionExceeded,
                   "alignment outlier rejection left too few samples", samples_.size());
  }

  std::vector<double> inlier_angles;
  std::vector<double> inlier_weights;
  std::vector<double> inlier_authoritative_angles;
  std::vector<double> inlier_authoritative_weights;
  inlier_angles.reserve(inliers.size());
  inlier_weights.reserve(inliers.size());
  inlier_authoritative_angles.reserve(inliers.size());
  inlier_authoritative_weights.reserve(inliers.size());
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  weight_sum = 0.0;
  double inlier_weight_square_sum = 0.0;
  double maximum_tilt = 0.0;
  for (const auto index : inliers) {
    inlier_angles.push_back(angles[index]);
    inlier_weights.push_back(weights[index]);
    if (samples_[index].yaw_authoritative) {
      inlier_authoritative_angles.push_back(angles[index]);
      inlier_authoritative_weights.push_back(weights[index]);
    }
    translation += weights[index] * translations[index];
    weight_sum += weights[index];
    inlier_weight_square_sum += weights[index] * weights[index];
    maximum_tilt = std::max(maximum_tilt, tilts[index]);
  }
  double yaw = 0.0;
  if (yaw_authoritative && !inlier_authoritative_angles.empty()) {
    yaw = weightedCircularMean(inlier_authoritative_angles,
                               inlier_authoritative_weights);
  } else {
    std::deque<AlignmentSample> inlier_samples;
    inlier_samples.clear();
    for (const auto index : inliers) inlier_samples.push_back(samples_[index]);
    yaw = motionObservedYaw(inlier_samples, inlier_weights);
  }
  translation = Eigen::Vector3d::Zero();
  weight_sum = 0.0;
  const Eigen::AngleAxisd final_yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
  for (const auto index : inliers) {
    translations[index] = samples_[index].lio_position -
                          final_yaw_rotation * samples_[index].px4_position;
    translation += weights[index] * translations[index];
    weight_sum += weights[index];
  }
  translation /= weight_sum;

  double translation_squared = 0.0;
  double yaw_squared = 0.0;
  Eigen::Matrix3d translation_covariance = Eigen::Matrix3d::Zero();
  std::vector<double> times;
  std::vector<double> position_trends;
  std::vector<double> yaw_trends;
  times.reserve(inliers.size());
  position_trends.reserve(inliers.size());
  yaw_trends.reserve(inliers.size());
  for (const auto index : inliers) {
    const Eigen::Vector3d residual = translations[index] - translation;
    const double yaw_residual = yaw_authoritative && samples_[index].yaw_authoritative
                                    ? wrap(angles[index] - yaw)
                                    : 0.0;
    translation_squared += weights[index] * residual.squaredNorm();
    yaw_squared += weights[index] * yaw_residual * yaw_residual;
    translation_covariance += weights[index] * (residual * residual.transpose());
    times.push_back(static_cast<double>(samples_[index].timestamp_ns) * 1e-9);
    position_trends.push_back(residual.norm());
    yaw_trends.push_back(std::abs(yaw_residual));
  }
  const double effective_sample_count =
      weight_sum * weight_sum /
      std::max(inlier_weight_square_sum, std::numeric_limits<double>::min());
  const double unbiased_denominator =
      weight_sum * std::max(1.0 - 1.0 / std::max(effective_sample_count, 1.0), 1e-12);
  translation_covariance /= unbiased_denominator;
  const double translation_dispersion = std::sqrt(translation_squared / weight_sum);
  const double yaw_dispersion = std::sqrt(yaw_squared / weight_sum);
  const double position_trend = std::abs(weightedSlope(times, position_trends, inlier_weights));
  const double yaw_trend = std::abs(weightedSlope(times, yaw_trends, inlier_weights));

  Eigen::Vector3d mean_lio = Eigen::Vector3d::Zero();
  for (const auto index : inliers) mean_lio += samples_[index].lio_position;
  mean_lio /= static_cast<double>(inliers.size());
  double excitation = 0.0;
  for (const auto index : inliers) {
    const Eigen::Vector3d delta = samples_[index].lio_position - mean_lio;
    excitation = std::max(excitation, delta.head<2>().norm());
  }
  if (!yaw_authoritative && excitation < config_.minimum_horizontal_excitation_m) {
    return invalid(AlignmentEstimateStatus::kYawUnobservable,
                   "yaw is unobservable without authoritative yaw or horizontal motion",
                   inliers.size());
  }
  if (maximum_tilt > config_.maximum_roll_pitch_disagreement_rad) {
    return invalid(AlignmentEstimateStatus::kRollPitchDisagreement,
                   "roll/pitch disagreement cannot be absorbed by world alignment", inliers.size());
  }
  if (translation_dispersion > config_.maximum_translation_dispersion_m ||
      yaw_dispersion > config_.maximum_yaw_dispersion_rad) {
    return invalid(AlignmentEstimateStatus::kDispersionExceeded,
                   "alignment residual dispersion exceeded gate", inliers.size());
  }
  if (position_trend > config_.maximum_translation_residual_trend_m_s ||
      yaw_trend > config_.maximum_yaw_residual_trend_rad_s) {
    return invalid(AlignmentEstimateStatus::kResidualTrendExceeded,
                   "alignment residual trend exceeded gate", inliers.size());
  }
  if (!translation_covariance.allFinite() || !std::isfinite(yaw_dispersion)) {
    return invalid(AlignmentEstimateStatus::kCovarianceInvalid,
                   "alignment covariance is non-finite", inliers.size());
  }

  AlignmentEstimate result;
  result.status = AlignmentEstimateStatus::kValid;
  result.alignment.target_frame = "lio_odom";
  result.alignment.source_frame = "px4_odom";
  result.alignment.target_from_source_orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  result.alignment.target_from_source_translation = translation;
  result.alignment.yaw_rad = yaw;
  result.alignment.roll_pitch_disagreement_rad = maximum_tilt;
  result.alignment.effective_sample_count = effective_sample_count;
  result.alignment.yaw_mode =
      yaw_authoritative ? "ORIENTATION_AIDED" : "MOTION_OBSERVED";
  result.alignment.epoch_ns = samples_.back().timestamp_ns;
  result.alignment.epoch_start_ns = samples_.front().timestamp_ns;
  result.alignment.epoch_end_ns = samples_.back().timestamp_ns;
  result.alignment.sample_count = inliers.size();
  result.alignment.reset_generation = first.px4_reset_generation;
  result.alignment.frame_generation = first.px4_frame_generation;
  result.alignment.time_generation = first.px4_time_generation;
  result.alignment.lio_generation = first.lio_generation;
  result.alignment.source = "paired_timestamp.4dof.circular_mean";
  result.alignment.valid = true;
  result.covariance.topLeftCorner<3, 3>() = translation_covariance;
  result.covariance(3, 3) = yaw_authoritative
                               ? yaw_squared / unbiased_denominator
                               : 0.0;
  for (int index = 0; index < 4; ++index) {
    result.covariance(index, index) =
        std::max(result.covariance(index, index), config_.covariance_floor);
  }
  result.alignment.covariance = result.covariance;
  result.translation_dispersion_m = translation_dispersion;
  result.yaw_dispersion_rad = yaw_dispersion;
  result.roll_pitch_disagreement_rad = maximum_tilt;
  result.excitation_metric_m = excitation;
  result.translation_residual_trend_m_s = position_trend;
  result.yaw_residual_trend_rad_s = yaw_trend;
  result.effective_sample_count = effective_sample_count;
  result.yaw_mode = yaw_authoritative ? AlignmentYawMode::kOrientationAided
                                      : AlignmentYawMode::kMotionObserved;
  result.sample_count = inliers.size();
  result.rejected_outlier_count = samples_.size() - inliers.size();
  result.epoch_start_ns = samples_.front().timestamp_ns;
  result.epoch_end_ns = samples_.back().timestamp_ns;
  result.lio_generation = first.lio_generation;
  result.px4_reset_generation = first.px4_reset_generation;
  result.px4_frame_generation = first.px4_frame_generation;
  result.px4_time_generation = first.px4_time_generation;
  return result;
}

}  // namespace odometry_supervisor
