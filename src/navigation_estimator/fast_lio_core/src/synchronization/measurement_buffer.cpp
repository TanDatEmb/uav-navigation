#include "fast_lio_core/synchronization/measurement_buffer.hpp"

#include <algorithm>
#include <utility>

namespace uav::nav::lio {
namespace {

[[nodiscard]] TimestampValidatorConfig validatorConfig(const MeasurementBufferConfig& config) {
  TimestampValidatorConfig validator_config;
  validator_config.reject_regression = config.reject_timestamp_regression;
  validator_config.allow_equal_timestamps = false;
  return validator_config;
}

void accountTimestampError(const Status& status, MeasurementBufferStats& stats) {
  if (status.code() == StatusCode::kTimestampRegression) {
    ++stats.timestamp_regressions;
  } else if (status.code() == StatusCode::kClockDomainMismatch) {
    ++stats.clock_domain_mismatches;
  }
}

}  // namespace

MeasurementBuffer::MeasurementBuffer(MeasurementBufferConfig config)
    : config_(config),
      lidar_start_validator_(validatorConfig(config)),
      lidar_end_validator_(validatorConfig(config)),
      imu_validator_(validatorConfig(config)) {}

Status MeasurementBuffer::pushLidar(LidarScan scan) {
  const Status scan_status = scan.validate();
  if (!scan_status.ok()) {
    std::scoped_lock lock(mutex_);
    ++stats_.rejected_lidar_scans;
    accountTimestampError(scan_status, stats_);
    return scan_status;
  }

  std::scoped_lock lock(mutex_);
  if (lidar_scans_.size() >= config_.maximum_lidar_scans) {
    ++stats_.rejected_lidar_scans;
    ++stats_.buffer_full_rejections;
    return Status(StatusCode::kBufferFull, "LiDAR scan buffer is full");
  }
  TimestampValidator start_validator_candidate = lidar_start_validator_;
  TimestampValidator end_validator_candidate = lidar_end_validator_;
  const std::size_t prior_start_regressions = start_validator_candidate.regressionCount();
  const std::size_t prior_end_regressions = end_validator_candidate.regressionCount();
  const Status start_status = start_validator_candidate.validate(scan.start_time);
  if (!start_status.ok()) {
    ++stats_.rejected_lidar_scans;
    accountTimestampError(start_status, stats_);
    return start_status;
  }
  const Status end_status = end_validator_candidate.validate(scan.end_time);
  if (!end_status.ok()) {
    ++stats_.rejected_lidar_scans;
    accountTimestampError(end_status, stats_);
    return end_status;
  }
  lidar_start_validator_ = std::move(start_validator_candidate);
  lidar_end_validator_ = std::move(end_validator_candidate);
  stats_.timestamp_regressions +=
      lidar_start_validator_.regressionCount() - prior_start_regressions;
  stats_.timestamp_regressions += lidar_end_validator_.regressionCount() - prior_end_regressions;
  if (config_.reject_timestamp_regression || lidar_scans_.empty() ||
      lidar_scans_.back().start_time.nanoseconds() < scan.start_time.nanoseconds()) {
    lidar_scans_.push_back(std::move(scan));
  } else {
    const auto insertion =
        std::upper_bound(lidar_scans_.begin(), lidar_scans_.end(), scan.start_time.nanoseconds(),
                         [](std::int64_t time_ns, const LidarScan& queued_scan) {
                           return time_ns < queued_scan.start_time.nanoseconds();
                         });
    lidar_scans_.insert(insertion, std::move(scan));
  }
  ++stats_.accepted_lidar_scans;
  return Status::Ok();
}

Status MeasurementBuffer::pushImu(ImuSample sample) {
  const Status sample_status = sample.validate();
  if (!sample_status.ok()) {
    std::scoped_lock lock(mutex_);
    ++stats_.rejected_imu_samples;
    return sample_status;
  }

  std::scoped_lock lock(mutex_);
  if (imu_samples_.size() >= config_.maximum_imu_samples) {
    ++stats_.rejected_imu_samples;
    ++stats_.buffer_full_rejections;
    return Status(StatusCode::kBufferFull, "IMU sample buffer is full");
  }
  const std::size_t prior_regressions = imu_validator_.regressionCount();
  const Status time_status = imu_validator_.validate(sample.time);
  if (!time_status.ok()) {
    ++stats_.rejected_imu_samples;
    accountTimestampError(time_status, stats_);
    return time_status;
  }
  stats_.timestamp_regressions += imu_validator_.regressionCount() - prior_regressions;
  if (config_.reject_timestamp_regression || imu_samples_.empty() ||
      imu_samples_.back().time.nanoseconds() < sample.time.nanoseconds()) {
    imu_samples_.push_back(std::move(sample));
  } else {
    const auto insertion =
        std::upper_bound(imu_samples_.begin(), imu_samples_.end(), sample.time.nanoseconds(),
                         [](std::int64_t time_ns, const ImuSample& queued_sample) {
                           return time_ns < queued_sample.time.nanoseconds();
                         });
    imu_samples_.insert(insertion, std::move(sample));
  }
  ++stats_.accepted_imu_samples;
  return Status::Ok();
}

std::size_t MeasurementBuffer::lidarSize() const {
  std::scoped_lock lock(mutex_);
  return lidar_scans_.size();
}

std::optional<Timestamp> MeasurementBuffer::nextLidarStartTime() const {
  std::scoped_lock lock(mutex_);
  if (lidar_scans_.empty()) {
    return std::nullopt;
  }
  return lidar_scans_.front().start_time;
}

std::size_t MeasurementBuffer::imuSize() const {
  std::scoped_lock lock(mutex_);
  return imu_samples_.size();
}

bool MeasurementBuffer::empty() const {
  std::scoped_lock lock(mutex_);
  return lidar_scans_.empty() && imu_samples_.empty();
}

MeasurementBufferStats MeasurementBuffer::stats() const {
  std::scoped_lock lock(mutex_);
  return stats_;
}

void MeasurementBuffer::clear() {
  std::scoped_lock lock(mutex_);
  lidar_scans_.clear();
  imu_samples_.clear();
  lidar_start_validator_.reset();
  lidar_end_validator_.reset();
  imu_validator_.reset();
  stats_ = {};
}

}  // namespace uav::nav::lio
