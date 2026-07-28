#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <string>
#include <utility>

namespace uav::nav::lio {

MeasurementSynchronizer::MeasurementSynchronizer(MeasurementSynchronizerConfig config)
    : config_(config) {}

Result<std::optional<MeasurementGroup>> MeasurementSynchronizer::synchronizeNext(
    MeasurementBuffer& buffer) {
  std::scoped_lock lock(buffer.mutex_);
  if (buffer.lidar_scans_.empty() || buffer.imu_samples_.empty()) {
    ++stats_.waiting_count;
    return std::optional<MeasurementGroup>{};
  }

  const LidarScan& scan = buffer.lidar_scans_.front();
  if (!scan.start_time.sameClockDomain(buffer.imu_samples_.front().time) ||
      !scan.start_time.sameClockDomain(buffer.imu_samples_.back().time)) {
    buffer.lidar_scans_.pop_front();
    ++stats_.rejected_clock_domain;
    return Status(StatusCode::kClockDomainMismatch,
                  "LiDAR and IMU queues use different clock domains");
  }

  const auto& imu = buffer.imu_samples_;
  if (last_synchronized_end_time_) {
    if (!last_synchronized_end_time_->sameClockDomain(scan.start_time)) {
      buffer.lidar_scans_.pop_front();
      ++stats_.rejected_clock_domain;
      return Status(StatusCode::kClockDomainMismatch,
                    "Successive synchronized scans use different clock domains");
    }
    if (scan.start_time.nanoseconds() < last_synchronized_end_time_->nanoseconds()) {
      buffer.lidar_scans_.pop_front();
      return Status(StatusCode::kTimestampRegression,
                    "LiDAR scans overlap a previously propagated interval");
    }
  }
  const auto start_after = std::upper_bound(imu.begin(), imu.end(), scan.start_time.nanoseconds(),
                                            [](std::int64_t time_ns, const ImuSample& sample) {
                                              return time_ns < sample.time.nanoseconds();
                                            });
  if (start_after == imu.begin()) {
    buffer.lidar_scans_.pop_front();
    ++stats_.rejected_missing_start_bracket;
    return Status(StatusCode::kInsufficientData, "No IMU sample at or before LiDAR scan start");
  }
  const auto start_bracket = std::prev(start_after);

  const Timestamp propagation_start_time = last_synchronized_end_time_.value_or(scan.start_time);
  const auto propagation_start_after =
      std::upper_bound(imu.begin(), imu.end(), propagation_start_time.nanoseconds(),
                       [](std::int64_t time_ns, const ImuSample& sample) {
                         return time_ns < sample.time.nanoseconds();
                       });
  if (propagation_start_after == imu.begin()) {
    buffer.lidar_scans_.pop_front();
    ++stats_.rejected_missing_start_bracket;
    return Status(StatusCode::kInsufficientData, "No IMU sample at or before propagation start");
  }
  const auto propagation_start_bracket = std::prev(propagation_start_after);

  const auto end_bracket =
      std::lower_bound(propagation_start_bracket, imu.end(), scan.end_time.nanoseconds(),
                       [](const ImuSample& sample, std::int64_t time_ns) {
                         return sample.time.nanoseconds() < time_ns;
                       });
  if (end_bracket == imu.end()) {
    ++stats_.waiting_count;
    return std::optional<MeasurementGroup>{};
  }

  std::int64_t maximum_gap_ns = 0;
  for (auto current = std::next(propagation_start_bracket); current != std::next(end_bracket);
       ++current) {
    const auto previous = std::prev(current);
    maximum_gap_ns =
        std::max(maximum_gap_ns, current->time.nanoseconds() - previous->time.nanoseconds());
  }
  if (config_.maximum_imu_gap_ns > 0 && maximum_gap_ns > config_.maximum_imu_gap_ns) {
    buffer.lidar_scans_.pop_front();
    ++stats_.rejected_imu_gap;
    return Status(StatusCode::kInsufficientData,
                  "IMU gap exceeds configured synchronization limit: observed=" +
                      std::to_string(maximum_gap_ns) + "ns limit=" +
                      std::to_string(config_.maximum_imu_gap_ns) + "ns");
  }

  MeasurementGroup group;
  group.scan = std::move(buffer.lidar_scans_.front());
  group.imu_samples.assign(propagation_start_bracket, std::next(end_bracket));
  group.propagation_start_time = propagation_start_time;
  group.has_start_bracket =
      start_bracket->time.nanoseconds() <= group.scan.start_time.nanoseconds();
  group.has_end_bracket = end_bracket->time.nanoseconds() >= group.scan.end_time.nanoseconds();
  group.max_imu_gap_ns = maximum_gap_ns;

  buffer.lidar_scans_.pop_front();
  // Keep samples on both sides of the current scan end so the next group can
  // interpolate IMU input exactly at its propagation start.
  const auto retained_begin =
      end_bracket == buffer.imu_samples_.begin() ? end_bracket : std::prev(end_bracket);
  buffer.imu_samples_.erase(buffer.imu_samples_.begin(), retained_begin);
  last_synchronized_end_time_ = group.scan.end_time;
  ++stats_.synchronized_groups;
  return std::optional<MeasurementGroup>(std::move(group));
}

const MeasurementSynchronizerStats& MeasurementSynchronizer::stats() const noexcept {
  return stats_;
}

void MeasurementSynchronizer::reset() noexcept {
  stats_ = {};
  last_synchronized_end_time_.reset();
}

}  // namespace uav::nav::lio
