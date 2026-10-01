#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <string>
#include <utility>

namespace uav::nav::lio {

Result<SynchronizationResult> MeasurementSynchronizer::synchronizeNext(
    MeasurementBuffer& buffer) {
  std::scoped_lock lock(buffer.mutex_);
  if (buffer.lidar_scans_.empty() || buffer.imu_samples_.empty()) {
    ++stats_.waiting_count;
    return SynchronizationResult{};
  }

  const LidarScan& scan = buffer.lidar_scans_.front();
  const Status scan_status = scan.validate();
  if (!scan_status.ok()) {
    buffer.lidar_scans_.pop_front();
    return scan_status;
  }
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
      const std::int64_t previous_end_ns =
          last_synchronized_end_time_->nanoseconds();
      const std::int64_t scan_start_ns = scan.start_time.nanoseconds();
      const std::int64_t scan_end_ns = scan.end_time.nanoseconds();
      const auto overlap = checkedDifference(*last_synchronized_end_time_,
                                             scan.start_time);
      if (!overlap.ok()) {
        buffer.lidar_scans_.pop_front();
        return overlap.status();
      }
      buffer.lidar_scans_.pop_front();
      ++stats_.rejected_scan_overlap;
      const std::size_t rejected_index = scan_index_++;
      return Status(
          StatusCode::kOverlappingLidarInterval,
          "OVERLAPPING_LIDAR_INTERVAL previous_synchronized_end_ns=" +
              std::to_string(previous_end_ns) +
              " current_scan_start_ns=" + std::to_string(scan_start_ns) +
              " current_scan_end_ns=" + std::to_string(scan_end_ns) +
              " overlap_duration_ns=" +
              std::to_string(overlap.value().nanoseconds()) +
              " scan_index=" + std::to_string(rejected_index));
    }
  }
  const auto start_after = std::upper_bound(imu.begin(), imu.end(), scan.start_time.nanoseconds(),
                                            [](std::int64_t time_ns, const ImuSample& sample) {
                                              return time_ns < sample.time.nanoseconds();
                                            });
  if (start_after == imu.begin()) {
    buffer.lidar_scans_.pop_front();
    ++stats_.rejected_missing_start_bracket;
    return Status(StatusCode::kMissingStartBracket,
                  "No IMU sample at or before LiDAR scan start");
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
    return Status(StatusCode::kMissingStartBracket,
                  "No IMU sample at or before propagation start");
  }
  const auto propagation_start_bracket = std::prev(propagation_start_after);

  const auto end_bracket =
      std::lower_bound(propagation_start_bracket, imu.end(), scan.end_time.nanoseconds(),
                       [](const ImuSample& sample, std::int64_t time_ns) {
                         return sample.time.nanoseconds() < time_ns;
                       });
  if (end_bracket == imu.end()) {
    ++stats_.waiting_count;
    return SynchronizationResult{};
  }

  std::int64_t maximum_gap_ns = 0;
  std::int64_t maximum_gap_previous_ns = 0;
  std::int64_t maximum_gap_current_ns = 0;
  for (auto current = std::next(propagation_start_bracket); current != std::next(end_bracket);
       ++current) {
    const auto previous = std::prev(current);
    const auto gap = checkedDifference(current->time, previous->time);
    if (!gap.ok()) {
      buffer.lidar_scans_.pop_front();
      return gap.status();
    }
    const std::int64_t gap_ns = gap.value().nanoseconds();
    if (gap_ns > maximum_gap_ns) {
      maximum_gap_ns = gap_ns;
      maximum_gap_previous_ns = previous->time.nanoseconds();
      maximum_gap_current_ns = current->time.nanoseconds();
    }
  }
  if (maximum_gap_ns > config_.maximum_imu_gap_ns) {
    const Timestamp scan_start = scan.start_time;
    const Timestamp scan_end = scan.end_time;
    const Timestamp gap_begin(maximum_gap_previous_ns,
                              scan.start_time.clock_domain());
    const Timestamp gap_end(maximum_gap_current_ns,
                            scan.start_time.clock_domain());
    buffer.lidar_scans_.pop_front();
    ++stats_.rejected_imu_gap;
    const auto resume = std::lower_bound(
        buffer.imu_samples_.begin(), buffer.imu_samples_.end(),
        maximum_gap_current_ns,
        [](const ImuSample& sample, std::int64_t time_ns) {
          return sample.time.nanoseconds() < time_ns;
        });
    buffer.imu_samples_.erase(buffer.imu_samples_.begin(), resume);
    last_synchronized_end_time_ = gap_end;
    ++scan_index_;
    SynchronizationResult result;
    result.discontinuity = PropagationDiscontinuity{
        SynchronizationFailureKind::kImuDiscontinuity,
        gap_begin,
        gap_end,
        maximum_gap_ns,
        scan_start,
        scan_end,
        gap_end};
    return result;
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
  ++scan_index_;
  ++stats_.synchronized_groups;
  SynchronizationResult result;
  result.measurement_group = std::move(group);
  return result;
}

const MeasurementSynchronizerStats& MeasurementSynchronizer::stats() const noexcept {
  return stats_;
}

const std::optional<Timestamp>& MeasurementSynchronizer::epoch() const noexcept {
  return last_synchronized_end_time_;
}

void MeasurementSynchronizer::reset() noexcept {
  stats_ = {};
  last_synchronized_end_time_.reset();
  scan_index_ = 0;
}

}  // namespace uav::nav::lio
