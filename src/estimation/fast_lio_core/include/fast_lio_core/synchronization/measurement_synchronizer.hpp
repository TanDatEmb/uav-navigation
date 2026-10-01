#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/sensor/measurement_group.hpp"
#include "fast_lio_core/synchronization/measurement_buffer.hpp"

namespace uav::nav::lio {

struct MeasurementSynchronizerConfig {
  std::int64_t maximum_imu_gap_ns{20'000'000};
};

struct MeasurementSynchronizerStats {
  std::size_t synchronized_groups{0};
  std::size_t waiting_count{0};
  std::size_t rejected_missing_start_bracket{0};
  std::size_t rejected_imu_gap{0};
  std::size_t rejected_clock_domain{0};
  std::size_t rejected_scan_overlap{0};
};

enum class SynchronizationFailureKind {
  kNone,
  kImuDiscontinuity,
};

struct PropagationDiscontinuity {
  SynchronizationFailureKind failure_kind{
      SynchronizationFailureKind::kImuDiscontinuity};
  Timestamp gap_begin;
  Timestamp gap_end;
  std::int64_t gap_duration_ns{0};
  Timestamp scan_start;
  Timestamp scan_end;
  Timestamp resume_time;
};

struct SynchronizationResult {
  std::optional<MeasurementGroup> measurement_group;
  std::optional<PropagationDiscontinuity> discontinuity;

  [[nodiscard]] bool has_value() const noexcept {
    return measurement_group.has_value();
  }
  [[nodiscard]] MeasurementGroup* operator->() noexcept {
    return &measurement_group.value();
  }
  [[nodiscard]] const MeasurementGroup* operator->() const noexcept {
    return &measurement_group.value();
  }
  [[nodiscard]] MeasurementGroup& operator*() {
    return measurement_group.value();
  }
  [[nodiscard]] const MeasurementGroup& operator*() const {
    return measurement_group.value();
  }
};

class MeasurementSynchronizer {
 public:
  explicit MeasurementSynchronizer(MeasurementSynchronizerConfig config = {})
      : config_(config) {
    if (config_.maximum_imu_gap_ns <= 0) {
      throw std::invalid_argument("maximum IMU gap must be positive");
    }
  }

  // A successful empty optional means more data is required. An error means
  // the front scan was permanently invalid and has been removed.
  [[nodiscard]] Result<SynchronizationResult> synchronizeNext(
      MeasurementBuffer& buffer);

  [[nodiscard]] const MeasurementSynchronizerStats& stats() const noexcept;
  [[nodiscard]] const std::optional<Timestamp>& epoch() const noexcept;
  void reset() noexcept;

 private:
  MeasurementSynchronizerConfig config_;
  MeasurementSynchronizerStats stats_;
  std::optional<Timestamp> last_synchronized_end_time_;
  std::size_t scan_index_{0};
};

}  // namespace uav::nav::lio
