#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

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
};

class MeasurementSynchronizer {
 public:
  explicit MeasurementSynchronizer(MeasurementSynchronizerConfig config = {});

  // A successful empty optional means more data is required. An error means
  // the front scan was permanently invalid and has been removed.
  [[nodiscard]] Result<std::optional<MeasurementGroup>> synchronizeNext(MeasurementBuffer& buffer);

  [[nodiscard]] const MeasurementSynchronizerStats& stats() const noexcept;
  void reset() noexcept;

 private:
  MeasurementSynchronizerConfig config_;
  MeasurementSynchronizerStats stats_;
  std::optional<Timestamp> last_synchronized_end_time_;
};

}  // namespace uav::nav::lio
