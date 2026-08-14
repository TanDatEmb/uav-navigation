#pragma once

#include <cstddef>
#include <cstdint>

#include "fast_lio_core/sensor/lidar_scan.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

enum class DeskewStatus {
  kApplied,
  kBypassedSimultaneousScan,
};

struct DeskewResult {
  LidarScan scan;
  DeskewStatus status{DeskewStatus::kBypassedSimultaneousScan};
  Timestamp reference_time;
  bool deskew_applied{false};
  std::uint32_t point_time_min_ns{0};
  std::uint32_t point_time_max_ns{0};
  std::size_t interpolation_failure_count{0};
};

}  // namespace uav::nav::lio
