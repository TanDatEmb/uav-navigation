#pragma once

#include <string_view>

namespace uav::nav::lio {

enum class ClockDomain {
  kRosTime,
  kSimulationTime,
  kSensorTime,
  kSystemTime,
  kSteadyTime,
};

[[nodiscard]] constexpr std::string_view toString(ClockDomain domain) noexcept {
  switch (domain) {
    case ClockDomain::kRosTime:
      return "ros_time";
    case ClockDomain::kSimulationTime:
      return "simulation_time";
    case ClockDomain::kSensorTime:
      return "sensor_time";
    case ClockDomain::kSystemTime:
      return "system_time";
    case ClockDomain::kSteadyTime:
      return "steady_time";
  }
  return "unknown";
}

}  // namespace uav::nav::lio
