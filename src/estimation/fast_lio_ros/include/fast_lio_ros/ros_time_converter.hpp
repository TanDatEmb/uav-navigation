#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

class RosTimeConverter {
 public:
  [[nodiscard]] static Timestamp fromRos(
      const builtin_interfaces::msg::Time& stamp,
      ClockDomain clock_domain = ClockDomain::kRosTime);
  [[nodiscard]] static builtin_interfaces::msg::Time toRos(Timestamp stamp);
};

}  // namespace uav::nav::lio
