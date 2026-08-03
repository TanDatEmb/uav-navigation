#pragma once

#include <optional>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

class ResidualCalculator {
 public:
  static bool valid(const OdometryState& state);
  static std::optional<Residual> compare(
      const OdometryState& lio, const OdometryState& px4,
      const std::optional<Residual>& previous = std::nullopt);

 private:
  static double wrap_to_pi(double angle);
};

}  // namespace odometry_supervisor
