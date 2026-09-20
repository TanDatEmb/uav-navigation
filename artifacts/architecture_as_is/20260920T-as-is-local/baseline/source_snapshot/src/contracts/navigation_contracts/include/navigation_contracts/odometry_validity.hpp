#pragma once

#include <cstdint>

namespace navigation_contracts {

enum OdometryComponentValidity : std::uint32_t {
  kPositionValid = 1U << 0,
  kOrientationValid = 1U << 1,
  kLinearVelocityValid = 1U << 2,
  kAngularVelocityValid = 1U << 3,
};

enum OdometryCovarianceAvailability : std::uint32_t {
  kPositionCovarianceAvailable = 1U << 0,
  kOrientationCovarianceAvailable = 1U << 1,
  kLinearVelocityCovarianceAvailable = 1U << 2,
  kAngularVelocityCovarianceAvailable = 1U << 3,
};

}  // namespace navigation_contracts
