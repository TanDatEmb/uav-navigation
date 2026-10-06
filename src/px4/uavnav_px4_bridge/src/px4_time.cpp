#include "uavnav/px4bridge/px4_time.hpp"

#include <expected>

namespace uavnav::px4bridge {

Result<time::Px4Time, Px4TimeError> to_px4(time::SensorTime t, ClockMode mode) {
  // Fail closed: only the explicitly supported mode passes; kRealtime and any unknown value are rejected
  // before the sign is looked at.
  if (mode != ClockMode::kSimulationIdentity) return std::unexpected(Px4TimeError::kRealtimeNotSupported);
  if (t.ns < 0) return std::unexpected(Px4TimeError::kNegative);
  return time::Px4Time{t.ns};
}

std::uint64_t to_px4_us(time::Px4Time t) {
  if (t.ns < 0) return 0;  // contract violation; saturate instead of wrapping to a huge unsigned value
  return static_cast<std::uint64_t>(t.ns) / 1000U;
}

}  // namespace uavnav::px4bridge
