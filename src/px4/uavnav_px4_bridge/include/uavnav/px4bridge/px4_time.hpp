#pragma once

#include <cstdint>
#include <string_view>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"

// SensorTime -> PX4 time (SYSTEM_DESIGN §4.1, D21). The beta is SITL only (§0): the sensor
// stamps already come from the simulation /clock, which is the PX4 clock, so the conversion is
// the identity. A realtime clock offset is not implemented and is rejected, never guessed.
namespace uavnav::px4bridge {

enum class ClockMode : std::uint8_t { kSimulationIdentity, kRealtime };

enum class Px4TimeError : std::uint8_t {
  kRealtimeNotSupported,  ///< ClockMode::kRealtime: no sensor->PX4 offset estimator in the beta
  kNegative               ///< the instant is before the PX4 epoch (ns < 0)
};

constexpr std::string_view to_string(Px4TimeError e) {
  switch (e) {
    case Px4TimeError::kRealtimeNotSupported: return "REALTIME_NOT_SUPPORTED";
    case Px4TimeError::kNegative: return "NEGATIVE";
  }
  return "";
}
static_assert(ReasonEnum<Px4TimeError>);

/// kSimulationIdentity: same nanoseconds, retagged as Px4Time (error kNegative if ns < 0).
/// kRealtime: always kRealtimeNotSupported, checked before the sign. Any other mode value is
/// treated like kRealtime (fail closed).
Result<time::Px4Time, Px4TimeError> to_px4(time::SensorTime t, ClockMode mode);

/// Microseconds for VehicleOdometry.timestamp_sample: ns / 1000, truncating toward zero
/// (1999 ns -> 1 us). Contract: t.ns >= 0, which to_px4 guarantees. A negative input
/// saturates to 0 instead of wrapping to a huge unsigned value.
std::uint64_t to_px4_us(time::Px4Time t);

}  // namespace uavnav::px4bridge
