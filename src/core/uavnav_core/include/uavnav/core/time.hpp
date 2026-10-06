#pragma once

#include <compare>
#include <cstdint>

// Clock-domain time types (SYSTEM_DESIGN §6.3, D25).
//
// Every instant is an int64 nanosecond count tagged with its clock domain, so a
// sensor stamp, a ROS stamp, a steady-clock reading and a PX4 stamp can never be
// mixed by accident. There are no implicit conversions between domains, and none
// to or from a raw integer: build a value with brace-init and read it with `.ns`.
//
// Overflow contract: arithmetic here is plain int64 arithmetic. It is never
// clamped or saturated, so a negative Duration stays negative and ordering is
// preserved. Signed overflow is UNDEFINED BEHAVIOUR, and this layer neither checks
// for it nor saturates, by design. A compiler may assume it cannot happen and fold
// away a staleness check, which is the opposite of fail-closed. Therefore code that
// builds a TimePoint or Duration from external data (ROS, PX4, sensor stamps, which
// can be garbage, uninitialised, sentinel INT64_MIN/MAX, or uint64 cast to int64) or
// from config (a large n in seconds(n) or milliseconds(n)) must range-check the raw
// value BEFORE constructing it.
namespace uavnav::time {

/// A signed span of time in nanoseconds. Domain-free.
struct Duration {
  std::int64_t ns{0};
  constexpr auto operator<=>(const Duration&) const = default;
};

constexpr Duration nanoseconds(std::int64_t n) { return Duration{n}; }
constexpr Duration milliseconds(std::int64_t n) { return Duration{n * 1'000'000}; }
constexpr Duration seconds(std::int64_t n) { return Duration{n * 1'000'000'000}; }

/// Only at calculation boundaries (filters, kinematics); never store the result.
/// Divides by 1e9 (exact), not multiplies by 1e-9 (inexact), so whole seconds round-trip.
constexpr double to_seconds(Duration d) { return static_cast<double>(d.ns) / 1e9; }

constexpr Duration operator+(Duration a, Duration b) { return Duration{a.ns + b.ns}; }
constexpr Duration operator-(Duration a, Duration b) { return Duration{a.ns - b.ns}; }

/// An instant in the clock domain named by `Tag`. Only same-Tag instants combine.
template <class Tag>
struct TimePoint {
  std::int64_t ns{0};
  constexpr auto operator<=>(const TimePoint&) const = default;

  friend constexpr Duration operator-(TimePoint a, TimePoint b) { return Duration{a.ns - b.ns}; }
  friend constexpr TimePoint operator+(TimePoint t, Duration d) { return TimePoint{t.ns + d.ns}; }
  friend constexpr TimePoint operator-(TimePoint t, Duration d) { return TimePoint{t.ns - d.ns}; }
};

struct SensorTag;
struct RosTag;
struct SteadyTag;
struct Px4Tag;

using SensorTime = TimePoint<SensorTag>;  ///< Sensor hardware / driver stamp.
using RosTime = TimePoint<RosTag>;        ///< ROS clock (may follow sim time).
using SteadyTime = TimePoint<SteadyTag>;  ///< Monotonic; for budgets and staleness.
using Px4Time = TimePoint<Px4Tag>;        ///< PX4 / flight-controller clock.

/// One snapshot of the two locally readable clocks, taken once per cycle.
struct TimeSnapshot {
  SteadyTime steady;
  RosTime ros;
};

/// Reads std::chrono::steady_clock. Monotonic non-decreasing.
SteadyTime steady_now() noexcept;

}  // namespace uavnav::time
