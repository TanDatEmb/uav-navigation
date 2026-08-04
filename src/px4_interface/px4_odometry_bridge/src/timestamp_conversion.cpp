#include "px4_odometry_bridge/timestamp_conversion.hpp"

#include <limits>

namespace px4_odometry_bridge {
namespace {

TimestampConversionResult invalid_result(const std::uint64_t generation,
                                         std::string reason) {
  TimestampConversionResult result;
  result.generation = generation;
  result.reason = std::move(reason);
  return result;
}

}  // namespace

std::optional<std::uint64_t> nanoseconds_to_microseconds(
    const std::int64_t nanoseconds) {
  if (nanoseconds <= 0) {
    return std::nullopt;
  }
  const auto value = static_cast<std::uint64_t>(nanoseconds / 1'000);
  if (value == 0 || value > std::numeric_limits<std::uint64_t>::max()) {
    return std::nullopt;
  }
  return value;
}

TimestampConversionResult TimestampConverter::convert(
    const std::int64_t measurement_time_ns,
    const std::int64_t publication_time_ns,
    const bool simulation_time_equivalence_proven,
    const std::uint64_t generation) {
  const auto fail = [this, generation](std::string reason) {
    ++diagnostics_.conversion_failure_count;
    diagnostics_.failure_reason = reason;
    return invalid_result(generation, std::move(reason));
  };
  if (!simulation_time_equivalence_proven) {
    return fail("TIME_DOMAIN_UNRESOLVED");
  }
  if (generation == 0) {
    return fail("PUBLIC_FRAME_GENERATION_INVALID");
  }
  const auto measurement_us = nanoseconds_to_microseconds(measurement_time_ns);
  const auto publication_us = nanoseconds_to_microseconds(publication_time_ns);
  if (!measurement_us || !publication_us) {
    return fail("TIMESTAMP_ZERO_OR_OVERFLOW");
  }
  std::int64_t age = 0;
  if (publication_time_ns >= measurement_time_ns) {
    age = publication_time_ns - measurement_time_ns;
  } else {
    age = -(measurement_time_ns - publication_time_ns);
  }
  if (age > maximum_age_ns_) {
    return fail("TIMESTAMP_STALE");
  }
  if (age < -maximum_age_ns_) {
    return fail("TIMESTAMP_FUTURE");
  }
  const bool generation_changed = !last_generation_.has_value() ||
                                  *last_generation_ != generation;
  if (!generation_changed &&
      ((!last_measurement_time_ns_.has_value() ||
        measurement_time_ns < *last_measurement_time_ns_) ||
       (!last_publication_time_ns_.has_value() ||
        publication_time_ns < *last_publication_time_ns_))) {
    ++diagnostics_.regression_count;
    return fail("TIMESTAMP_REGRESSION");
  }
  last_generation_ = generation;
  last_measurement_time_ns_ = measurement_time_ns;
  last_publication_time_ns_ = publication_time_ns;
  TimestampConversionResult result;
  result.valid = true;
  result.source_domain = "ROS_SIMULATION_TIME";
  result.target_domain = "PX4_SIMULATION_TIME";
  result.measurement_time_us = *measurement_us;
  result.publication_time_us = *publication_us;
  result.generation = generation;
  result.timestamp_age_ns = static_cast<std::int64_t>(age);
  result.reason = "VALID_SIMULATION_TIME_EQUIVALENCE";
  return result;
}

}  // namespace px4_odometry_bridge
