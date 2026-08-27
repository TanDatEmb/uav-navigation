#include "px4_odometry_bridge/timestamp_conversion.hpp"

#include <navigation_common/time.hpp>

#include <utility>

namespace px4_odometry_bridge {
namespace {

TimestampConversionResult invalid_result(
    const std::uint64_t timestamp_mapping_generation,
    std::string reason) {
  TimestampConversionResult result;
  result.timestamp_mapping_generation = timestamp_mapping_generation;
  result.reason = std::move(reason);
  return result;
}

}  // namespace

TimestampMappingMode timestampMappingModeFor(
    const bool use_sim_time, const std::string_view input_clock_domain) noexcept {
  if (use_sim_time && input_clock_domain == "simulation_time") {
    return TimestampMappingMode::kSimulationIdentity;
  }
  if (!use_sim_time &&
      (input_clock_domain == "ros_time" || input_clock_domain == "system_time")) {
    return TimestampMappingMode::kRealtimeTransport;
  }
  return TimestampMappingMode::kUnresolved;
}

TimestampConversionResult TimestampConverter::convert(
    const std::int64_t measurement_time_ns,
    const std::int64_t publication_time_ns,
    const TimestampMappingMode mapping_mode,
    const std::uint64_t timestamp_mapping_generation) {
  const auto fail = [this, timestamp_mapping_generation](std::string reason) {
    ++diagnostics_.conversion_failure_count;
    diagnostics_.failure_reason = reason;
    return invalid_result(timestamp_mapping_generation, std::move(reason));
  };
  if (mapping_mode == TimestampMappingMode::kUnresolved) {
    return fail("TIME_DOMAIN_UNRESOLVED");
  }
  if (timestamp_mapping_generation == 0) {
    return fail("TIMESTAMP_MAPPING_GENERATION_INVALID");
  }
  const bool mapping_generation_changed =
      !last_timestamp_mapping_generation_.has_value() ||
      *last_timestamp_mapping_generation_ != timestamp_mapping_generation;
  if (mapping_generation_changed) {
    if (last_timestamp_mapping_generation_.has_value()) {
      ++diagnostics_.timestamp_mapping_generation_change_count;
    }
    last_timestamp_mapping_generation_ = timestamp_mapping_generation;
    diagnostics_.timestamp_mapping_generation = timestamp_mapping_generation;
    last_measurement_time_us_.reset();
    last_publication_time_us_.reset();
  }
  const auto measurement_us = navigation_common::nanosecondsToMicroseconds(measurement_time_ns);
  const auto publication_us = navigation_common::nanosecondsToMicroseconds(publication_time_ns);
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
  if (!mapping_generation_changed) {
    if (last_measurement_time_us_.has_value() &&
        *measurement_us < *last_measurement_time_us_) {
      ++diagnostics_.regression_count;
      ++diagnostics_.timestamp_sample_regression_count;
      return fail("TIMESTAMP_SAMPLE_REGRESSION");
    }
    if (last_measurement_time_us_.has_value() &&
        *measurement_us == *last_measurement_time_us_) {
      ++diagnostics_.duplicate_measurement_suppressed_count;
      diagnostics_.failure_reason = "DUPLICATE_MEASUREMENT_SUPPRESSED";
      auto result = invalid_result(timestamp_mapping_generation,
                                   "DUPLICATE_MEASUREMENT_SUPPRESSED");
      result.suppressed = true;
      result.measurement_time_us = *measurement_us;
      result.publication_time_us = *publication_us;
      result.timestamp_age_ns = age;
      return result;
    }
    if (last_publication_time_us_.has_value() &&
        *publication_us < *last_publication_time_us_) {
      ++diagnostics_.regression_count;
      ++diagnostics_.publication_timestamp_regression_count;
      return fail("PUBLICATION_TIMESTAMP_REGRESSION");
    }
  }
  last_measurement_time_us_ = *measurement_us;
  last_publication_time_us_ = *publication_us;
  TimestampConversionResult result;
  result.valid = true;
  if (mapping_mode == TimestampMappingMode::kSimulationIdentity) {
    result.source_domain = "ROS_SIMULATION_TIME";
    result.target_domain = "PX4_SIMULATION_TIME";
    result.reason = "VALID_SIMULATION_TIME_EQUIVALENCE";
  } else {
    result.source_domain = "ROS_SYSTEM_TIME";
    result.target_domain = "PX4_TIME_VIA_UXRCE_TIMESYNC";
    result.reason = "VALID_REALTIME_UXRCE_MAPPING";
  }
  result.measurement_time_us = *measurement_us;
  result.publication_time_us = *publication_us;
  result.timestamp_mapping_generation = timestamp_mapping_generation;
  result.timestamp_age_ns = static_cast<std::int64_t>(age);
  return result;
}

}  // namespace px4_odometry_bridge
