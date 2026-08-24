#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace navigation_interfaces {

enum class ExecutionStateFreshnessReason : std::uint8_t {
  kValid = 0,
  kInvalidLimit,
  kMissingSourceStamp,
  kMissingReceiveStamp,
  kSourceStale,
  kSourceFuture,
  kReceiveStale,
  kReceiveFuture,
};

struct ExecutionStateFreshness {
  ExecutionStateFreshnessReason reason{ExecutionStateFreshnessReason::kInvalidLimit};
  double source_age_ms{0.0};
  double receive_age_ms{0.0};

  bool valid() const noexcept { return reason == ExecutionStateFreshnessReason::kValid; }
};

inline ExecutionStateFreshness evaluateExecutionStateFreshness(
    std::int64_t now_ros_ns, std::int64_t source_stamp_ros_ns,
    std::int64_t now_steady_ns, std::int64_t receive_stamp_steady_ns,
    double maximum_age_s) noexcept {
  ExecutionStateFreshness result;
  if (!std::isfinite(maximum_age_s) || maximum_age_s <= 0.0 ||
      maximum_age_s > static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1.0e-9) {
    return result;
  }
  if (source_stamp_ros_ns <= 0) {
    result.reason = ExecutionStateFreshnessReason::kMissingSourceStamp;
    return result;
  }
  if (receive_stamp_steady_ns <= 0) {
    result.reason = ExecutionStateFreshnessReason::kMissingReceiveStamp;
    return result;
  }
  const auto maximum_age_ns = static_cast<long double>(maximum_age_s) * 1.0e9L;
  const auto source_age_ns = static_cast<long double>(now_ros_ns) -
                             static_cast<long double>(source_stamp_ros_ns);
  const auto receive_age_ns = static_cast<long double>(now_steady_ns) -
                              static_cast<long double>(receive_stamp_steady_ns);
  result.source_age_ms = static_cast<double>(source_age_ns * 1.0e-6L);
  result.receive_age_ms = static_cast<double>(receive_age_ns * 1.0e-6L);
  if (source_age_ns > maximum_age_ns) {
    result.reason = ExecutionStateFreshnessReason::kSourceStale;
  } else if (source_age_ns < -maximum_age_ns) {
    result.reason = ExecutionStateFreshnessReason::kSourceFuture;
  } else if (receive_age_ns > maximum_age_ns) {
    result.reason = ExecutionStateFreshnessReason::kReceiveStale;
  } else if (receive_age_ns < 0) {
    result.reason = ExecutionStateFreshnessReason::kReceiveFuture;
  } else {
    result.reason = ExecutionStateFreshnessReason::kValid;
  }
  return result;
}

inline const char* executionStateFreshnessReasonName(
    ExecutionStateFreshnessReason reason) noexcept {
  switch (reason) {
    case ExecutionStateFreshnessReason::kValid: return "VALID";
    case ExecutionStateFreshnessReason::kInvalidLimit: return "INVALID_LIMIT";
    case ExecutionStateFreshnessReason::kMissingSourceStamp: return "MISSING_SOURCE_STAMP";
    case ExecutionStateFreshnessReason::kMissingReceiveStamp: return "MISSING_RECEIVE_STAMP";
    case ExecutionStateFreshnessReason::kSourceStale: return "SOURCE_STALE";
    case ExecutionStateFreshnessReason::kSourceFuture: return "SOURCE_FUTURE";
    case ExecutionStateFreshnessReason::kReceiveStale: return "RECEIVE_STALE";
    case ExecutionStateFreshnessReason::kReceiveFuture: return "RECEIVE_FUTURE";
  }
  return "UNKNOWN";
}

}  // namespace navigation_interfaces
