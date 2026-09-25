#pragma once

#include <cstdint>

#include <navigation_execution/timestamp_freshness.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_runtime {

// A value assessment of the source-time evidence carried by an immutable
// world snapshot. This is deliberately not authority state and does not
// conflate snapshot identity with publication/receive freshness.
enum class WorldTemporalReason : std::uint8_t {
  kCurrent = 0,
  kNoSnapshot,
  kInvalidTimeContract,
  kSourceStampMissing,
  kSourceStale,
  kSourceFuture,
};

struct WorldTemporalAssessment {
  navigation_world_model::WorldSnapshotIdentity identity{};
  WorldTemporalReason reason{WorldTemporalReason::kNoSnapshot};
  navigation_execution::TimestampFreshness source_time_result{
      navigation_execution::TimestampFreshness::INVALID};

  [[nodiscard]] bool sourceCurrent() const noexcept {
    return reason == WorldTemporalReason::kCurrent;
  }

  [[nodiscard]] navigation_execution::TimestampFreshness sourceTimeResult()
      const noexcept {
    return source_time_result;
  }
};

[[nodiscard]] inline WorldTemporalAssessment assessWorldTemporal(
    const bool snapshot_available,
    const navigation_world_model::WorldSnapshotIdentity& identity,
    const std::int64_t now_ros_ns,
    const std::int64_t maximum_source_age_ns) noexcept {
  WorldTemporalAssessment result{identity, WorldTemporalReason::kNoSnapshot};
  if (!snapshot_available) return result;
  if (maximum_source_age_ns <= 0 || now_ros_ns <= 0) {
    result.reason = WorldTemporalReason::kInvalidTimeContract;
    return result;
  }
  if (identity.observation_stamp_ns <= 0) {
    result.reason = WorldTemporalReason::kSourceStampMissing;
    return result;
  }
  switch (navigation_execution::classifyTimestampFreshness(
      now_ros_ns, identity.observation_stamp_ns, maximum_source_age_ns)) {
    case navigation_execution::TimestampFreshness::VALID:
      result.reason = WorldTemporalReason::kCurrent;
      result.source_time_result = navigation_execution::TimestampFreshness::VALID;
      break;
    case navigation_execution::TimestampFreshness::STALE:
      result.reason = WorldTemporalReason::kSourceStale;
      result.source_time_result = navigation_execution::TimestampFreshness::STALE;
      break;
    case navigation_execution::TimestampFreshness::FUTURE:
      result.reason = WorldTemporalReason::kSourceFuture;
      result.source_time_result = navigation_execution::TimestampFreshness::FUTURE;
      break;
    case navigation_execution::TimestampFreshness::INVALID:
      result.reason = WorldTemporalReason::kInvalidTimeContract;
      result.source_time_result = navigation_execution::TimestampFreshness::INVALID;
      break;
  }
  return result;
}

}  // namespace navigation_runtime
