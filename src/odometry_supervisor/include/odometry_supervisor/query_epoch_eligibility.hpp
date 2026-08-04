#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

// A query key includes every generation that gives an exact-time sample its
// meaning.  The key is also the unit used for duplicate suppression.
struct QueryEpochKey {
  std::int64_t epoch_ns{0};
  std::uint64_t lio_generation{0};
  std::uint64_t frame_generation{0};
  std::uint64_t time_generation{0};

  friend bool operator==(const QueryEpochKey&, const QueryEpochKey&) = default;
};

enum class QueryEpochEligibilityReason : std::uint8_t {
  kEligible = 0,
  kNoPropagatedHistory,
  kNoPx4History,
  kLioEpochAheadOfPx4,
};

struct QueryEpochEligibility {
  std::optional<OdometryState> state;
  QueryEpochEligibilityReason reason{QueryEpochEligibilityReason::kNoPropagatedHistory};
};

// Select the newest propagated epoch that PX4 can sample without
// extrapolation.  Alignment acquisition and locked comparison must use this
// same selector; neither path may query latest_propagated_ directly.
QueryEpochEligibility latest_propagated_epoch_available_to_px4(
    const std::deque<OdometryState>& propagated_history,
    const std::optional<OdometryState>& latest_px4);

QueryEpochKey query_epoch_key(const OdometryState& state, std::uint64_t lio_generation,
                              std::uint64_t frame_generation,
                              std::uint64_t time_generation);

}  // namespace odometry_supervisor
