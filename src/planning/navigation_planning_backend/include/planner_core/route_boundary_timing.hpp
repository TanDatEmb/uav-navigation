#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace navigation_planning_backend {

// Return the guide time for the junction after a marked route-boundary cell.
// A direct look-ahead can expose only its final endpoint after the mission
// waypoint; split the remaining interval so the final SFC piece is not
// reduced to the generic numerical 0.01 s duration clamp.
inline double routeBoundaryJunctionTime(
    const bool outgoing_from_route_gate,
    const int nearest_guide_index,
    const std::size_t guide_sample_count,
    const int junction_index,
    const double previous_junction_time,
    const double final_guide_time,
    const double nearest_guide_time) noexcept {
  if (outgoing_from_route_gate && nearest_guide_index >= 0 &&
      static_cast<std::size_t>(nearest_guide_index + 1) == guide_sample_count &&
      junction_index > 0 && std::isfinite(previous_junction_time) &&
      std::isfinite(final_guide_time) && final_guide_time > previous_junction_time + 0.02) {
    return previous_junction_time +
        0.5 * (final_guide_time - previous_junction_time);
  }
  return nearest_guide_time;
}

// Consecutive corridor overlaps can project to the same discrete guide sample.
// A fixed tiny duration for the duplicate timestamps creates an ill-conditioned
// septic segment and does not preserve the guide's total timing. Spread only
// the repeated timestamps between their neighbouring guide-time anchors. The
// result is deterministic, strictly increasing, and introduces no flight-tuned
// minimum duration. A decreasing or degenerate guide remains invalid.
inline bool spreadRepeatedGuideJunctionTimes(
    std::vector<double>& timestamps_s) noexcept {
  if (timestamps_s.size() < 2U ||
      !std::all_of(timestamps_s.begin(), timestamps_s.end(),
                   [](const double value) { return std::isfinite(value); })) {
    return false;
  }
  for (std::size_t index = 1; index < timestamps_s.size(); ++index) {
    if (timestamps_s[index] < timestamps_s[index - 1]) return false;
  }
  if (!(timestamps_s.back() > timestamps_s.front())) return false;

  std::size_t index = 1U;
  while (index < timestamps_s.size()) {
    if (timestamps_s[index] > timestamps_s[index - 1]) {
      ++index;
      continue;
    }

    const std::size_t lower_anchor = index - 1U;
    std::size_t upper_anchor = index;
    while (upper_anchor < timestamps_s.size() &&
           !(timestamps_s[upper_anchor] > timestamps_s[lower_anchor])) {
      ++upper_anchor;
    }
    if (upper_anchor < timestamps_s.size()) {
      const double lower = timestamps_s[lower_anchor];
      const double span = timestamps_s[upper_anchor] - lower;
      const double intervals = static_cast<double>(upper_anchor - lower_anchor);
      for (std::size_t offset = 1U;
           lower_anchor + offset < upper_anchor; ++offset) {
        timestamps_s[lower_anchor + offset] =
            lower + span * static_cast<double>(offset) / intervals;
      }
      index = upper_anchor + 1U;
      continue;
    }

    // The repeated run includes the final guide timestamp. Keep that terminal
    // anchor fixed and spread the run backward from the preceding lower time.
    std::size_t first_repeated = lower_anchor;
    while (first_repeated > 0U &&
           timestamps_s[first_repeated - 1U] ==
               timestamps_s[lower_anchor]) {
      --first_repeated;
    }
    if (first_repeated == 0U) return false;
    const std::size_t previous_anchor = first_repeated - 1U;
    const std::size_t final_anchor = timestamps_s.size() - 1U;
    const double lower = timestamps_s[previous_anchor];
    const double span = timestamps_s[final_anchor] - lower;
    const double intervals = static_cast<double>(final_anchor - previous_anchor);
    if (!(span > 0.0) || !(intervals > 0.0)) return false;
    for (std::size_t offset = 1U;
         previous_anchor + offset < final_anchor; ++offset) {
      timestamps_s[previous_anchor + offset] =
          lower + span * static_cast<double>(offset) / intervals;
    }
    break;
  }

  for (std::size_t sample = 1U; sample < timestamps_s.size(); ++sample) {
    if (!(timestamps_s[sample] > timestamps_s[sample - 1U])) return false;
  }
  return true;
}

}  // namespace navigation_planning_backend
