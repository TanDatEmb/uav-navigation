#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace navigation_planning_backend {

// Retained command samples are future points relative to the immutable splice
// boundary, not relative to the first retained sample. Keep the anchor in the
// same point/time sequence before calculating any spatial horizon.
template <typename Point, typename SampleContainer, typename PointContainer>
inline bool buildAnchoredGuidePrefix(
    const Point& anchor, const double anchor_time_s,
    const SampleContainer& samples, PointContainer& points,
    std::vector<double>& elapsed_s) {
  points.clear();
  elapsed_s.clear();
  if (!anchor.allFinite() || !std::isfinite(anchor_time_s)) return false;
  double previous_time_s = anchor_time_s;
  for (const auto& sample : samples) {
    if (!sample.second.allFinite() || !std::isfinite(sample.first) ||
        !(sample.first > previous_time_s) ||
        !std::isfinite(sample.first - anchor_time_s)) return false;
    previous_time_s = sample.first;
  }
  points.emplace_back(anchor);
  elapsed_s.emplace_back(0.0);
  for (const auto& sample : samples) {
    points.emplace_back(sample.second);
    elapsed_s.emplace_back(sample.first - anchor_time_s);
  }
  return true;
}

// Select an entry on the ordered incoming guide, rather than extrapolating
// behind its final (possibly very short) edge. Interpolate point and time on
// the same edge; a zero-length prefix retains the immutable anchor alone.
template <typename PointContainer>
inline bool truncateTimedGuideAtDistance(
    const PointContainer& points, const std::vector<double>& elapsed_s,
    const double maximum_length_m, PointContainer& prefix,
    std::vector<double>& prefix_elapsed_s) {
  prefix.clear();
  prefix_elapsed_s.clear();
  if (points.empty() || points.size() != elapsed_s.size() ||
      !std::isfinite(maximum_length_m) || maximum_length_m < 0.0 ||
      !std::isfinite(elapsed_s.front()) || elapsed_s.front() != 0.0) return false;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (!points[index].allFinite() || !std::isfinite(elapsed_s[index]) ||
        (index > 0U && !(elapsed_s[index] > elapsed_s[index - 1U]))) return false;
  }
  prefix.emplace_back(points.front());
  prefix_elapsed_s.emplace_back(elapsed_s.front());
  double remaining_m = maximum_length_m;
  for (std::size_t index = 1U; index < points.size() && remaining_m > 0.0; ++index) {
    const double length_m = (points[index] - points[index - 1U]).norm();
    if (!std::isfinite(length_m)) {
      prefix.clear();
      prefix_elapsed_s.clear();
      return false;
    }
    if (length_m <= remaining_m) {
      prefix.emplace_back(points[index]);
      prefix_elapsed_s.emplace_back(elapsed_s[index]);
      remaining_m -= length_m;
      continue;
    }
    const double fraction = remaining_m / length_m;
    prefix.emplace_back(points[index - 1U] +
                        fraction * (points[index] - points[index - 1U]));
    prefix_elapsed_s.emplace_back(elapsed_s[index - 1U] +
        fraction * (elapsed_s[index] - elapsed_s[index - 1U]));
    break;
  }
  return true;
}

// Project onto the portion of the ordered guide inside a convex overlap.
// Clip each chord by its halfspaces, then project within the clipped interval.
// The guide coordinate (edge index + fraction), position and time describe
// the same point. Discrete sample lookup cannot represent a narrow overlap
// between two samples and must not supply time for an unrelated interior.
template <typename PointContainer, typename PlaneMatrix, typename Point>
inline bool projectOrderedGuideIntoOverlap(
    const PointContainer& guide, const std::vector<double>& elapsed_s,
    const PlaneMatrix& planes, const Point& target,
    const double minimum_coordinate, const double maximum_coordinate,
    Point& point, double& time_s, double& coordinate) {
  if (guide.size() < 2U || guide.size() != elapsed_s.size() ||
      planes.rows() == 0 || planes.cols() != 4 || !planes.allFinite() ||
      !target.allFinite() || !std::isfinite(minimum_coordinate) ||
      !std::isfinite(maximum_coordinate) || minimum_coordinate < 0.0 ||
      maximum_coordinate < minimum_coordinate ||
      maximum_coordinate > static_cast<double>(guide.size() - 1U)) return false;
  for (std::size_t index = 0U; index < guide.size(); ++index) {
    if (!guide[index].allFinite() || !std::isfinite(elapsed_s[index]) ||
        (index > 0U && !(elapsed_s[index] > elapsed_s[index - 1U]))) return false;
  }
  double best_distance_squared = std::numeric_limits<double>::infinity();
  Point best_point = target;
  double best_time = 0.0, best_coordinate = 0.0;
  const auto first_edge = std::min(
      static_cast<std::size_t>(std::floor(minimum_coordinate)), guide.size() - 2U);
  for (std::size_t edge = first_edge; edge + 1U < guide.size(); ++edge) {
    if (static_cast<double>(edge) > maximum_coordinate) break;
    double lower = std::max(0.0, minimum_coordinate - static_cast<double>(edge));
    double upper = std::min(1.0, maximum_coordinate - static_cast<double>(edge));
    const Point direction = guide[edge + 1U] - guide[edge];
    const double length_squared = direction.squaredNorm();
    if (!direction.allFinite() || !std::isfinite(length_squared)) return false;
    for (decltype(planes.rows()) row = 0; row < planes.rows() && lower <= upper; ++row) {
      const double value = planes(row, 0) * guide[edge].x() +
          planes(row, 1) * guide[edge].y() + planes(row, 2) * guide[edge].z() +
          planes(row, 3);
      const double slope = planes(row, 0) * direction.x() +
          planes(row, 1) * direction.y() + planes(row, 2) * direction.z();
      if (!std::isfinite(value) || !std::isfinite(slope)) return false;
      if (slope == 0.0) {
        if (value > 0.0) upper = -1.0;
      } else if (slope > 0.0) {
        upper = std::min(upper, -value / slope);
      } else {
        lower = std::max(lower, -value / slope);
      }
    }
    if (lower > upper) continue;
    double fraction = lower;
    if (length_squared > 0.0) {
      const double projection = (target - guide[edge]).dot(direction) / length_squared;
      if (!std::isfinite(projection)) return false;
      fraction = std::clamp(projection, lower, upper);
    }
    const Point candidate = guide[edge] + fraction * direction;
    const double distance_squared = (candidate - target).squaredNorm();
    const double candidate_time = elapsed_s[edge] + fraction *
        (elapsed_s[edge + 1U] - elapsed_s[edge]);
    if (!candidate.allFinite() || !std::isfinite(distance_squared) ||
        !std::isfinite(candidate_time)) return false;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_point = candidate;
      best_time = candidate_time;
      best_coordinate = static_cast<double>(edge) + fraction;
    }
  }
  if (!std::isfinite(best_distance_squared)) return false;
  point = best_point;
  time_s = best_time;
  coordinate = best_coordinate;
  return true;
}

// Use the geometric route-boundary point as the temporal anchor.  An overlap
// interior can be closer to an unrelated early guide sample than the point
// that the boundary contract actually requires the trajectory to reach.
template <typename PointContainer, typename Point>
inline int nearestGuideSampleIndex(
    const PointContainer& guide_path,
    const Point& boundary_point,
    const std::size_t begin_index,
    const std::size_t end_index) noexcept {
  if (guide_path.empty() || !boundary_point.allFinite() ||
      begin_index >= guide_path.size() || begin_index >= end_index) {
    return -1;
  }
  const std::size_t bounded_end = std::min(end_index, guide_path.size());
  int nearest_index = -1;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = begin_index; index < bounded_end; ++index) {
    if (!guide_path[index].allFinite()) continue;
    const double distance =
        (guide_path[index] - boundary_point).norm();
    if (std::isfinite(distance) && distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = static_cast<int>(index);
    }
  }
  return nearest_index;
}

template <typename PointContainer, typename Point>
inline int nearestGuideSampleIndex(
    const PointContainer& guide_path,
    const Point& boundary_point) noexcept {
  return nearestGuideSampleIndex(
      guide_path, boundary_point, 0U, guide_path.size());
}

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
