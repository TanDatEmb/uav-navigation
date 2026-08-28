#include "navigation_mission/route_progress.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace navigation_mission {
namespace {

constexpr double kMinimumSegmentLengthM = 1.0e-9;
constexpr double kProjectionDistanceTieSquaredM2 = 1.0e-12;

double clampArc(const double arc_length_m, const double total_length_m) {
  if (!std::isfinite(arc_length_m)) return 0.0;
  return std::clamp(arc_length_m, 0.0, total_length_m);
}

std::optional<double> segmentDistanceToPoint(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& point) noexcept {
  if (!start.allFinite() || !end.allFinite() || !point.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Vector3d delta = end - start;
  const double length_squared = delta.squaredNorm();
  if (!std::isfinite(length_squared)) return std::nullopt;
  const double fraction = length_squared > 1.0e-12
      ? std::clamp((point - start).dot(delta) / length_squared, 0.0, 1.0)
      : 0.0;
  const double distance = (start + fraction * delta - point).norm();
  return std::isfinite(distance) ? std::optional<double>{distance} : std::nullopt;
}

}  // namespace

bool ImmutableRouteSnapshot::valid() const noexcept {
  if (mission_id.empty() || frame.empty() || route_revision == 0U ||
      request_id == 0U || waypoints.empty() ||
      active_waypoint_index >= waypoints.size() ||
      waypoint_arc_lengths_m.size() != waypoints.size() ||
      !std::isfinite(total_length_m) || total_length_m < 0.0 ||
      !measured_progress.valid ||
      !measured_progress.projection.valid ||
      !std::isfinite(measured_progress.progress_arc_m) ||
      !std::isfinite(measured_progress.projection.arc_length_m) ||
      !std::isfinite(measured_progress.projection.lateral_error_m) ||
      !measured_progress.projection.point.allFinite() ||
      !measured_progress.projection.tangent.allFinite() ||
      measured_progress.progress_arc_m < 0.0 ||
      measured_progress.progress_arc_m > total_length_m + 1.0e-6 ||
      measured_progress.projection.arc_length_m < 0.0 ||
      measured_progress.projection.arc_length_m > total_length_m + 1.0e-6 ||
      measured_progress.projection.lateral_error_m < 0.0) {
    return false;
  }
  std::set<std::string> waypoint_ids;
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    const auto& waypoint = waypoints[index];
    if (waypoint.id.empty() || !waypoint.position_enu.allFinite() ||
        !std::isfinite(waypoint.acceptance_radius_m) ||
        waypoint.acceptance_radius_m <= 0.0 ||
        !std::isfinite(waypoint_arc_lengths_m[index])) {
      return false;
    }
    if (!waypoint_ids.insert(waypoint.id).second) return false;
    if (index > 0U && waypoint_arc_lengths_m[index] + 1.0e-9 <
                          waypoint_arc_lengths_m[index - 1U]) {
      return false;
    }
  }
  for (const auto& segment : segments) {
    if (segment.start_waypoint_index >= waypoints.size() ||
        segment.end_waypoint_index >= waypoints.size() ||
        segment.start_waypoint_index >= segment.end_waypoint_index ||
        !segment.start.allFinite() || !segment.end.allFinite() ||
        !segment.tangent.allFinite() || !std::isfinite(segment.start_arc_m) ||
        !std::isfinite(segment.end_arc_m) || !std::isfinite(segment.length_m) ||
        segment.length_m <= 0.0 || segment.end_arc_m <= segment.start_arc_m) {
      return false;
    }
    if ((segment.start - waypoints[segment.start_waypoint_index].position_enu).norm() >
            1.0e-6 ||
        (segment.end - waypoints[segment.end_waypoint_index].position_enu).norm() >
            1.0e-6 ||
        std::abs(segment.start_arc_m -
                 waypoint_arc_lengths_m[segment.start_waypoint_index]) > 1.0e-6 ||
        std::abs(segment.end_arc_m -
                 waypoint_arc_lengths_m[segment.end_waypoint_index]) > 1.0e-6) {
      return false;
    }
  }
  if (std::abs(waypoint_arc_lengths_m.back() - total_length_m) > 1.0e-6) {
    return false;
  }
  if (segments.empty()) {
    return measured_progress.projection.segment_index ==
           std::numeric_limits<std::size_t>::max();
  }
  if (measured_progress.projection.segment_index >= segments.size()) return false;
  const auto& measured_segment =
      segments[measured_progress.projection.segment_index];
  if (measured_progress.projection.arc_length_m + 1.0e-6 <
          measured_segment.start_arc_m ||
      measured_progress.projection.arc_length_m >
          measured_segment.end_arc_m + 1.0e-6) {
    return false;
  }
  return true;
}

std::optional<Eigen::Vector3d> ImmutableRouteSnapshot::pointAtArc(
    const double arc_length_m) const noexcept {
  if (!valid() || !std::isfinite(arc_length_m)) return std::nullopt;
  if (segments.empty()) return waypoints.front().position_enu;
  const double bounded_arc = std::clamp(arc_length_m, 0.0, total_length_m);
  const auto iterator = std::lower_bound(
      segments.begin(), segments.end(), bounded_arc,
      [](const RouteSegment& segment, const double arc) {
        return segment.end_arc_m < arc;
      });
  const auto& segment = iterator == segments.end() ? segments.back() : *iterator;
  const double fraction = std::clamp(
      (bounded_arc - segment.start_arc_m) / segment.length_m, 0.0, 1.0);
  return segment.start + fraction * (segment.end - segment.start);
}

std::optional<Eigen::Vector3d> ImmutableRouteSnapshot::routeLookaheadPoint(
    const double lookahead_m) const noexcept {
  if (!valid() || !std::isfinite(lookahead_m) || lookahead_m < 0.0) {
    return std::nullopt;
  }
  return pointAtArc(measured_progress.progress_arc_m + lookahead_m);
}

RouteProgress::RouteProgress(const Mission& mission, RouteProgressConfig config)
    : waypoints_(mission.waypoints), config_(config) {
  if (waypoints_.empty()) {
    throw std::invalid_argument("route progress requires at least one waypoint");
  }
  if (!std::isfinite(config_.backtrack_tolerance_m) ||
      config_.backtrack_tolerance_m < 0.0) {
    throw std::invalid_argument("route backtrack tolerance must be finite and non-negative");
  }

  waypoint_arc_lengths_.assign(waypoints_.size(), 0.0);
  for (std::size_t index = 0U; index < waypoints_.size(); ++index) {
    const auto& waypoint = waypoints_[index];
    if (!waypoint.position_enu.allFinite() ||
        !std::isfinite(waypoint.acceptance_radius_m) ||
        waypoint.acceptance_radius_m <= 0.0) {
      throw std::invalid_argument("route waypoint geometry is invalid");
    }
    if (index == 0U) continue;
    const Eigen::Vector3d delta = waypoint.position_enu -
                                  waypoints_[index - 1U].position_enu;
    const double length_m = delta.norm();
    if (!std::isfinite(length_m)) {
      throw std::invalid_argument("route waypoint segment is non-finite");
    }
    if (length_m > kMinimumSegmentLengthM) {
      RouteSegment segment;
      segment.start_waypoint_index = index - 1U;
      segment.end_waypoint_index = index;
      segment.start_arc_m = total_length_m_;
      segment.length_m = length_m;
      segment.end_arc_m = total_length_m_ + length_m;
      segment.start = waypoints_[index - 1U].position_enu;
      segment.end = waypoint.position_enu;
      segment.tangent = delta / length_m;
      segments_.push_back(segment);
      total_length_m_ = segment.end_arc_m;
    }
    waypoint_arc_lengths_[index] = total_length_m_;
  }
}

void RouteProgress::reset() noexcept { state_ = {}; }

RouteProjection RouteProgress::project(const Eigen::Vector3d& position) const noexcept {
  RouteProjection result;
  if (!position.allFinite() || waypoints_.empty()) return result;

  if (segments_.empty()) {
    const double distance_m = (position - waypoints_.front().position_enu).norm();
    if (!std::isfinite(distance_m)) return result;
    result.valid = true;
    result.arc_length_m = 0.0;
    result.lateral_error_m = distance_m;
    result.point = waypoints_.front().position_enu;
    return result;
  }

  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < segments_.size(); ++index) {
    const auto& segment = segments_[index];
    const Eigen::Vector3d delta = position - segment.start;
    const double raw_fraction = delta.dot(segment.tangent) / segment.length_m;
    if (!std::isfinite(raw_fraction)) continue;
    const double fraction = std::clamp(raw_fraction, 0.0, 1.0);
    const Eigen::Vector3d projected = segment.start + fraction *
                                      (segment.end - segment.start);
    const double distance_squared = (position - projected).squaredNorm();
    if (!std::isfinite(distance_squared) || distance_squared >= best_distance_squared) {
      continue;
    }
    best_distance_squared = distance_squared;
    result.valid = true;
    result.segment_index = index;
    result.arc_length_m = segment.start_arc_m + fraction * segment.length_m;
    result.lateral_error_m = std::sqrt(distance_squared);
    result.segment_fraction = fraction;
    result.point = projected;
    result.tangent = segment.tangent;
  }
  return result;
}

RouteProgressState RouteProgress::update(const Eigen::Vector3d& position) noexcept {
  auto projection = project(position);
  if (!projection.valid) return state_;
  const bool was_valid = state_.valid;
  const double previous_progress = state_.progress_arc_m;
  if (was_valid && !segments_.empty()) {
    // A route may overlap itself, especially at a 180-degree reversal. Pure
    // nearest-point projection cannot identify the active branch there. Among
    // geometrically tied candidates, select the arc closest to prior monotonic
    // progress and prefer the forward branch on an exact arc-distance tie.
    const double best_distance_squared = projection.lateral_error_m *
                                         projection.lateral_error_m;
    double best_arc_distance = std::abs(projection.arc_length_m - previous_progress);
    for (std::size_t index = 0U; index < segments_.size(); ++index) {
      const auto& segment = segments_[index];
      const Eigen::Vector3d delta = position - segment.start;
      const double raw_fraction = delta.dot(segment.tangent) / segment.length_m;
      if (!std::isfinite(raw_fraction)) continue;
      const double fraction = std::clamp(raw_fraction, 0.0, 1.0);
      const Eigen::Vector3d point = segment.start + fraction *
                                    (segment.end - segment.start);
      const double distance_squared = (position - point).squaredNorm();
      if (!std::isfinite(distance_squared) ||
          std::abs(distance_squared - best_distance_squared) >
              kProjectionDistanceTieSquaredM2) {
        continue;
      }
      const double arc = segment.start_arc_m + fraction * segment.length_m;
      const double arc_distance = std::abs(arc - previous_progress);
      const bool closer_to_progress = arc_distance + 1.0e-12 < best_arc_distance;
      const bool forward_on_tie = std::abs(arc_distance - best_arc_distance) <= 1.0e-12 &&
                                  arc > projection.arc_length_m;
      if (!closer_to_progress && !forward_on_tie) continue;
      projection.valid = true;
      projection.segment_index = index;
      projection.arc_length_m = arc;
      projection.lateral_error_m = std::sqrt(distance_squared);
      projection.segment_fraction = fraction;
      projection.point = point;
      projection.tangent = segment.tangent;
      best_arc_distance = arc_distance;
    }
  }
  state_.valid = true;
  state_.backtracking_exceeded = was_valid &&
      projection.arc_length_m + config_.backtrack_tolerance_m < previous_progress;
  state_.projection = projection;
  state_.progress_arc_m = was_valid
      ? std::max(previous_progress, projection.arc_length_m)
      : projection.arc_length_m;
  return state_;
}

double RouteProgress::waypointArcLengthM(const std::size_t waypoint_index) const {
  if (waypoint_index >= waypoint_arc_lengths_.size()) {
    throw std::out_of_range("route waypoint index is out of range");
  }
  return waypoint_arc_lengths_[waypoint_index];
}

std::size_t RouteProgress::segmentForArc(const double arc_length_m) const noexcept {
  if (segments_.empty()) return std::numeric_limits<std::size_t>::max();
  const double clamped_arc = clampArc(arc_length_m, total_length_m_);
  const auto iterator = std::lower_bound(
      segments_.begin(), segments_.end(), clamped_arc,
      [](const RouteSegment& segment, const double value) {
        return segment.end_arc_m < value;
      });
  if (iterator == segments_.end()) return segments_.size() - 1U;
  return static_cast<std::size_t>(std::distance(segments_.begin(), iterator));
}

std::optional<Eigen::Vector3d> RouteProgress::pointAtArc(
    const double arc_length_m) const noexcept {
  if (waypoints_.empty()) return std::nullopt;
  if (segments_.empty()) return waypoints_.front().position_enu;
  const std::size_t index = segmentForArc(arc_length_m);
  if (index >= segments_.size()) return std::nullopt;
  const auto& segment = segments_[index];
  const double fraction = (clampArc(arc_length_m, total_length_m_) -
                           segment.start_arc_m) / segment.length_m;
  return segment.start + std::clamp(fraction, 0.0, 1.0) *
      (segment.end - segment.start);
}

double RouteProgress::altitudeAtArc(const double arc_length_m) const noexcept {
  if (waypoints_.empty() || segments_.empty()) {
    return waypoints_.empty() ? std::numeric_limits<double>::quiet_NaN()
                              : waypoints_.front().position_enu.z();
  }
  const std::size_t index = segmentForArc(arc_length_m);
  if (index >= segments_.size()) return std::numeric_limits<double>::quiet_NaN();
  const auto& segment = segments_[index];
  const double fraction = (clampArc(arc_length_m, total_length_m_) -
                           segment.start_arc_m) / segment.length_m;
  return segment.start.z() + std::clamp(fraction, 0.0, 1.0) *
      (segment.end.z() - segment.start.z());
}

std::optional<Eigen::Vector3d> RouteProgress::incomingTangent(
    const std::size_t waypoint_index) const noexcept {
  if (waypoint_index >= waypoints_.size()) return std::nullopt;
  for (auto iterator = segments_.rbegin(); iterator != segments_.rend(); ++iterator) {
    if (iterator->end_waypoint_index <= waypoint_index) return iterator->tangent;
  }
  return std::nullopt;
}

std::optional<Eigen::Vector3d> RouteProgress::outgoingTangent(
    const std::size_t waypoint_index) const noexcept {
  if (waypoint_index >= waypoints_.size()) return std::nullopt;
  for (const auto& segment : segments_) {
    if (segment.start_waypoint_index >= waypoint_index) return segment.tangent;
  }
  return std::nullopt;
}

bool RouteProgress::insideAcceptance(const std::size_t waypoint_index,
                                     const Eigen::Vector3d& position) const noexcept {
  if (waypoint_index >= waypoints_.size() || !position.allFinite()) return false;
  const double distance_m = (position - waypoints_[waypoint_index].position_enu).norm();
  return std::isfinite(distance_m) &&
         distance_m <= waypoints_[waypoint_index].acceptance_radius_m;
}

std::optional<double> RouteProgress::measuredWaypointCrossingError(
    const std::size_t waypoint_index, const Eigen::Vector3d& current_position,
    const std::optional<Eigen::Vector3d>& previous_position,
    const double sample_gap_s, const double maximum_sample_gap_s) const noexcept {
  if (waypoint_index >= waypoints_.size() || !current_position.allFinite()) {
    return std::nullopt;
  }
  const auto& waypoint = waypoints_[waypoint_index];
  const double current_error = (current_position - waypoint.position_enu).norm();
  if (std::isfinite(current_error) && current_error <= waypoint.acceptance_radius_m) {
    return current_error;
  }
  if (!previous_position.has_value() || !previous_position->allFinite() ||
      !std::isfinite(sample_gap_s) || !std::isfinite(maximum_sample_gap_s) ||
      sample_gap_s < 0.0 || maximum_sample_gap_s < 0.0 ||
      sample_gap_s > maximum_sample_gap_s) {
    return std::nullopt;
  }
  const auto incoming = incomingTangent(waypoint_index);
  if (!incoming.has_value()) return std::nullopt;
  const Eigen::Vector3d measured_motion = current_position - *previous_position;
  if (!measured_motion.allFinite() || measured_motion.dot(*incoming) <= 1.0e-6) {
    return std::nullopt;
  }
  const auto error = segmentDistanceToPoint(
      *previous_position, current_position, waypoint.position_enu);
  if (!error.has_value() || *error > waypoint.acceptance_radius_m + 1.0e-6) {
    return std::nullopt;
  }
  return error;
}

ImmutableRouteSnapshot RouteProgress::snapshot(
    const std::string& mission_id, const std::string& frame,
    const std::uint64_t route_revision, const std::uint64_t request_id,
    const std::size_t active_waypoint_index) const {
  ImmutableRouteSnapshot output;
  output.mission_id = mission_id;
  output.frame = frame;
  output.route_revision = route_revision;
  output.request_id = request_id;
  output.active_waypoint_index = active_waypoint_index;
  output.waypoints = waypoints_;
  output.segments = segments_;
  output.waypoint_arc_lengths_m = waypoint_arc_lengths_;
  output.measured_progress = state_;
  output.total_length_m = total_length_m_;
  return output;
}

}  // namespace navigation_mission
