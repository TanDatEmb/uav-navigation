#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/planning_timing.hpp>

namespace navigation_runtime {

enum class PathRelativeTrackingStatus : std::uint8_t {
  kInvalidInput,
  kUnavailableSupport,
  kLowReferenceSpeed,
  kAmbiguousProjection,
  kPhaseExceeded,
  kOutsidePathTube,
  kReverseMotion,
  kPredictionOutsideTube,
  kAbsoluteDivergence,
  kAccepted,
};

struct PathRelativeTrackingResult {
  PathRelativeTrackingStatus status{PathRelativeTrackingStatus::kInvalidInput};
  std::int64_t projected_stamp_ns{0};
  double phase_offset_s{std::numeric_limits<double>::quiet_NaN()};
  double predicted_phase_offset_s{std::numeric_limits<double>::quiet_NaN()};
  double path_error_m{std::numeric_limits<double>::quiet_NaN()};
  double cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double vertical_error_m{std::numeric_limits<double>::quiet_NaN()};
  double predicted_path_error_m{std::numeric_limits<double>::quiet_NaN()};
  double progress_rate{std::numeric_limits<double>::quiet_NaN()};
  double raw_error_m{std::numeric_limits<double>::quiet_NaN()};
  double outer_anchor_limit_m{std::numeric_limits<double>::quiet_NaN()};
  unsigned evaluation_count{0};

  [[nodiscard]] bool accepted() const noexcept {
    return status == PathRelativeTrackingStatus::kAccepted;
  }
};

// Local geometric path following assessment. The phase is an actual closest
// point parameter, independent of sensor age: a vehicle ahead of or behind the
// timed command can still be on the intended path. No reference is retimed.
//
// Call outside world/command locks. Work is bounded by 33 coarse samples and
// at most four 26-evaluation local refinements, plus endpoint/prediction
// samples. Every distance uses the analytic evaluator, never a polyline chord.
// Missed narrow minima can cause conservative rejection; a returned point is
// still a real point on the declared MAIN path. Competing local minima with
// indistinguishable distance are rejected instead of choosing a branch.
//
// The prediction retains the existing constant-velocity vehicle model. It
// advances the path parameter at measured tangential speed and checks both
// geometric error and phase drift. It is not a nonlinear reachability proof.
inline PathRelativeTrackingResult assessPathRelativeTracking(
    const navigation_planning::CandidateBundle& bundle,
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& measured_velocity,
    std::int64_t now_ns, std::int64_t source_stamp_ns,
    double validation_interval_s, double phase_window_s,
    double contour_budget_m, double base_outer_anchor_limit_m,
    bool state_fresh, bool body_known_free, bool path_clear) {
  using navigation_planning::CandidateRole;
  using navigation_planning::TrajectoryPoint;
  PathRelativeTrackingResult out;
  if (!state_fresh || !body_known_free || !path_clear || !bundle.valid() ||
      !measured_position.allFinite() || !measured_velocity.allFinite() ||
      now_ns <= 0 || source_stamp_ns <= 0 || source_stamp_ns > now_ns ||
      !std::isfinite(validation_interval_s) || validation_interval_s <= 0.0 ||
      !std::isfinite(phase_window_s) || phase_window_s <= 0.0 ||
      !std::isfinite(contour_budget_m) || contour_budget_m <= 0.0 ||
      !std::isfinite(base_outer_anchor_limit_m) || base_outer_anchor_limit_m <= 0.0) {
    return out;
  }
  out.status = PathRelativeTrackingStatus::kUnavailableSupport;
  if (bundle.role != CandidateRole::kMain || bundle.terminal_stop ||
      now_ns < bundle.valid_from_ns || now_ns > bundle.valid_until_ns ||
      static_cast<long double>(validation_interval_s) * 1.0e9L >
          static_cast<long double>(bundle.valid_until_ns) - now_ns) {
    return out;
  }

  // Express the search relative to source time, avoiding epoch-sized double
  // arithmetic. Geometric history can precede activation/valid_from: retained
  // polynomial metadata is not permission to publish an expired command.
  const long double source_local_s =
      (static_cast<long double>(source_stamp_ns) - bundle.declared_start_ns) * 1.0e-9L;
  const double source_time_s = static_cast<double>(source_local_s);
  const auto source_role = bundle.scheduledRole(source_time_s);
  if (!source_role || *source_role != CandidateRole::kMain) return out;
  double main_begin_s = 0.0, main_end_s = 0.0;
  bool found_interval = false;
  for (const auto& interval : bundle.role_schedule) {
    if (interval.role == CandidateRole::kMain &&
        source_time_s >= interval.begin_time_s && source_time_s < interval.end_time_s) {
      main_begin_s = interval.begin_time_s;
      main_end_s = interval.end_time_s;
      found_interval = true;
      break;
    }
  }
  if (!found_interval) return out;
  const double now_local_s = static_cast<double>(
      (static_cast<long double>(now_ns) - bundle.declared_start_ns) * 1.0e-9L);
  if (now_local_s < main_begin_s ||
      now_local_s + validation_interval_s >= main_end_s) return out;
  const double low = std::max(-phase_window_s, main_begin_s - source_time_s);
  // Keep geometric queries on the MAIN side of an exact role seam.
  const double high = std::min(phase_window_s, main_end_s - source_time_s - 1.0e-9);
  if (!(high > low)) return out;

  const auto stampAtOffset = [&](double offset_s, std::int64_t& stamp) {
    const long double value = static_cast<long double>(source_stamp_ns) +
        std::round(static_cast<long double>(offset_s) * 1.0e9L);
    if (!std::isfinite(value) || value <= 0 ||
        value > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      return false;
    }
    stamp = static_cast<std::int64_t>(value);
    return true;
  };
  bool evaluation_failed = false;
  const auto sample = [&](double offset_s, TrajectoryPoint& point) {
    std::int64_t stamp = 0;
    if (!stampAtOffset(offset_s, stamp)) {
      evaluation_failed = true;
      return false;
    }
    ++out.evaluation_count;
    const auto value = bundle.sampleAtDeclaredStamp(stamp);
    if (!value || value->role != CandidateRole::kMain) {
      evaluation_failed = true;
      return false;
    }
    point = *value;
    return true;
  };
  const auto squaredDistance = [&](double offset_s) {
    TrajectoryPoint point;
    if (!sample(offset_s, point)) return std::numeric_limits<double>::infinity();
    return (point.position_world - measured_position).squaredNorm();
  };

  constexpr unsigned kSegments = 32;
  constexpr unsigned kMaximumMinima = 4;
  constexpr unsigned kRefinementSteps = 24;
  constexpr double kGolden = 0.6180339887498948482;
  std::array<double, kSegments + 1> offsets{}, distances{};
  for (unsigned i = 0; i <= kSegments; ++i) {
    offsets[i] = low + (high - low) * static_cast<double>(i) / kSegments;
    distances[i] = squaredDistance(offsets[i]);
    if (!std::isfinite(distances[i])) return out;
  }
  std::array<double, kMaximumMinima> minima_offsets{}, minima_distances{};
  unsigned minima_count = 0, refinement_count = 0;
  for (unsigned i = 0; i <= kSegments; ++i) {
    if ((i > 0 && distances[i] > distances[i - 1]) ||
        (i < kSegments && distances[i] > distances[i + 1])) continue;
    if (refinement_count++ == kMaximumMinima) {
      out.status = PathRelativeTrackingStatus::kAmbiguousProjection;
      return out;
    }
    double left = offsets[i == 0 ? 0 : i - 1];
    double right = offsets[i == kSegments ? kSegments : i + 1];
    double x1 = right - kGolden * (right - left);
    double x2 = left + kGolden * (right - left);
    double d1 = squaredDistance(x1), d2 = squaredDistance(x2);
    for (unsigned step = 0; step < kRefinementSteps; ++step) {
      if (d1 <= d2) {
        right = x2; x2 = x1; d2 = d1;
        x1 = right - kGolden * (right - left); d1 = squaredDistance(x1);
      } else {
        left = x1; x1 = x2; d1 = d2;
        x2 = left + kGolden * (right - left); d2 = squaredDistance(x2);
      }
    }
    if (evaluation_failed) return out;
    double minimum_offset = d1 <= d2 ? x1 : x2;
    double minimum_distance = std::min(d1, d2);
    if (distances[i] < minimum_distance) {
      minimum_offset = offsets[i]; minimum_distance = distances[i];
    }
    if (!std::isfinite(minimum_distance)) return out;
    // Two adjacent coarse cells can bracket the same smooth minimum.
    bool duplicate = false;
    for (unsigned j = 0; j < minima_count; ++j) {
      if (std::abs(minima_offsets[j] - minimum_offset) <= 1.0e-6) {
        if (minimum_distance < minima_distances[j]) {
          minima_offsets[j] = minimum_offset; minima_distances[j] = minimum_distance;
        }
        duplicate = true;
      }
    }
    if (!duplicate) {
      minima_offsets[minima_count] = minimum_offset;
      minima_distances[minima_count++] = minimum_distance;
    }
  }
  if (minima_count == 0) return out;
  unsigned best = 0;
  for (unsigned i = 1; i < minima_count; ++i) {
    if (minima_distances[i] < minima_distances[best]) best = i;
  }
  for (unsigned i = 0; i < minima_count; ++i) {
    if (i != best && std::abs(minima_distances[i] - minima_distances[best]) <=
        1.0e-10 * std::max(1.0, minima_distances[best])) {
      out.status = PathRelativeTrackingStatus::kAmbiguousProjection;
      return out;
    }
  }
  out.phase_offset_s = minima_offsets[best];
  TrajectoryPoint projected, command_now;
  if (!sample(out.phase_offset_s, projected) ||
      !sample(static_cast<double>(now_ns - source_stamp_ns) * 1.0e-9, command_now) ||
      !stampAtOffset(out.phase_offset_s, out.projected_stamp_ns)) return out;
  const double reference_speed = projected.velocity_world.norm();
  if (!std::isfinite(reference_speed) ||
      reference_speed < navigation_planning::PlanningTimingContract::kStationarySpeedMps) {
    out.status = PathRelativeTrackingStatus::kLowReferenceSpeed;
    return out;
  }
  const Eigen::Vector3d tangent = projected.velocity_world / reference_speed;
  const Eigen::Vector3d error = measured_position - projected.position_world;
  const double longitudinal_residual = error.dot(tangent);
  // A constrained endpoint is not evidence that a larger phase error fits
  // the phase window; the unconstrained minimum must lie in local support.
  if ((out.phase_offset_s <= low + 1.0e-7 && longitudinal_residual < -1.0e-6) ||
      (out.phase_offset_s >= high - 1.0e-7 && longitudinal_residual > 1.0e-6)) {
    out.status = PathRelativeTrackingStatus::kPhaseExceeded;
    return out;
  }
  out.path_error_m = error.norm();
  out.cross_track_error_m = (error - longitudinal_residual * tangent).norm();
  out.vertical_error_m = std::abs(error.z());
  out.raw_error_m = (command_now.position_world - measured_position).norm();
  if (!std::isfinite(out.path_error_m) || out.path_error_m > contour_budget_m) {
    out.status = PathRelativeTrackingStatus::kOutsidePathTube;
    return out;
  }
  // The outer pointwise guard must not undo the phase-based path check at
  // higher speeds. Its additional along-path travel allowance scales with
  // the same finite phase window; the primary 3-D path tube stays unchanged.
  out.outer_anchor_limit_m = base_outer_anchor_limit_m + phase_window_s *
      std::max({reference_speed, command_now.velocity_world.norm(), measured_velocity.norm()});
  if (!std::isfinite(out.raw_error_m) || !std::isfinite(out.outer_anchor_limit_m) ||
      out.raw_error_m > out.outer_anchor_limit_m) {
    out.status = PathRelativeTrackingStatus::kAbsoluteDivergence;
    return out;
  }
  const double tangential_speed = measured_velocity.dot(tangent);
  if (!std::isfinite(tangential_speed) || tangential_speed < 0.0) {
    out.status = PathRelativeTrackingStatus::kReverseMotion;
    return out;
  }
  out.progress_rate = tangential_speed / reference_speed;
  const double horizon_s = static_cast<double>(now_ns - source_stamp_ns) * 1.0e-9 +
      validation_interval_s;
  out.predicted_phase_offset_s = out.phase_offset_s + (out.progress_rate - 1.0) * horizon_s;
  if (!std::isfinite(out.predicted_phase_offset_s) ||
      std::abs(out.predicted_phase_offset_s) > phase_window_s + 1.0e-7) {
    out.status = PathRelativeTrackingStatus::kPhaseExceeded;
    return out;
  }
  TrajectoryPoint future_reference;
  const double future_offset_s = out.phase_offset_s + out.progress_rate * horizon_s;
  if (!std::isfinite(future_offset_s) ||
      source_time_s + future_offset_s < main_begin_s ||
      source_time_s + future_offset_s >= main_end_s ||
      !sample(future_offset_s, future_reference)) return out;
  const Eigen::Vector3d predicted_position = measured_position + measured_velocity * horizon_s;
  out.predicted_path_error_m = (predicted_position - future_reference.position_world).norm();
  if (!std::isfinite(out.predicted_path_error_m) || out.predicted_path_error_m > contour_budget_m) {
    out.status = PathRelativeTrackingStatus::kPredictionOutsideTube;
    return out;
  }
  out.status = PathRelativeTrackingStatus::kAccepted;
  return out;
}

}  // namespace navigation_runtime
