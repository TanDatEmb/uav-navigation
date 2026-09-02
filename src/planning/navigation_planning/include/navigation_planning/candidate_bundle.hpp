#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include <navigation_planning/route_boundary.hpp>
#include <navigation_common/time.hpp>

namespace navigation_planning {

enum class CandidateRole : std::uint8_t { kMain, kBackup, kEmergency };

enum class CandidateBundleKind : std::uint8_t {
  kMainWithBackup,
  kTerminalStop,
  kBackupOnly,
  kEmergencyBrake,
};

enum class CandidateSource : std::uint8_t {
  kDeterministicBaseline,
  kRefined,
  kRetained,
  kEmergency,
};

struct CandidateRoleInterval {
  double begin_time_s{0.0};
  double end_time_s{0.0};
  CandidateRole role{CandidateRole::kMain};
};

struct CompleteBundleCertificates {
  bool dynamics{false};
  bool flatness{false};
  bool world{false};
  bool terminal_stop{false};

  [[nodiscard]] bool completeFor(CandidateBundleKind kind) const noexcept {
    if (!dynamics || !flatness || !world) return false;
    return kind != CandidateBundleKind::kTerminalStop || terminal_stop;
  }
};

struct CompleteCandidateQuality {
  double remaining_main_horizon_s{std::numeric_limits<double>::quiet_NaN()};
  double minimum_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double maximum_dynamic_utilization{std::numeric_limits<double>::quiet_NaN()};
  double route_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double objective_cost{std::numeric_limits<double>::quiet_NaN()};

  [[nodiscard]] bool finite() const noexcept {
    return std::isfinite(remaining_main_horizon_s) &&
           std::isfinite(minimum_clearance_m) && minimum_clearance_m >= 0.0 &&
           std::isfinite(maximum_dynamic_utilization) &&
           maximum_dynamic_utilization >= 0.0 &&
           std::isfinite(route_progress_m) && route_progress_m >= 0.0 &&
           std::isfinite(objective_cost);
  }
};

[[nodiscard]] constexpr bool candidateRoleValid(CandidateRole role) noexcept {
  return role == CandidateRole::kMain || role == CandidateRole::kBackup ||
         role == CandidateRole::kEmergency;
}

struct TrajectoryPoint {
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  CandidateRole role{CandidateRole::kMain};
  bool finished{false};
  double trajectory_time_s{0.0};

  [[nodiscard]] bool finite() const noexcept {
    return position_world.allFinite() && velocity_world.allFinite() &&
           acceleration_world.allFinite() && jerk_world.allFinite() &&
           std::isfinite(yaw) && std::isfinite(yaw_rate) &&
           std::isfinite(trajectory_time_s);
  }
};

// A candidate owns no mutable planner state.  The evaluator is an immutable,
// product-owned callable supplied by a planner implementation and is never
// invoked while a world or commit mutex is held.
struct CandidateBundle {
  navigation_world_model::WorldSnapshotIdentity pinned_world_identity;
  navigation_world_model::WorldSnapshotIdentity world_identity;
  std::uint64_t localization_epoch{0};
  std::uint64_t goal_epoch{0};
  std::uint64_t request_id{0};
  std::uint64_t bundle_generation{0};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  // Future activation is distinct from the trajectory's analytic origin. A
  // successor may retain historical polynomial metadata while execution must
  // not expose it before this producer-declared splice instant.
  std::int64_t activation_stamp_ns{0};
  CandidateRole role{CandidateRole::kMain};
  // Optional trajectory metadata used by execution diagnostics and handover
  // checks. The evaluator remains the only source for a sampled point.
  double start_wall_time_s{std::numeric_limits<double>::quiet_NaN()};
  double duration_s{std::numeric_limits<double>::quiet_NaN()};
  // Producer-owned integer wall-clock declaration. When present, endpoint
  // checks use this exact nanosecond boundary rather than re-rounding two
  // independently represented doubles at the consumer.
  std::int64_t declared_start_ns{0};
  std::int64_t declared_end_ns{0};
  double backup_start_time_s{std::numeric_limits<double>::quiet_NaN()};
  bool backup_available{false};
  // CandidateBundleKind describes the executable role partition. This
  // explicit semantic bit distinguishes a terminal STOP with a braking
  // suffix from an ordinary moving MAIN+BACKUP command.
  bool terminal_stop{false};
  CandidateBundleKind kind{CandidateBundleKind::kTerminalStop};
  CandidateSource source{CandidateSource::kDeterministicBaseline};
  CompleteBundleCertificates certificates{};
  navigation_world_model::AxisAlignedBox protected_region{};
  std::vector<CandidateRoleInterval> role_schedule{};
  std::optional<CompleteCandidateQuality> quality{};
  std::optional<RouteBoundaryConstraint> route_boundary_constraint{};
  std::optional<RouteBoundaryEvent> route_boundary_event{};
  std::function<bool(std::int64_t, TrajectoryPoint&)> evaluator;

  [[nodiscard]] bool roleScheduleValid() const noexcept {
    if (!std::isfinite(duration_s) || duration_s < 0.0 || role_schedule.empty()) {
      return false;
    }
    double cursor = 0.0;
    for (const auto& interval : role_schedule) {
      if (!std::isfinite(interval.begin_time_s) ||
          !std::isfinite(interval.end_time_s) ||
          std::abs(interval.begin_time_s - cursor) > 1.0e-6 ||
          interval.end_time_s <= interval.begin_time_s ||
          interval.end_time_s > duration_s + 1.0e-6 ||
          !candidateRoleValid(interval.role)) {
        return false;
      }
      cursor = interval.end_time_s;
    }
    return std::abs(cursor - duration_s) <= 1.0e-6;
  }

  [[nodiscard]] std::optional<CandidateRole> scheduledRole(
      double trajectory_time_s) const noexcept {
    if (!roleScheduleValid() || !std::isfinite(trajectory_time_s) ||
        trajectory_time_s < 0.0 || trajectory_time_s > duration_s + 1.0e-9) {
      return std::nullopt;
    }
    // Role boundaries are part of the integer timestamp contract.  Rounding
    // both the sampled local time and interval offsets avoids reintroducing
    // epoch-scale cancellation through a second, double-only boundary test.
    const auto to_offset_ns = [](const double seconds) -> std::optional<std::int64_t> {
      if (!std::isfinite(seconds) || seconds < 0.0) return std::nullopt;
      const long double rounded = std::round(
          static_cast<long double>(seconds) * 1.0e9L);
      if (!std::isfinite(rounded) ||
          rounded > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        return std::nullopt;
      }
      return static_cast<std::int64_t>(rounded);
    };
    const auto sample_ns = to_offset_ns(trajectory_time_s);
    if (!sample_ns) return std::nullopt;
    for (const auto& interval : role_schedule) {
      const auto begin_ns = to_offset_ns(interval.begin_time_s);
      const auto end_ns = to_offset_ns(interval.end_time_s);
      if (begin_ns && end_ns && *sample_ns >= *begin_ns &&
          *sample_ns < *end_ns) {
        return interval.role;
      }
    }
    // The half-open intervals deliberately assign an exact internal boundary
    // to the following role. At the declared end there is no following
    // interval, so select the final producer-declared role explicitly. The
    // previous endpoint clause returned the first interval at every endpoint,
    // rejecting valid MainWithBackup samples when the evaluator correctly
    // emitted BACKUP at the final instant.
    const auto duration_ns = to_offset_ns(duration_s);
    if (duration_ns && *sample_ns == *duration_ns) {
      return role_schedule.back().role;
    }
    return std::nullopt;
  }

  [[nodiscard]] bool sampledRoleAllowed(CandidateRole sampled_role) const noexcept {
    if (!candidateRoleValid(sampled_role)) return false;
    if (role == CandidateRole::kEmergency) {
      return sampled_role == CandidateRole::kEmergency;
    }
    if (role == CandidateRole::kBackup) {
      return sampled_role == CandidateRole::kBackup;
    }
    return sampled_role == CandidateRole::kMain ||
           (backup_available && sampled_role == CandidateRole::kBackup);
  }

  [[nodiscard]] bool valid() const noexcept {
    const bool kind_contract =
        (kind == CandidateBundleKind::kMainWithBackup &&
         role == CandidateRole::kMain && backup_available) ||
        (kind == CandidateBundleKind::kTerminalStop &&
         role == CandidateRole::kMain && !backup_available) ||
        (kind == CandidateBundleKind::kBackupOnly &&
         role == CandidateRole::kBackup) ||
        (kind == CandidateBundleKind::kEmergencyBrake &&
         role == CandidateRole::kEmergency);
    const bool terminal_stop_contract =
        !terminal_stop ||
        (role == CandidateRole::kMain &&
         (kind == CandidateBundleKind::kTerminalStop ||
          kind == CandidateBundleKind::kMainWithBackup) &&
         certificates.terminal_stop);
    const auto expected_start_ns = navigation_common::secondsToNanoseconds(start_wall_time_s);
    const auto expected_end_ns = navigation_common::secondsSumToNanoseconds(
        start_wall_time_s, duration_s);
    const bool endpoint_metadata_consistent =
        (declared_start_ns > 0 && declared_end_ns >= declared_start_ns &&
         expected_start_ns.has_value() && expected_end_ns.has_value() &&
         declared_start_ns == *expected_start_ns && declared_end_ns == *expected_end_ns);
    const bool activation_metadata_consistent = activation_stamp_ns > 0 &&
        activation_stamp_ns == valid_from_ns;
    return localization_epoch != 0 && goal_epoch != 0 && request_id != 0 &&
           bundle_generation != 0 && valid_from_ns > 0 &&
           valid_until_ns >= valid_from_ns && static_cast<bool>(evaluator) &&
           candidateRoleValid(role) && kind_contract && terminal_stop_contract &&
           certificates.completeFor(kind) && protected_region.valid() &&
           activation_metadata_consistent &&
           endpoint_metadata_consistent &&
           roleScheduleValid() && (!quality || quality->finite()) &&
           (!route_boundary_constraint || route_boundary_constraint->valid()) &&
           (!route_boundary_event || route_boundary_event->valid()) &&
           (!route_boundary_constraint || route_boundary_event.has_value()) &&
           pinned_world_identity.localization_epoch == localization_epoch &&
           pinned_world_identity.generation != 0 &&
           pinned_world_identity.revision != 0 &&
           pinned_world_identity.observation_stamp_ns > 0 &&
           world_identity.localization_epoch == localization_epoch &&
           world_identity.generation != 0 && world_identity.revision != 0 &&
           world_identity.observation_stamp_ns > 0;
  }

  [[nodiscard]] bool hasTrajectoryMetadata() const noexcept {
    return std::isfinite(start_wall_time_s) && start_wall_time_s > 0.0 &&
           std::isfinite(duration_s) && duration_s >= 0.0 &&
           std::isfinite(backup_start_time_s) && backup_start_time_s >= 0.0 &&
           backup_start_time_s <= duration_s + 1.0e-9;
  }

  // The declared endpoint exists for every executable trajectory, including a
  // main-only candidate that has no backup suffix metadata. Keep this query
  // separate from hasTrajectoryMetadata(), which is also used by the
  // retained-backup safety path and therefore intentionally requires a valid
  // backup-start declaration.
  [[nodiscard]] bool hasDeclaredEndpointMetadata() const noexcept {
    return std::isfinite(start_wall_time_s) && start_wall_time_s > 0.0 &&
           std::isfinite(duration_s) && duration_s >= 0.0;
  }

  [[nodiscard]] std::optional<TrajectoryPoint> sample(std::int64_t stamp_ns) const {
    if (!valid() || stamp_ns < valid_from_ns || stamp_ns > valid_until_ns) {
      return std::nullopt;
    }
    TrajectoryPoint point;
    point.role = role;
    if (!evaluator(stamp_ns, point) || !point.finite() ||
        !sampledRoleAllowed(point.role)) {
      return std::nullopt;
    }
    const auto scheduled = scheduledRole(point.trajectory_time_s);
    if (!scheduled || *scheduled != point.role) return std::nullopt;
    return point;
  }

  // The declared trajectory endpoint can lie beyond the short execution lease
  // used for incremental replanning. This metadata query intentionally samples
  // the evaluator at the terminal trajectory timestamp without widening the
  // executable validity interval used by sample().
  [[nodiscard]] std::optional<TrajectoryPoint> sampleAtDeclaredEnd() const {
    if (!valid() || !hasDeclaredEndpointMetadata()) return std::nullopt;
    const auto end_stamp_ns = declared_end_ns > 0
        ? std::optional<std::int64_t>{declared_end_ns}
        : navigation_common::secondsSumToNanoseconds(start_wall_time_s, duration_s);
    if (!end_stamp_ns || *end_stamp_ns <= 0) {
      return std::nullopt;
    }
    TrajectoryPoint point;
    point.role = role;
    if (!evaluator(*end_stamp_ns, point) || !point.finite() ||
        !sampledRoleAllowed(point.role)) {
      return std::nullopt;
    }
    const auto scheduled = scheduledRole(duration_s);
    if (!scheduled || *scheduled != point.role) return std::nullopt;
    // The evaluator's runtime completion predicate is intentionally strict
    // (it marks a sample finished only after the declared end). This API is
    // explicitly the declared endpoint, so normalize its lifecycle flag for
    // consumers performing terminal handover checks.
    point.finished = true;
    return point;
  }
};

// Numerical boundary tolerances for replacing one analytic command with the
// next. These are not tracking or safety margins. They only account for the
// roundoff of evaluating two immutable polynomials at the same wall timestamp.
struct CandidateHandoffCertificate final {
  static constexpr double kPositionToleranceM = 1.0e-5;
  static constexpr double kVelocityToleranceMps = 1.0e-5;
  static constexpr double kAccelerationToleranceMps2 = 1.0e-4;
  static constexpr double kJerkToleranceMps3 = 1.0e-3;
  static constexpr double kYawToleranceRad = 1.0e-6;
  static constexpr double kYawRateToleranceRadS = 1.0e-5;
};

// Verify the exact old-command -> new-command boundary before either command
// is exposed. Measured-state emergency candidates intentionally do not use
// this helper: their separate certificate preserves measured P/V when the old
// command is already outside the tracking envelope.
[[nodiscard]] inline bool candidateBundleHandoffContinuous(
    const CandidateBundle& previous,
    const CandidateBundle& next,
    std::int64_t* handoff_stamp_ns = nullptr,
    bool allow_expired_safety_after_certified_stop = false) {
  if (!previous.valid() || !next.valid() ||
      next.kind == CandidateBundleKind::kEmergencyBrake ||
      !next.hasDeclaredEndpointMetadata()) {
    return false;
  }
  const auto handoff = next.declared_start_ns > 0
      ? std::optional<std::int64_t>{next.declared_start_ns}
      : navigation_common::secondsToNanoseconds(next.start_wall_time_s);
  if (!handoff || *handoff <= 0) return false;
  if (handoff_stamp_ns) *handoff_stamp_ns = *handoff;

  // A measured emergency brake or a certified MAIN+BACKUP safety suffix may
  // end at its analytic stop point while PX4/LIO tracking still leaves a
  // bounded position residual. Once the suffix has fully expired and the
  // runtime has observed a certified stop, the next PlanFromRest command is a
  // new measured-state episode, not an overlapping analytic splice. Requiring
  // the expired suffix polynomial to equal the new measured state would reject
  // the safe recovery path and force PX4 Hold.
  //
  // Keep this exception deliberately narrow: prove the old endpoint is
  // sampleable, require the handoff at or after the declared end, and only
  // allow a completed safety role or a completed terminal-stop hold. The
  // latter is needed when a STOP waypoint must be replanned from the measured
  // stopped state because the analytic endpoint was inside the acceptance
  // ball but the vehicle itself was not; it never applies to an active
  // nominal MAIN.
  if (allow_expired_safety_after_certified_stop &&
      previous.declared_end_ns > 0 && *handoff >= previous.declared_end_ns) {
    const auto previous_endpoint = previous.sampleAtDeclaredEnd();
    const bool expired_emergency = previous.kind == CandidateBundleKind::kEmergencyBrake &&
        previous_endpoint && previous_endpoint->role == CandidateRole::kEmergency;
    const bool expired_backup_suffix =
        (previous.kind == CandidateBundleKind::kMainWithBackup ||
         previous.kind == CandidateBundleKind::kBackupOnly) &&
        previous_endpoint && previous_endpoint->role == CandidateRole::kBackup;
    const bool expired_terminal_stop =
        previous.kind == CandidateBundleKind::kTerminalStop &&
        previous.terminal_stop && previous_endpoint &&
        previous_endpoint->role == CandidateRole::kMain;
    if (previous_endpoint && previous_endpoint->finished &&
        (expired_emergency || expired_backup_suffix || expired_terminal_stop)) {
      return true;
    }
  }

  const auto previous_sample = previous.sample(*handoff);
  if (!previous_sample || !next.evaluator) return false;
  TrajectoryPoint next_sample;
  next_sample.role = next.role;
  if (!next.evaluator(*handoff, next_sample) || !next_sample.finite()) return false;
  const auto scheduled_role = next.scheduledRole(next_sample.trajectory_time_s);
  if (!scheduled_role || *scheduled_role != next_sample.role) return false;

  const double yaw_residual = std::remainder(
      next_sample.yaw - previous_sample->yaw, 2.0 * std::acos(-1.0));
  return (next_sample.position_world - previous_sample->position_world).norm() <=
             CandidateHandoffCertificate::kPositionToleranceM &&
         (next_sample.velocity_world - previous_sample->velocity_world).norm() <=
             CandidateHandoffCertificate::kVelocityToleranceMps &&
         (next_sample.acceleration_world - previous_sample->acceleration_world).norm() <=
             CandidateHandoffCertificate::kAccelerationToleranceMps2 &&
         (next_sample.jerk_world - previous_sample->jerk_world).norm() <=
             CandidateHandoffCertificate::kJerkToleranceMps3 &&
         std::abs(yaw_residual) <= CandidateHandoffCertificate::kYawToleranceRad &&
         std::abs(next_sample.yaw_rate - previous_sample->yaw_rate) <=
             CandidateHandoffCertificate::kYawRateToleranceRadS;
}

}  // namespace navigation_planning
