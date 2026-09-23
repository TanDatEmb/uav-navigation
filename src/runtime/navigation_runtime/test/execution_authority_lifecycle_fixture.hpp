#pragma once

#include <memory>

#include <navigation_execution/execution_authority.hpp>
#include <navigation_runtime/execution_lifecycle_view.hpp>
#include <navigation_runtime/execution_recovery_state.hpp>

namespace navigation_runtime {

// Test fixture whose every transition reaches the product
// ExecutionAuthority; it has no second mutable lifecycle.
class ExecutionLifecycleFixture final {
 public:
  [[nodiscard]] navigation_execution::ExecutionAuthoritySnapshot snapshot() const noexcept {
    return authority_.snapshot();
  }

  void reset(std::uint64_t localization_epoch) noexcept {
    authority_.reset(localization_epoch);
  }

  void beginGoal(std::uint64_t localization_epoch, std::uint64_t goal_epoch,
                 std::uint64_t, bool retain_command) noexcept {
    (void)authority_.beginGoal(localization_epoch, goal_epoch, retain_command);
  }

  navigation_execution::CommitDecision commandCommitted(
      const navigation_planning::CandidateBundle& source) {
    auto candidate = complete(source);
    const auto before = authority_.snapshot();
    if (!before.world_identity ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *before.world_identity, candidate.world_identity)) {
      (void)authority_.publishWorldIdentityIfCurrent(
          candidate.world_identity, before.version, before.active, false);
    }
    auto goal = std::make_shared<navigation_contracts::msg::NavigationGoal>();
    goal->mission_id = "lifecycle_fixture";
    goal->request_id = candidate.request_id;
    return authority_.tryCommit(
        {candidate.world_identity, candidate.goal_epoch, ++transaction_id_},
        std::move(goal),
        std::make_shared<const navigation_planning::CandidateBundle>(
            std::move(candidate)));
  }

  bool observeRetainedCommand(const navigation_planning::CandidateBundle& bundle,
                              bool safety_suffix_active) noexcept {
    return authority_.observeRetainedCommand(bundle, safety_suffix_active);
  }
  bool observeSampledSafetyRole(const navigation_planning::CandidateBundle& bundle,
                                navigation_planning::CandidateRole role) noexcept {
    return authority_.observeSampledSafetyRole(bundle, role);
  }
  bool preserveSafetySuffix(const navigation_planning::CandidateBundle& bundle) noexcept {
    return authority_.preserveSafetySuffix(bundle);
  }
  bool requestRestartFromRest(const navigation_planning::CandidateBundle& bundle) noexcept {
    return authority_.requestRestartFromRest(bundle);
  }
  void clearRestartFromRest() noexcept { authority_.clearRestartFromRest(); }
  bool applyRecoveryEvent(ExecutionRecoveryEvent event,
                          const navigation_planning::CandidateBundle& bundle) noexcept {
    return authority_.applyRecoveryEvent(event, bundle);
  }
  bool stoppedHold(const navigation_planning::CandidateBundle& bundle) noexcept {
    return authority_.stoppedHold(bundle);
  }
  void failClosed() noexcept { authority_.failClosed(); }
  void suspendCommand() noexcept { authority_.suspendCommand(); }
  void clearGoal(std::uint64_t localization_epoch) noexcept {
    authority_.clearGoal(localization_epoch);
  }

 private:
  static navigation_planning::CandidateBundle complete(
      const navigation_planning::CandidateBundle& source) {
    auto candidate = source;
    candidate.world_identity = {source.localization_epoch, 1U, 1U, 1};
    candidate.pinned_world_identity = candidate.world_identity;
    candidate.valid_from_ns = 1;
    candidate.valid_until_ns = 100;
    candidate.activation_stamp_ns = 1;
    candidate.start_wall_time_s = 1.0e-9;
    candidate.duration_s = 400.0e-9;
    candidate.declared_start_ns = 1;
    candidate.declared_end_ns = 401;
    candidate.certificates = {true, true, true, true};
    candidate.protected_region.minimum = Eigen::Vector3d::Zero();
    candidate.protected_region.maximum = Eigen::Vector3d::Ones();
    candidate.backup_available =
        candidate.kind == navigation_planning::CandidateBundleKind::kMainWithBackup;
    candidate.backup_start_time_s = candidate.backup_available ? 200.0e-9 : 0.0;
    if (candidate.backup_available) {
      candidate.role_schedule = {
          {0.0, 200.0e-9, navigation_planning::CandidateRole::kMain},
          {200.0e-9, 400.0e-9, navigation_planning::CandidateRole::kBackup}};
    } else {
      candidate.role_schedule = {{0.0, 400.0e-9, candidate.role}};
    }
    candidate.evaluator = [](
        std::int64_t, navigation_planning::TrajectoryPoint& point) {
      point = {};
      return true;
    };
    return candidate;
  }

  navigation_execution::ExecutionAuthority authority_;
  std::uint64_t transaction_id_{0U};
};

}  // namespace navigation_runtime
