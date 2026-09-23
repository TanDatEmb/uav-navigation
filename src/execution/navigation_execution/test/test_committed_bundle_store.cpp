#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <new>

#include <gtest/gtest.h>

#include <navigation_execution/command_sampler.hpp>
#include <navigation_execution/execution_state_store.hpp>
#include <navigation_execution/timestamp_freshness.hpp>

namespace navigation_execution {

// Existing timeline cases exercise the same product authority with a complete
// immutable goal paired to each synthetic bundle. The adapter is test-local:
// production commits must supply their independently validated full goal.
class TestExecutionAuthority : public ExecutionAuthority {
 public:
  static std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goalFor(
      const std::shared_ptr<const navigation_planning::CandidateBundle>& bundle) {
    if (!bundle) return {};
    auto goal = std::make_shared<navigation_contracts::msg::NavigationGoal>();
    goal->mission_id = "test_mission";
    goal->request_id = bundle->request_id;
    return goal;
  }

  CommitDecision tryCommit(
      const CommitToken& token,
      std::shared_ptr<const navigation_planning::CandidateBundle> bundle) noexcept {
    auto goal = goalFor(bundle);
    return ExecutionAuthority::tryCommit(token, std::move(goal), std::move(bundle));
  }

  template <typename AdmissionFn>
  CommitDecision tryCommitIfCurrent(
      const CommitToken& token, const ExecutionTimelineSnapshot& predecessor,
      std::shared_ptr<const navigation_planning::CandidateBundle> bundle,
      AdmissionFn&& admit) noexcept {
    auto goal = goalFor(bundle);
    return ExecutionAuthority::tryCommitIfCurrent(
        token, predecessor, std::move(goal), std::move(bundle),
        std::forward<AdmissionFn>(admit));
  }

  template <typename FinalizeFn>
  CommitDecision tryCommitAndFinalize(
      const CommitToken& token,
      std::shared_ptr<const navigation_planning::CandidateBundle> bundle,
      FinalizeFn&& finalize) noexcept {
    auto goal = goalFor(bundle);
    return ExecutionAuthority::tryCommitAndFinalize(
        token, std::move(goal), std::move(bundle),
        std::forward<FinalizeFn>(finalize));
  }

  StageDecision stagePending(
      const CommitToken& token, const ExecutionAnchor& anchor,
      std::shared_ptr<const navigation_planning::CandidateBundle> bundle) noexcept {
    auto goal = goalFor(bundle);
    return ExecutionAuthority::stagePending(
        token, anchor, std::move(goal), std::move(bundle));
  }

  template <typename FinalizeFn>
  StageDecision stagePendingAndFinalize(
      const CommitToken& token, const ExecutionAnchor& anchor,
      std::shared_ptr<const navigation_planning::CandidateBundle> bundle,
      FinalizeFn&& finalize) noexcept {
    auto goal = goalFor(bundle);
    return ExecutionAuthority::stagePendingAndFinalize(
        token, anchor, std::move(goal), std::move(bundle),
        std::forward<FinalizeFn>(finalize));
  }
};

struct ExecutionTimelineStoreTestAccess {
  template <typename Finalize, typename Prepare>
  static navigation_world_model::WorldCommitDecision publish(
      TestExecutionAuthority& store,
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const ExecutionTimelineSnapshot& expected,
      const bool retain_active, const std::int64_t refreshed_until_ns,
      const bool retain_pending, Finalize&& finalize, Prepare&& prepare) {
    return store.publishWorldIdentityIfCurrentAndFinalizeRevocationImpl(
        identity, expected.version, expected.active, retain_active,
        std::forward<Finalize>(finalize), refreshed_until_ns,
        expected.pending, retain_pending, std::forward<Prepare>(prepare));
  }
};

}  // namespace navigation_execution

namespace {

navigation_planning::CandidateBundle candidateFor(
    std::uint64_t goal_epoch, std::uint64_t revision) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 3;
  candidate.world_identity.generation = 4;
  candidate.world_identity.revision = revision;
  candidate.world_identity.observation_stamp_ns = static_cast<std::int64_t>(revision);
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 3;
  candidate.goal_epoch = goal_epoch;
  candidate.request_id = goal_epoch + 10;
  candidate.bundle_generation = goal_epoch + 20;
  candidate.valid_from_ns = 1;
  candidate.valid_until_ns = 100;
  candidate.start_wall_time_s = 1.0e-9;
  candidate.duration_s = 399.0e-9;
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.declared_start_ns = *navigation_common::secondsToNanoseconds(
      candidate.start_wall_time_s);
  candidate.declared_end_ns = *navigation_common::secondsSumToNanoseconds(
      candidate.start_wall_time_s, candidate.duration_s);
  candidate.backup_start_time_s = 0.0;
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d::Zero();
  candidate.protected_region.maximum = Eigen::Vector3d::Ones();
  candidate.role_schedule = {
      {0.0, 399.0e-9, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [](std::int64_t stamp, navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  return candidate;
}

std::shared_ptr<const navigation_planning::CandidateBundle> successorFor(
    const navigation_execution::ExecutionAnchor& anchor,
    std::uint64_t goal_epoch) {
  auto successor = candidateFor(goal_epoch, anchor.command_world.revision);
  successor.localization_epoch = anchor.localization_epoch;
  successor.world_identity = anchor.command_world;
  successor.pinned_world_identity = anchor.command_world;
  successor.valid_from_ns = anchor.activation_stamp_ns;
  successor.activation_stamp_ns = anchor.activation_stamp_ns;
  successor.bundle_generation = anchor.active_bundle_generation + 100U;
  return std::make_shared<const navigation_planning::CandidateBundle>(successor);
}

bool publishWorldIdentityForTest(
    navigation_execution::TestExecutionAuthority& store,
    const navigation_world_model::WorldSnapshotIdentity& identity,
    std::shared_ptr<const navigation_planning::CandidateBundle> expected_bundle = {},
    bool retain_validated_bundle = false,
    std::int64_t refreshed_valid_until_ns = 0,
    std::shared_ptr<const navigation_planning::CandidateBundle> expected_pending = {},
    bool retain_validated_pending = false) {
  const auto snapshot = store.snapshot();
  if (!expected_bundle) expected_bundle = snapshot.active;
  if (!expected_pending) expected_pending = snapshot.pending;
  return store.publishWorldIdentityIfCurrent(
             identity, snapshot.version, std::move(expected_bundle),
             retain_validated_bundle, refreshed_valid_until_ns,
             std::move(expected_pending), retain_validated_pending) ==
         navigation_world_model::WorldCommitDecision::kCommitted;
}

TEST(TestExecutionAuthority, CommitRequiresCurrentWorldAndGoal) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  navigation_execution::CommitToken token{world, 7, 1};
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  EXPECT_EQ(store.tryCommit(token, candidate), navigation_execution::CommitDecision::kCommitted);

  auto advanced = world;
  advanced.revision = 2;
  advanced.observation_stamp_ns = 2;
  ASSERT_TRUE(publishWorldIdentityForTest(store, advanced));
  EXPECT_EQ(store.tryCommit(token, candidate), navigation_execution::CommitDecision::kWorldAdvanced);
  EXPECT_FALSE(store.load());

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 2));
  EXPECT_EQ(store.tryCommit({advanced, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_TRUE(store.load());
}

TEST(TestExecutionAuthority, RejectsOutOfOrderTransactionIdentity) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto first = std::make_shared<const navigation_planning::CandidateBundle>(candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 2}, first),
            navigation_execution::CommitDecision::kCommitted);
  auto stale = std::make_shared<const navigation_planning::CandidateBundle>(candidateFor(7, 1));
  EXPECT_EQ(store.tryCommit({world, 7, 1}, stale),
            navigation_execution::CommitDecision::kCancelled);
  EXPECT_EQ(store.load(), first);
}

class ConditionalCommit : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(publishWorldIdentityForTest(store, world));
    ASSERT_TRUE(store.setActiveGoalEpoch(7));
    active = makeCandidate(27);
    ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
              navigation_execution::CommitDecision::kCommitted);
  }

  std::shared_ptr<const navigation_planning::CandidateBundle> makeCandidate(
      std::uint64_t generation) const {
    auto data = candidateFor(7, 1);
    data.bundle_generation = generation;
    return std::make_shared<const navigation_planning::CandidateBundle>(data);
  }

  void expectUnchanged(
      const navigation_execution::ExecutionTimelineSnapshot& before) const {
    const auto after = store.snapshot();
    EXPECT_EQ(after.version, before.version);
    EXPECT_EQ(after.active, before.active);
    EXPECT_EQ(after.pending, before.pending);
    EXPECT_EQ(after.pending_activation_ns, before.pending_activation_ns);
    ASSERT_EQ(after.world_identity.has_value(), before.world_identity.has_value());
    if (before.world_identity) {
      EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
          *after.world_identity, *before.world_identity));
    }
  }

  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  std::shared_ptr<const navigation_planning::CandidateBundle> active;
};

TEST_F(ConditionalCommit, CurrentOwnerBeforeEndAdmitsExactlyOneReplacement) {
  const auto before = store.snapshot();
  const auto replacement = makeCandidate(28);
  std::int64_t fake_now_ns = active->declared_end_ns - 1;
  int predicate_calls = 0;
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, before, replacement, [&] {
              ++predicate_calls;
              return fake_now_ns < active->declared_end_ns;
            }), navigation_execution::CommitDecision::kCommitted);
  const auto after = store.snapshot();
  EXPECT_EQ(after.active, replacement);
  EXPECT_EQ(after.version, before.version + 1U);
  EXPECT_FALSE(after.pending);
  EXPECT_EQ(after.pending_activation_ns, 0);
  EXPECT_EQ(predicate_calls, 1);
}

TEST_F(ConditionalCommit, ExactEndAndAfterEndRejectWithoutMutationOrConsumedToken) {
  const auto before = store.snapshot();
  const auto anchor_before = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor_before);
  const auto replacement = makeCandidate(28);
  std::int64_t fake_now_ns = active->declared_end_ns;
  for (const auto now_ns : {active->declared_end_ns, active->declared_end_ns + 1}) {
    fake_now_ns = now_ns;
    EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, before, replacement, [&] {
                return fake_now_ns < active->declared_end_ns;
              }), navigation_execution::CommitDecision::kAdmissionRejected);
    expectUnchanged(before);
    const auto anchor_after = store.reserveAnchor(50, 50);
    ASSERT_TRUE(anchor_after);
    EXPECT_EQ(anchor_after->execution_lineage_version,
              anchor_before->execution_lineage_version);
  }
  // Same token is still available: rejection must not consume the watermark.
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, before, replacement,
                                    [] { return true; }),
            navigation_execution::CommitDecision::kCommitted);
}

TEST_F(ConditionalCommit, AdmissionIsRecheckedAfterWaitingForStoreLock) {
  const auto before = store.snapshot();
  const auto replacement = makeCandidate(28);
  std::atomic<std::int64_t> fake_now_ns{active->declared_end_ns - 1};
  std::atomic<int> predicate_calls{0};
  std::promise<void> holder_entered;
  std::promise<void> release_holder;
  auto release = release_holder.get_future().share();
  auto holder = std::async(std::launch::async, [&] {
    return store.publishIfCurrent(active, 7, [&] {
      holder_entered.set_value();
      release.wait();
      return true;
    });
  });
  holder_entered.get_future().wait();
  std::promise<void> request_started;
  auto started = request_started.get_future();
  auto waiter = std::async(std::launch::async, [&] {
    // Signals request invocation, not positive acquisition/waiting inside
    // the API. Source-order review supplies the under-lock placement proof;
    // this case controls END advancement while the store lock is held.
    request_started.set_value();
    return store.tryCommitIfCurrent({world, 7, 2}, before, replacement, [&] {
      ++predicate_calls;
      return fake_now_ns.load() < active->declared_end_ns;
    });
  });
  started.wait();
  fake_now_ns.store(active->declared_end_ns);
  release_holder.set_value();
  EXPECT_TRUE(holder.get());
  EXPECT_EQ(waiter.get(), navigation_execution::CommitDecision::kAdmissionRejected);
  EXPECT_EQ(predicate_calls.load(), 1);
  expectUnchanged(before);
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, before, replacement,
                                    [] { return true; }),
            navigation_execution::CommitDecision::kCommitted);
}

TEST_F(ConditionalCommit, PredicateExceptionIsRejectedWithoutMutation) {
  const auto before = store.snapshot();
  const auto replacement = makeCandidate(28);
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, before, replacement,
                                    []() -> bool {
                                      throw std::runtime_error("clock failure");
                                    }),
            navigation_execution::CommitDecision::kAdmissionRejected);
  expectUnchanged(before);
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, before, replacement,
                                    [] { return true; }),
            navigation_execution::CommitDecision::kCommitted);
}

TEST_F(ConditionalCommit, SameGoalNewerActiveOwnerRejectsBeforePredicate) {
  const auto observed = store.snapshot();
  const auto newer = makeCandidate(28);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, newer),
            navigation_execution::CommitDecision::kCommitted);
  const auto before = store.snapshot();
  bool predicate_called = false;
  // This token is NEWER than the winning commit: cancellation is not the
  // discriminator. The exact predecessor itself must be checked.
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 3}, observed, makeCandidate(29), [&] {
              predicate_called = true;
              return true;
            }), navigation_execution::CommitDecision::kPredecessorAdvanced);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(before);
}

TEST_F(ConditionalCommit, PendingOnlyMutationRejectsBeforePredicate) {
  const auto observed = store.snapshot();
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  const auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  const auto before = store.snapshot();
  bool predicate_called = false;
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 3}, observed, makeCandidate(29), [&] {
              predicate_called = true;
              return true;
            }), navigation_execution::CommitDecision::kPredecessorAdvanced);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(before);
}

TEST_F(ConditionalCommit, CurrentPendingCanBeReplacedOnlyWithExplicitAdmission) {
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kStaged);
  const auto before = store.snapshot();
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 3}, before, makeCandidate(29),
                                    [] { return false; }),
            navigation_execution::CommitDecision::kAdmissionRejected);
  expectUnchanged(before);
  const auto replacement = makeCandidate(29);
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 3}, before, replacement,
                                    [] { return true; }),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_FALSE(store.snapshot().pending);
}

TEST_F(ConditionalCommit, WorldGoalAndTokenGatesDoNotInvokePredicate) {
  const auto observed = store.snapshot();
  bool predicate_called = false;
  auto predicate = [&] {
    predicate_called = true;
    return true;
  };
  auto wrong_world = world;
  ++wrong_world.revision;
  ++wrong_world.observation_stamp_ns;
  EXPECT_EQ(store.tryCommitIfCurrent({wrong_world, 7, 2}, observed,
                                    makeCandidate(28), predicate),
            navigation_execution::CommitDecision::kWorldAdvanced);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(observed);
  EXPECT_EQ(store.tryCommitIfCurrent({world, 8, 2}, observed,
                                    makeCandidate(28), predicate),
            navigation_execution::CommitDecision::kGoalAdvanced);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(observed);
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 1}, observed,
                                    makeCandidate(28), predicate),
            navigation_execution::CommitDecision::kCancelled);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(observed);
}

TEST_F(ConditionalCommit, ActualWorldAdvanceRejectsBeforeStalePredecessor) {
  const auto observed = store.snapshot();
  auto advanced = world;
  ++advanced.revision;
  ++advanced.observation_stamp_ns;
  ASSERT_TRUE(publishWorldIdentityForTest(store, advanced, active, true));
  const auto before = store.snapshot();
  bool predicate_called = false;
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, observed, makeCandidate(28), [&] {
              predicate_called = true;
              return true;
            }), navigation_execution::CommitDecision::kWorldAdvanced);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(before);
}

TEST_F(ConditionalCommit, ActualGoalAdvanceRejectsBeforeStalePredecessor) {
  const auto observed = store.snapshot();
  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  const auto before = store.snapshot();
  bool predicate_called = false;
  EXPECT_EQ(store.tryCommitIfCurrent({world, 7, 2}, observed, makeCandidate(28), [&] {
              predicate_called = true;
              return true;
            }), navigation_execution::CommitDecision::kGoalAdvanced);
  EXPECT_FALSE(predicate_called);
  expectUnchanged(before);
}

TEST_F(ConditionalCommit, LegacyImmediateCommitHasNoPredecessorOrDeadlineFence) {
  // Control demonstrating why callers that prepare a phase-bound replacement
  // cannot use tryCommit(): it intentionally has no expected timeline/clock.
  const auto originally_observed = store.snapshot();
  const auto newer = makeCandidate(28);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, newer),
            navigation_execution::CommitDecision::kCommitted);
  const std::int64_t fake_now_ns = active->declared_end_ns;
  ASSERT_GE(fake_now_ns, originally_observed.active->declared_end_ns);
  const auto late = makeCandidate(29);
  EXPECT_EQ(store.tryCommit({world, 7, 3}, late),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_EQ(store.load(), late);
}

TEST(TestExecutionAuthority, FinalizerFailureRestoresPreviousExecutionPointer) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto previous = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, previous),
            navigation_execution::CommitDecision::kCommitted);

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  EXPECT_EQ(store.tryCommitAndFinalize(
                {world, 7, 2}, replacement, [] { return false; }),
            navigation_execution::CommitDecision::kFinalizationFailed);
  EXPECT_EQ(store.load(), previous);

  auto after_rollback = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  EXPECT_EQ(store.tryCommitAndFinalize(
                {world, 7, 2}, after_rollback, [] { return true; }),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_EQ(store.load(), after_rollback);
}

TEST(TestExecutionAuthority, SuccessfulFinalizerKeepsReplacementPointer) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  bool finalized = false;
  EXPECT_EQ(store.tryCommitAndFinalize(
                {world, 7, 2}, replacement, [&finalized] {
                  finalized = true;
                  return true;
                }),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_TRUE(finalized);
  EXPECT_EQ(store.load(), replacement);
}

TEST(TestExecutionAuthority, GoalReplacementInvalidatesAndSamplerDoesNotLockWorld) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(store);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50)));
  ASSERT_TRUE(store.setActiveGoalEpoch(8));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50)));
}

TEST(CommandSampler, RejectsRetainedBundleFromPreviousGoal) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  // Hot-retarget may retain the physical pointer while the new goal is
  // waiting for a fresh solve.  The sampler must not expose or relabel it.
  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  navigation_execution::CommandSampler sampler(store);
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 8)));
}

TEST(TestExecutionAuthority, RetainedCommandStaysOldUntilSuccessorActivation) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  navigation_execution::CommandSampler sampler(store);
  const auto retained = store.load();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->goal_epoch, 7U);
  EXPECT_EQ(retained->request_id, 17U);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 7)));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 8)));

  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = candidateFor(8, 1);
  successor.request_id = 20U;
  successor.valid_from_ns = 50;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(successor);
  ASSERT_EQ(navigation_execution::candidateMatchesAnchor(*successor_ptr, *anchor),
            navigation_execution::AnchorMatchResult::kMatch);
  ASSERT_EQ(store.stagePending({world, 8, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(store.load(), retained);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 7)));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 8)));

  const auto activation = store.snapshot();
  ASSERT_TRUE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return true; }));
  EXPECT_EQ(store.load(), successor_ptr);
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 7)));
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 8)));
}

TEST(ExecutionAnchor, CandidateMatchConvertsEvaluatorExceptionToNoSample) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  auto successor = candidateFor(8, 1);
  successor.valid_from_ns = anchor->activation_stamp_ns;
  successor.activation_stamp_ns = anchor->activation_stamp_ns;
  successor.evaluator = [](
                            std::int64_t,
                            navigation_planning::TrajectoryPoint&) -> bool {
    throw std::runtime_error("synthetic anchor-match evaluator failure");
  };

  EXPECT_EQ(navigation_execution::candidateMatchesAnchor(successor, *anchor),
            navigation_execution::AnchorMatchResult::kNoSample);
}

TEST(TestExecutionAuthority, RejectsSuccessorWhenPredecessorAdvanced) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  auto replacement_data = candidateFor(7, 1);
  replacement_data.bundle_generation = active->bundle_generation + 1U;
  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      replacement_data);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_EQ(store.stagePending({world, 7, 3}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kPredecessorAdvanced);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority, RejectsSuccessorAfterSameIdentityPredecessorReplacement) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  auto replacement_data = candidateFor(7, 1);
  replacement_data.evaluator = [](
                                   std::int64_t stamp,
                                   navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp + 10);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      std::move(replacement_data));
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_EQ(store.stagePending({world, 7, 3}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kPredecessorAdvanced);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_FALSE(store.snapshot().pending);
}

TEST(TestExecutionAuthority, RevokedAnchorCannotBeRevivedByRecommittingSamePointer) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  store.invalidate();
  ASSERT_EQ(store.tryCommit({world, 7, 2}, active),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_EQ(store.stagePending({world, 7, 3}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kPredecessorAdvanced);
  EXPECT_EQ(store.load(), active);
  EXPECT_FALSE(store.snapshot().pending);
}

TEST(TestExecutionAuthority, RejectsFinalizedSuccessorWhenPredecessorAdvanced) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  auto replacement_data = candidateFor(7, 1);
  replacement_data.bundle_generation = active->bundle_generation + 1U;
  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      replacement_data);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);

  bool finalized = false;
  EXPECT_EQ(store.stagePendingAndFinalize(
                {world, 7, 3}, *anchor, successorFor(*anchor, 7),
                [&finalized] {
                  finalized = true;
                  return true;
                }),
            navigation_execution::StageDecision::kPredecessorAdvanced);
  EXPECT_FALSE(finalized);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority, FailedPendingFinalizationRestoresPriorPending) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);

  const auto first_anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(first_anchor);
  auto first_pending = successorFor(*first_anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *first_anchor, first_pending),
            navigation_execution::StageDecision::kStaged);

  const auto replacement_anchor = store.reserveAnchor(60, 70);
  ASSERT_TRUE(replacement_anchor);
  auto replacement_pending = successorFor(*replacement_anchor, 7);
  EXPECT_EQ(store.stagePendingAndFinalize(
                {world, 7, 3}, *replacement_anchor, replacement_pending,
                [] { return false; }),
            navigation_execution::StageDecision::kFinalizationFailed);
  const auto failed_snapshot = store.snapshot();
  EXPECT_EQ(failed_snapshot.pending, first_pending);
  EXPECT_EQ(failed_snapshot.pending_activation_ns, 50);
  EXPECT_TRUE(store.activatePendingIfDueAndFinalize(
      50, failed_snapshot, [](std::uint64_t) { return true; }));
  EXPECT_EQ(store.load(), first_pending);
}

TEST(TestExecutionAuthority, AcceptsAnchorAtExactMainBackupBoundary) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active_data = candidateFor(7, 1);
  active_data.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  active_data.backup_available = true;
  active_data.backup_start_time_s = 49.0e-9;
  active_data.role_schedule = {
      {0.0, 49.0e-9, navigation_planning::CandidateRole::kMain},
      {49.0e-9, 399.0e-9, navigation_planning::CandidateRole::kBackup}};
  active_data.evaluator = [](std::int64_t stamp,
                             navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    point.role = stamp >= 50 ? navigation_planning::CandidateRole::kBackup
                             : navigation_planning::CandidateRole::kMain;
    return true;
  };
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(active_data);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);

  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  EXPECT_EQ(anchor->active_role, navigation_planning::CandidateRole::kBackup);
  EXPECT_EQ(store.stagePending({world, 7, 2}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kStaged);
}

TEST(TestExecutionAuthority, RecertifiedPredecessorKeepsSuccessorActivationValid) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor),
            navigation_execution::StageDecision::kStaged);
  const auto old_timeline = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};

  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, true, 300,
                old_timeline.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_NE(store.load(), active);
  EXPECT_TRUE(static_cast<bool>(store.snapshot().pending));
  const auto activation = store.snapshot();
  EXPECT_TRUE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return true; }));
  EXPECT_EQ(store.load()->bundle_generation, successor->bundle_generation);
}

TEST(TestExecutionAuthority, ActiveInvalidPendingValidImmediatelyFailsClosed) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor),
            navigation_execution::StageDecision::kStaged);
  const auto old_timeline = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};

  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, false, 0,
                old_timeline.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_FALSE(store.load());
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
  EXPECT_TRUE(store.invariantHolds());
  const auto activation = store.snapshot();
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return true; }));
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority, PendingImpliesActiveAfterEveryStoreMutation) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  const auto assert_invariant = [&store] {
    const auto timeline = store.snapshot();
    EXPECT_TRUE(store.invariantHolds());
    EXPECT_TRUE(!timeline.pending || static_cast<bool>(timeline.active));
  };

  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  assert_invariant();
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  assert_invariant();
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  assert_invariant();
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  assert_invariant();
  const auto observed = store.snapshot();
  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, observed.version, observed.active, true, 300,
                observed.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);
  assert_invariant();
  const auto refreshed = store.snapshot();
  ASSERT_TRUE(store.activatePendingIfDueAndFinalize(
      50, refreshed, [](std::uint64_t) { return true; }));
  assert_invariant();
  store.invalidate();
  assert_invariant();
}

TEST(CommandSampler, RetainsFutureBundleUntilItsSampleValidityBoundary) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  std::size_t evaluations = 0;
  auto candidate = candidateFor(7, 1);
  candidate.valid_from_ns = 100;
  candidate.valid_until_ns = 200;
  candidate.activation_stamp_ns = 100;
  candidate.start_wall_time_s = 100.0e-9;
  candidate.duration_s = 300.0e-9;
  candidate.declared_start_ns = 100;
  candidate.declared_end_ns = 400;
  candidate.role_schedule = {
      {0.0, 300.0e-9, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [&evaluations](
      std::int64_t stamp, navigation_planning::TrajectoryPoint& point) {
    ++evaluations;
    point.position_world.x() = static_cast<double>(stamp);
    return true;
  };
  auto committed = std::make_shared<const navigation_planning::CandidateBundle>(candidate);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, committed),
            navigation_execution::CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(store);
  const auto before_activation = sampler.sample(99);
  EXPECT_FALSE(static_cast<bool>(before_activation));
  ASSERT_TRUE(before_activation.bundle);
  EXPECT_TRUE(before_activation.awaiting_activation);
  EXPECT_EQ(before_activation.status,
            navigation_execution::SampleStatus::kAwaitingActivation);
  EXPECT_EQ(evaluations, 0U);

  const auto active = sampler.sample(100);
  ASSERT_TRUE(static_cast<bool>(active));
  EXPECT_FALSE(active.awaiting_activation);
  EXPECT_EQ(active.status, navigation_execution::SampleStatus::kActiveSample);
  EXPECT_EQ(evaluations, 1U);
  EXPECT_DOUBLE_EQ(active.point->position_world.x(), 100.0);

  const auto expired = sampler.sample(201);
  EXPECT_FALSE(static_cast<bool>(expired));
  ASSERT_TRUE(expired.bundle);
  EXPECT_FALSE(expired.awaiting_activation);
  EXPECT_EQ(expired.status, navigation_execution::SampleStatus::kExpiredLease);
  EXPECT_EQ(evaluations, 1U);

  const auto planned_hold = sampler.sample(401);
  ASSERT_TRUE(static_cast<bool>(planned_hold));
  EXPECT_TRUE(planned_hold.planned_stop_hold);
  EXPECT_EQ(planned_hold.status, navigation_execution::SampleStatus::kStoppedHold);
  EXPECT_TRUE(planned_hold.point->finished);
  // The sampler returns the endpoint only as a typed hold fallback; it does
  // not evaluate the expired command again.
  EXPECT_EQ(evaluations, 2U);
}

TEST(CommandSampler, SamplesDeclaredMainToBackupBundleAcrossRoleBoundary) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = candidateFor(7, 1);
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.backup_start_time_s = 49.0e-9;
  candidate.role_schedule = {
      {0.0, 49.0e-9, navigation_planning::CandidateRole::kMain},
      {49.0e-9, 399.0e-9, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.role = stamp < 50
                     ? navigation_planning::CandidateRole::kMain
                     : navigation_planning::CandidateRole::kBackup;
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  auto committed =
      std::make_shared<const navigation_planning::CandidateBundle>(candidate);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, committed),
            navigation_execution::CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(store);
  const auto main = sampler.sample(49, 7);
  ASSERT_TRUE(static_cast<bool>(main));
  EXPECT_EQ(main.point->role, navigation_planning::CandidateRole::kMain);
  const auto backup = sampler.sample(50, 7);
  ASSERT_TRUE(static_cast<bool>(backup));
  EXPECT_EQ(backup.point->role, navigation_planning::CandidateRole::kBackup);

  const auto hold = sampler.sample(candidate.declared_end_ns + 1, 7);
  ASSERT_TRUE(hold);
  EXPECT_TRUE(hold.planned_stop_hold);
  EXPECT_TRUE(hold.point->finished);
  EXPECT_EQ(hold.point->role, navigation_planning::CandidateRole::kBackup);
}

TEST(CommandSampler, ExpiredEndpointCannotForgeBackupRoleAgainstSchedule) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = candidateFor(7, 1);
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.backup_start_time_s = candidate.duration_s;
  // Construction permits MAIN up to the end; evaluator cannot invent BACKUP.
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    point.role = navigation_planning::CandidateRole::kBackup;
    return true;
  };
  ASSERT_EQ(store.tryCommit({world, 7, 1},
      std::make_shared<const navigation_planning::CandidateBundle>(candidate)),
      navigation_execution::CommitDecision::kCommitted);
  const navigation_execution::CommandSampler sampler(store);
  const auto hold = sampler.sample(candidate.declared_end_ns + 1, 7);
  EXPECT_FALSE(hold);
  EXPECT_FALSE(hold.planned_stop_hold);
}

TEST(TestExecutionAuthority, StagesSuccessorUntilFutureAnchorActivation) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);

  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(navigation_execution::candidateMatchesAnchor(*successor_ptr, *anchor),
            navigation_execution::AnchorMatchResult::kMatch);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(store.load(), active);
  EXPECT_EQ(store.snapshot().pending, successor_ptr);

  navigation_execution::CommandSampler sampler(store);
  EXPECT_EQ(sampler.sample(49, 7).bundle, active);
  const auto activation = store.snapshot();
  ASSERT_TRUE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return true; }));
  const auto activated = sampler.sample(50, 7);
  ASSERT_TRUE(activated);
  EXPECT_EQ(activated.bundle, successor_ptr);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority, RenewsSuccessorsWithoutAnExecutionPointerGap) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto initial = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, initial),
            navigation_execution::CommitDecision::kCommitted);
  navigation_execution::CommandSampler sampler(store);

  const auto first_anchor = store.reserveAnchor(20, 50);
  ASSERT_TRUE(first_anchor);
  auto first_successor = candidateFor(7, 1);
  first_successor.bundle_generation = 28;
  first_successor.valid_from_ns = 50;
  first_successor.valid_until_ns = 90;
  first_successor.activation_stamp_ns = 50;
  auto first_successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      first_successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *first_anchor, first_successor_ptr),
            navigation_execution::StageDecision::kStaged);
  ASSERT_EQ(store.load(), initial);
  const auto activation = store.snapshot();
  ASSERT_TRUE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return true; }));
  ASSERT_EQ(store.load(), first_successor_ptr);

  const auto second_anchor = store.reserveAnchor(60, 70);
  ASSERT_TRUE(second_anchor);
  auto second_successor = first_successor;
  second_successor.bundle_generation = 29;
  second_successor.valid_from_ns = 70;
  second_successor.valid_until_ns = 95;
  second_successor.activation_stamp_ns = 70;
  auto second_successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      second_successor);
  ASSERT_EQ(store.stagePending({world, 7, 3}, *second_anchor, second_successor_ptr),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(store.load(), first_successor_ptr);
  EXPECT_TRUE(static_cast<bool>(store.snapshot().pending));
  const auto second_activation = store.snapshot();
  ASSERT_TRUE(store.activatePendingIfDueAndFinalize(
      70, second_activation, [](std::uint64_t) { return true; }));
  EXPECT_EQ(store.load(), second_successor_ptr);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));

  const auto sampled = sampler.sample(70, 7);
  ASSERT_TRUE(sampled);
  EXPECT_EQ(sampled.bundle, second_successor_ptr);
  EXPECT_EQ(sampled.point->position_world.x(), 70.0);
}

TEST(TestExecutionAuthority, WorldAdvanceInvalidatesPendingSuccessor) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  ASSERT_TRUE(publishWorldIdentityForTest(store, next_world));
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
  EXPECT_FALSE(store.load());
}

TEST(TestExecutionAuthority, RejectsPendingRecertificationWithoutRetainedActive) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  const auto old_timeline = store.snapshot();
  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, false, 0,
                old_timeline.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);
  const auto recertified_snapshot = store.snapshot();
  EXPECT_FALSE(recertified_snapshot.active);
  EXPECT_FALSE(recertified_snapshot.pending);
  EXPECT_TRUE(store.invariantHolds());
}

TEST(TestExecutionAuthority, ActivationFinalizesPlannerOnlyAtSwapBoundary) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  std::uint64_t finalized_generation = 0;
  const auto activation = store.snapshot();
  EXPECT_TRUE(store.activatePendingIfDueAndFinalize(
      50, activation, [&](const std::uint64_t generation) {
    finalized_generation = generation;
    return true;
  }));
  EXPECT_EQ(finalized_generation, successor_ptr->bundle_generation);
  EXPECT_EQ(store.load(), successor_ptr);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority,
     ActivationRejectsStaleExpectedPendingWithoutDroppingNewerCandidate) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());

  auto first = candidateFor(7, 1);
  first.bundle_generation = 28;
  first.valid_from_ns = 50;
  first.valid_until_ns = 90;
  first.activation_stamp_ns = 50;
  const auto first_ptr = std::make_shared<const navigation_planning::CandidateBundle>(first);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, first_ptr),
            navigation_execution::StageDecision::kStaged);
  const auto first_timeline = store.snapshot();

  auto newer = first;
  newer.bundle_generation = 29;
  const auto newer_ptr = std::make_shared<const navigation_planning::CandidateBundle>(newer);
  ASSERT_EQ(store.stagePending({world, 7, 3}, *anchor, newer_ptr),
            navigation_execution::StageDecision::kStaged);

  // Model a publisher that captured first_ptr before a newer pending candidate
  // won the store mutex. The stale activation must be a no-op, including its
  // finalizer, and must preserve the newer pending pointer.
  bool finalized = false;
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      50, first_timeline, [&](const std::uint64_t) {
        finalized = true;
        return true;
      }));
  EXPECT_FALSE(finalized);
  EXPECT_EQ(store.load(), active);
  EXPECT_EQ(store.snapshot().pending, newer_ptr);

  auto activation_mismatch = store.snapshot();
  ++activation_mismatch.pending_activation_ns;
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      50, activation_mismatch, [&](const std::uint64_t) {
        finalized = true;
        return true;
      }));
  EXPECT_FALSE(finalized);
  EXPECT_EQ(store.snapshot().pending, newer_ptr);
}

TEST(TestExecutionAuthority, ActivationFinalizerFailureKeepsOldAndDropsPending) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  const auto activation = store.snapshot();
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return false; }));
  EXPECT_EQ(store.load(), active);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority, MissedActivationKeepsActiveCommandAndDropsSuccessor) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 60;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  // Pending activation is an execution-store transition owned by the command
  // publisher; CommandSampler must remain read-only.
  const auto activation = store.snapshot();
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      61, activation, [](std::uint64_t) { return true; }));
  navigation_execution::CommandSampler sampler(store);
  const auto missed = sampler.sample(61, 7);
  ASSERT_TRUE(missed);
  EXPECT_EQ(missed.bundle, active);
  EXPECT_EQ(store.load(), active);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));
}

TEST(TestExecutionAuthority, FinalizerFailureRestoresPendingTransactionWatermark) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  EXPECT_EQ(store.stagePendingAndFinalize(
                {world, 7, 2}, *anchor, successor_ptr, [] { return false; }),
            navigation_execution::StageDecision::kFinalizationFailed);
  EXPECT_FALSE(static_cast<bool>(store.snapshot().pending));

  auto retry = std::make_shared<const navigation_planning::CandidateBundle>(successor);
  EXPECT_EQ(store.stagePending({world, 7, 2}, *anchor, retry),
            navigation_execution::StageDecision::kStaged);
}

TEST(TestExecutionAuthority, ExposureRejectsBundleInvalidatedAfterSampling) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  const auto sampled = store.load();
  ASSERT_EQ(sampled.get(), candidate.get());
  auto advanced = world;
  advanced.revision = 2;
  advanced.observation_stamp_ns = 2;
  ASSERT_TRUE(publishWorldIdentityForTest(store, advanced));

  bool exposed = false;
  EXPECT_FALSE(store.publishIfCurrent(sampled, 7, [&] {
    exposed = true;
    return true;
  }));
  EXPECT_FALSE(exposed);
}

TEST(TestExecutionAuthority, ExposureKeepsRetainedExecutionBundleUntilActivation) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  const auto sampled = store.load();
  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  bool exposed = false;
  EXPECT_TRUE(store.publishIfCurrent(sampled, 7, [&] {
    exposed = true;
    return true;
  }));
  EXPECT_TRUE(exposed);
}

TEST(TestExecutionAuthority, ExposureMustRecheckFreshnessAfterWaitingForStoreLock) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);
  const auto sampled = store.load();

  std::promise<void> holder_entered;
  std::promise<void> release_holder;
  auto release = release_holder.get_future().share();
  auto holder = std::async(std::launch::async, [&] {
    return store.publishIfCurrent(sampled, 7, [&] {
      holder_entered.set_value();
      release.wait();
      return true;
    });
  });
  holder_entered.get_future().wait();

  constexpr std::int64_t freshness_limit_ns = 10;
  std::atomic<std::int64_t> fake_now_ns{6};
  ASSERT_EQ(navigation_execution::classifyTimestampFreshness(
                fake_now_ns.load(), world.observation_stamp_ns,
                freshness_limit_ns),
            navigation_execution::TimestampFreshness::VALID);
  std::atomic_bool exposed{false};
  auto waiter = std::async(std::launch::async, [&] {
    return store.publishIfCurrent(sampled, 7, [&] {
      const bool fresh = navigation_execution::classifyTimestampFreshness(
          fake_now_ns.load(), world.observation_stamp_ns,
          freshness_limit_ns) == navigation_execution::TimestampFreshness::VALID;
      exposed.store(fresh);
      return fresh;
    });
  });
  fake_now_ns.store(12);
  release_holder.set_value();

  EXPECT_TRUE(holder.get());
  EXPECT_FALSE(waiter.get());
  EXPECT_FALSE(exposed.load());
}

TEST(TestExecutionAuthority, RecertifiesOnlyTheValidatedBundleOnWorldAdvance) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_TRUE(publishWorldIdentityForTest(store, next_world, candidate, true));
  const auto recertified = store.load();
  ASSERT_TRUE(recertified);
  EXPECT_NE(recertified.get(), candidate.get());
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      recertified->world_identity, next_world));
  EXPECT_TRUE(store.publishIfCurrent(recertified, 7, [] { return true; }));
}

TEST(TestExecutionAuthority, RecertificationRenewsOnlyTheValidatedExecutionWindow) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = candidateFor(7, 1);
  candidate.valid_until_ns = 100;
  auto committed = std::make_shared<const navigation_planning::CandidateBundle>(candidate);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, committed),
            navigation_execution::CommitDecision::kCommitted);

  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_TRUE(publishWorldIdentityForTest(store, next_world, committed, true, 300));
  const auto recertified = store.load();
  ASSERT_TRUE(recertified);
  EXPECT_EQ(recertified->valid_until_ns, 300);
  EXPECT_TRUE(store.publishIfCurrent(recertified, 7, [] { return true; }));
}

TEST(TestExecutionAuthority, RecertificationMismatchOrGoalChangeClearsBundle) {
  navigation_execution::TestExecutionAuthority store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  const auto unrelated_bundle = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 99));
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_TRUE(publishWorldIdentityForTest(store, next_world, unrelated_bundle, true));
  EXPECT_FALSE(store.load());
}

TEST(TestExecutionAuthority, SupersededWorldRefreshPreservesNewCommit) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto first = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, first),
            navigation_execution::CommitDecision::kCommitted);
  const auto old_timeline = store.snapshot();

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 2));
  auto replacement_copy = std::make_shared<navigation_planning::CandidateBundle>(*replacement);
  replacement_copy->world_identity = world;
  replacement_copy->pinned_world_identity = world;
  replacement = std::shared_ptr<const navigation_planning::CandidateBundle>(
      std::move(replacement_copy));
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  EXPECT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, true, 300),
            navigation_world_model::WorldCommitDecision::kSuperseded);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_EQ(store.snapshot().world_identity->revision, world.revision);
}

TEST(TestExecutionAuthority, ActiveValidPendingInvalidKeepsActive) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = candidateFor(7, 1);
  pending.valid_from_ns = 50;
  pending.valid_until_ns = 90;
  pending.activation_stamp_ns = 50;
  auto pending_ptr = std::make_shared<const navigation_planning::CandidateBundle>(pending);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending_ptr),
            navigation_execution::StageDecision::kStaged);
  const auto old_timeline = store.snapshot();
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};

  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, true, 300,
                old_timeline.pending, false),
            navigation_world_model::WorldCommitDecision::kCommitted);
  ASSERT_TRUE(store.load());
  EXPECT_NE(store.load(), active);
  EXPECT_EQ(store.load()->valid_from_ns, active->valid_from_ns);
  EXPECT_FALSE(store.snapshot().pending);
  EXPECT_EQ(store.snapshot().world_identity->revision, next_world.revision);
}

TEST(TestExecutionAuthority, PreparationFailureRevokesExactActiveAndRetriesWorld) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  const auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto expected = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  unsigned int finalized = 0;
  const auto fail_preparation = [](
      const navigation_planning::CandidateBundle&,
      const navigation_world_model::WorldSnapshotIdentity&, std::int64_t)
      -> std::shared_ptr<const navigation_planning::CandidateBundle> {
    throw std::bad_alloc();
  };

  EXPECT_EQ(navigation_execution::ExecutionTimelineStoreTestAccess::publish(
                store, next_world, expected, true, 0, false,
                [&]() noexcept { ++finalized; }, fail_preparation),
            navigation_world_model::WorldCommitDecision::kCandidateRejected);
  EXPECT_EQ(finalized, 1U);
  auto after_failure = store.snapshot();
  EXPECT_FALSE(after_failure.active);
  EXPECT_FALSE(after_failure.pending);
  ASSERT_TRUE(after_failure.world_identity);
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      *after_failure.world_identity, world));

  EXPECT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, after_failure.version, after_failure.active, false, 0,
                after_failure.pending, false),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      *store.snapshot().world_identity, next_world));
}

TEST(TestExecutionAuthority, PendingPreparationFailureDoesNotRevokeActive) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  const auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  const auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  const auto expected = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  unsigned int finalized = 0;
  const auto fail_pending_preparation = [pending] (
      const navigation_planning::CandidateBundle& source,
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const std::int64_t) -> std::shared_ptr<const navigation_planning::CandidateBundle> {
    if (source.bundle_generation == pending->bundle_generation) throw std::bad_alloc();
    auto copy = std::make_shared<navigation_planning::CandidateBundle>(source);
    copy->world_identity = identity;
    return copy;
  };

  EXPECT_EQ(navigation_execution::ExecutionTimelineStoreTestAccess::publish(
                store, next_world, expected, true, 0, true,
                [&]() noexcept { ++finalized; }, fail_pending_preparation),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_EQ(finalized, 0U);
  const auto after = store.snapshot();
  ASSERT_TRUE(after.active);
  EXPECT_NE(after.active, active);
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      after.active->world_identity, next_world));
  EXPECT_FALSE(after.pending);
}

TEST(TestExecutionAuthority, SnapshotSupersededDuringPreparationIsNoOp) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  const auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto expected = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  auto replacement_data = candidateFor(7, 1);
  ++replacement_data.bundle_generation;
  const auto replacement =
      std::make_shared<const navigation_planning::CandidateBundle>(replacement_data);
  unsigned int finalized = 0;
  const auto superseding_prepare = [&] (
      const navigation_planning::CandidateBundle& source,
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const std::int64_t) -> std::shared_ptr<const navigation_planning::CandidateBundle> {
    EXPECT_EQ(store.tryCommit({world, 7, 2}, replacement),
              navigation_execution::CommitDecision::kCommitted);
    auto copy = std::make_shared<navigation_planning::CandidateBundle>(source);
    copy->world_identity = identity;
    return copy;
  };

  EXPECT_EQ(navigation_execution::ExecutionTimelineStoreTestAccess::publish(
                store, next_world, expected, true, 0, false,
                [&]() noexcept { ++finalized; }, superseding_prepare),
            navigation_world_model::WorldCommitDecision::kSuperseded);
  EXPECT_EQ(finalized, 0U);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      *store.snapshot().world_identity, world));
}

TEST(TestExecutionAuthority, StaleRevokePreservesReplacementActiveBundle) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto observed = store.snapshot();
  auto replacement_data = candidateFor(7, 1);
  replacement_data.bundle_generation = active->bundle_generation + 1U;
  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      replacement_data);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_FALSE(store.invalidateIfCurrent(observed));
  EXPECT_EQ(store.load(), replacement);
}

TEST(TestExecutionAuthority, StaleRevokePreservesNewPendingSuccessor) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto observed = store.snapshot();
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);

  EXPECT_FALSE(store.invalidateIfCurrent(observed));
  EXPECT_EQ(store.load(), active);
  EXPECT_EQ(store.snapshot().pending, pending);
}

TEST(TestExecutionAuthority, ActivationWinsAgainstStaleWorldRefresh) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  const auto observed = store.snapshot();

  const auto activation = store.snapshot();
  ASSERT_TRUE(store.activatePendingIfDueAndFinalize(
      50, activation, [](std::uint64_t) { return true; }));
  EXPECT_FALSE(store.invalidateIfCurrent(observed));
  EXPECT_EQ(store.load(), pending);
}

TEST(TestExecutionAuthority, RevokeWinsAndBlocksPendingReexposure) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  const auto observed = store.snapshot();

  EXPECT_TRUE(store.invalidateIfCurrent(observed));
  EXPECT_FALSE(store.load());
  EXPECT_FALSE(store.snapshot().pending);
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      50, store.snapshot(), [](std::uint64_t) { return true; }));
}

TEST(TestExecutionAuthority, StaleRevokePreservesRecertifiedPendingSuccessor) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  const auto observed = store.snapshot();
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, observed.version, observed.active, true, 0,
                observed.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);

  EXPECT_FALSE(store.invalidateIfCurrent(observed));
  EXPECT_NE(store.load(), active);
  const auto pending_snapshot = store.snapshot();
  ASSERT_TRUE(pending_snapshot.pending);
  EXPECT_EQ(pending_snapshot.pending->world_identity.revision, next_world.revision);
}

TEST(TestExecutionAuthority, StaleRevokePreservesActiveAfterExpiredPendingIsDropped) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending_data = *successorFor(*anchor, 7);
  pending_data.valid_until_ns = 100;
  auto pending = std::make_shared<const navigation_planning::CandidateBundle>(pending_data);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending),
            navigation_execution::StageDecision::kStaged);
  const auto observed = store.snapshot();

  const auto activation = store.snapshot();
  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(
      101, activation, [](std::uint64_t) { return true; }));
  EXPECT_FALSE(store.invalidateIfCurrent(observed));
  EXPECT_EQ(store.load(), active);
  EXPECT_FALSE(store.snapshot().pending);
}

TEST(TestExecutionAuthority, ReserveAnchorDoesNotHoldStoreLockDuringEvaluation) {
  using namespace std::chrono_literals;
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto evaluator_entered = std::make_shared<std::promise<void>>();
  auto evaluator_release = std::make_shared<std::promise<void>>();
  const auto evaluator_entered_future = evaluator_entered->get_future();
  const auto evaluator_release_future = evaluator_release->get_future().share();
  auto candidate_data = candidateFor(7, 1);
  candidate_data.evaluator = [evaluator_entered, evaluator_release_future](
                                 const std::int64_t stamp,
                                 navigation_planning::TrajectoryPoint& point) {
    evaluator_entered->set_value();
    evaluator_release_future.wait();
    point.position_world.x() = static_cast<double>(stamp);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      std::move(candidate_data));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  auto anchor_future = std::async(std::launch::async, [&store]() {
    return store.reserveAnchor(50, 50);
  });
  const auto evaluator_status = evaluator_entered_future.wait_for(1s);
  if (evaluator_status != std::future_status::ready) {
    evaluator_release->set_value();
    EXPECT_EQ(evaluator_status, std::future_status::ready);
    return;
  }
  auto invalidation_started = std::make_shared<std::promise<void>>();
  const auto invalidation_started_future = invalidation_started->get_future();
  auto invalidation_future = std::async(
      std::launch::async, [&store, invalidation_started]() {
        invalidation_started->set_value();
        store.invalidate();
      });
  const auto invalidation_started_status = invalidation_started_future.wait_for(1s);
  if (invalidation_started_status != std::future_status::ready) {
    evaluator_release->set_value();
    invalidation_future.get();
    static_cast<void>(anchor_future.get());
    EXPECT_EQ(invalidation_started_status, std::future_status::ready);
    return;
  }
  const auto invalidation_status = invalidation_future.wait_for(250ms);

  evaluator_release->set_value();
  invalidation_future.get();
  const auto anchor = anchor_future.get();
  EXPECT_EQ(invalidation_status, std::future_status::ready);
  EXPECT_FALSE(anchor);
}

TEST(TestExecutionAuthority, ReserveAnchorConvertsEvaluatorExceptionToFailure) {
  navigation_execution::TestExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate_data = candidateFor(7, 1);
  candidate_data.evaluator = [](
                                 std::int64_t,
                                 navigation_planning::TrajectoryPoint&) -> bool {
    throw std::runtime_error("synthetic evaluator failure");
  };
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      std::move(candidate_data));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_FALSE(store.reserveAnchor(50, 50));
  EXPECT_EQ(store.load(), candidate);
}

TEST(ExecutionStateStore, RejectsOldEpochAndClearsStateOnReset) {
  navigation_execution::ExecutionStateStore store;
  navigation_planning::KinematicState state;
  state.position_world.x() = 1.0;
  state.source_stamp_ns = 10;
  state.receive_stamp_ns = 20;
  state.localization_epoch = 4;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(store.publish(state));
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.load()->ingress_sequence, 1U);
  EXPECT_FALSE(store.publish(state));
  store.resetForLocalizationEpoch(5);
  EXPECT_FALSE(store.load());
  state.localization_epoch = 4;
  state.source_stamp_ns = 11;
  EXPECT_FALSE(store.publish(state));
  state.localization_epoch = 5;
  EXPECT_TRUE(store.publish(state));
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.load()->ingress_sequence, 2U);
}

TEST(ExecutionStateStore, RejectsWildcardResetEpoch) {
  navigation_execution::ExecutionStateStore store;
  EXPECT_FALSE(store.resetForLocalizationEpoch(0));

  navigation_planning::KinematicState state;
  state.source_stamp_ns = 10;
  state.receive_stamp_ns = 20;
  state.localization_epoch = 4;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(store.publish(state));
  EXPECT_TRUE(store.load());
  EXPECT_TRUE(store.resetForLocalizationEpoch(5));
  EXPECT_FALSE(store.load());
}

}  // namespace
