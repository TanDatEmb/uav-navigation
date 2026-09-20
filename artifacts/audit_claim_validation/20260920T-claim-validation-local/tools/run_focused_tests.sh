#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$root/validation/install/setup.bash"
run_test() {
  local name="$1"
  local binary="$2"
  local filter="$3"
  local log="$root/validation/logs/$name"
  if [[ -n "$filter" ]]; then
    "$binary" "--gtest_filter=$filter" > "$log" 2>&1
  else
    "$binary" > "$log" 2>&1
  fi
  tail -n 14 "$log"
}
run_test h1_mission_contract_gtest.log "$root/validation/build/navigation_mission/test_mission_contract" ""
run_test h1_helper_boundary_gtest.log "$root/validation/build/navigation_runtime/test_certified_continuation" "CertifiedContinuation.ExactPlannedEntryReserveDoesNotAuthorizeLaterMeasuredHandoff"
run_test fsm_focused_gtest.log "$root/validation/build/navigation_runtime/test_planner_fsm" "PlannerFsm.ResumesOnlyExactFreshWorldRecertifiedGeneration:PlannerFsm.FiveSecondTimeoutExistsOnlyWhileStationary:PlannerFsm.WatchdogKeepsBoundedStoppedRecoveryHoldForRetry"
run_test h2_execution_episode_gtest.log "$root/validation/build/navigation_runtime/test_execution_episode" "ExecutionEpisode.SuspendAndClearDoNotRetainCommandIdentity:ExecutionEpisode.RecoveryEventsRemainOneWayInsideTheLifecycleRecord"
run_test store_focused_gtest.log "$root/validation/build/navigation_execution/test_committed_bundle_store" "CommittedBundleStore.ExposureMustRecheckFreshnessAfterWaitingForStoreLock:ExecutionTimelineStore.WorldAdvanceInvalidatesPendingSuccessor"
run_test admission_focused_gtest.log "$root/validation/build/navigation_planning/test_planning_contracts" "PlanningCandidate.AdmissionRequiresDerivedMainReserve:PlanningBudget.UsesSteadyClockAndCancellation"
run_test h5_planner_speed_governor_gtest.log "$root/validation/build/navigation_planning_backend/test_planner_config" "PlannerSpeedGovernor.*"
