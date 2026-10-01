# W3-B3 — Runtime split

Baseline: `432dc94630fbc76ca670138228f2f616f6840bb0` (`origin/main` at start).
Status: CONDITIONAL / W3-B3 split and full build/CTest pass; merge remains blocked until the required rebase after #7/A1.

## Deliverable

- MOVE `MissionProgress`, `TrajectoryCompletionWitness`, and waypoint predicates to `navigation_mission`.
- MOVE continuation/world-temporal decisions and the 27 execution predicates to `navigation_execution`.
- MOVE runtime policy headers and policy predicates to the `navigation_runtime_policy` target/include root; retain only `ordinaryRenewalFailureInjectionMayArm` in `planner_fsm.hpp` for W3-B4.
- Delete the two runtime execution alias headers; qualify owning namespaces. No safety threshold, authority, lease, UNKNOWN policy, or function body was intentionally changed.

## Verification

Commands and results:

- `git diff --check` — PASS at current checkpoint.
- `rg -n "using navigation_execution::" src/runtime/navigation_runtime/include` — no matches (header shim guard PASS).
- `navigation_runtime_policy` target dependency block — no `rclcpp` (dependency guard PASS).
- `rg -n "navigation_runtime/(desired_planning_intent|planning_supervisor|planning_key|baseline_refinement|runtime_boundaries|mission_dynamics|planning_policy)\\.hpp" src` — no stale policy include paths.
- `git submodule update --init --recursive` — PASS; initialized `px4_msgs` and `px4_ros2_interface_lib` required by the product build.
- `PARALLEL_WORKERS=2 MAKE_JOBS=2 make build` — PASS, 23 packages in 4 min 44 s; authoritative manifest written to `install/.uav_navigation_build_manifest.json`. Four packages emitted existing stderr warnings (`fast_lio_core`, `fast_lio_ros`, `fast_lio_tools`, `livox_ros_driver2`). The conservative 2/2 setting was chosen because the host had 6 GiB available RAM and 3 GiB swap in use; no 4/20 setting was used.
- The earlier `RecursionError` was reproduced before submodule initialization and disappeared after initialization; it is not a B3 code failure. The defensive provenance diagnostic/test belongs to A2 PR #2.
- Dependency-aware targeted direct colcon build through `navigation_runtime` — PASS, 10/10 packages, 4 min 20 s; the two B3 package changes (`navigation_mission`, `navigation_execution`) and runtime shell rebuilt successfully.
- Final CTest via direct colcon with `-E integration_tests` for `navigation_mission navigation_planning navigation_execution navigation_runtime` — PASS, 4/4 packages and 23/23 CTest tests; `navigation_runtime` passed 17/17.
- `tools/gate.sh all` — NOT_AVAILABLE on this baseline; A2 tooling is outside the W3-B3 write-set.

## 45-function map

| Owner | Functions | Test |
|---|---|---|
| `navigation_runtime_policy` | `PendingGoalHandoffOwner::{enqueueGoal,consumeGoal,goalMatchesStatus,clearGoal,clearIfCurrent}`; `canHotRetargetAtWaypointTransition`; `pendingGoalTerminalStatusMayClear`; `clearHotGoalTransitionAfterCommit`; `hotRetargetUsesCommittedFutureState`; `plannerTerminalStopBrakingDistanceM`; `plannerTerminalStopApproachDue`; `classifyPlannerRenewal`; `watchdogTimeoutMayRetainSafetySuffix`; `watchdogTimeoutMayRetainStoppedRecoveryHold`; `classifyPlannerResult` | `test_planner_fsm` |
| `navigation_execution` | `passThroughTerminalAckMayRetainCommand`; `stoppedHoldCommandRole`; `commandAnchorRecoveryDue`; `terminalStopMayDeferAnchorRecovery`; `terminalStopCompletionObserved`; `terminalStopEndpointContractValid`; `terminalSuccessorHoldMayTransfer`; `retainedValidationTransition`; `terminalMainHasIndeterminatePreStartPressure`; `measuredStateEmergencyMayReplaceCommittedCommand`; `makeMeasuredEmergencyBoundary`; `plannerEmergencyTerminalAltitude`; `terminalHoldIsPending`; `committedTerminalBundleHoldIsPending`; `backupStopNeedsMeasuredRestart`; `stoppedPlanningTimeoutMayFailClosed`; `worldFreshnessSuspendedCommandMayResume`; `supersedingBundleMayRemainAvailable`; `classifyStaleCommandPublication`; `committedSafetySuffixIsUsable`; `retainedSafetyTransitionMayActivateBackup`; `retainedCommandMatchesExecutionIdentity`; `retainedCommandTrackingLimit`; `assessTimeAlignedRetainedTracking`; `assessPhaseExecutionCertificate`; `phaseExecutionBridgeMayPreserveMain`; `projectedRetainedAnchorErrorUpperBound` | `test_planner_fsm` |
| `navigation_mission` | `completedPassThroughRequiresContinuation`; `waypointBehaviorContractValid` | `test_planner_fsm` |
| deferred to W3-B4 | `ordinaryRenewalFailureInjectionMayArm` | `test_planner_fsm` |

The map above counts 15 policy functions + 27 execution functions + 2 mission functions + 1 deferred SITL-harness function = 45 A4 symbols. `goalSnapshot` remains a supporting method but is not one of the 45 A4 rows.

## Header mapping and line evidence

The prompt's 26-header post-A1 manifest is recorded below. The actual B3 baseline still had 27 runtime headers because A1 was absent; `execution_lifecycle_view.hpp` and the additional alias-only `execution_recovery_state.hpp` are both deleted, while direct `navigation_execution` headers become the include authority. Split replacement headers are listed after the manifest. Baseline node lines: 9910. Current checkpoint node lines: 9971 (the +61 lines are includes/qualifiers/using declarations only; no node body rewrite).

| Baseline header | B3 destination/status | Test/consumer |
|---|---|---|
| `baseline_refinement.hpp` | `navigation_runtime_policy/baseline_refinement.hpp` | `test_planner_fsm` |
| `certified_continuation.hpp` | `navigation_execution` | `test_certified_continuation` |
| `commit_trace.hpp` | shell `navigation_runtime_core` | runtime shell |
| `desired_planning_intent.hpp` | `navigation_runtime_policy/desired_planning_intent.hpp` | `test_desired_planning_intent` |
| `execution_recovery_state.hpp` | DELETE alias; direct `navigation_execution` include | lifecycle/runtime tests |
| `execution_trace_snapshot.hpp` | shell `navigation_runtime_core` | trace consumers |
| `experimental_tracking.hpp` | shell (B4 harness boundary remains) | `test_path_relative_tracking` |
| `heading_rebind_worker.hpp` | shell `navigation_runtime_core` | planning worker consumers |
| `kinematic_derivative_estimator.hpp` | shell `navigation_runtime_core` | `test_kinematic_derivative_estimator` |
| `localization_epoch_reset.hpp` | shell `navigation_runtime_core` | `test_localization_epoch_reset` |
| `mapping_fail_stop.hpp` | shell `navigation_runtime_core` | `test_mapping_fail_stop` |
| `mapping_observation_contract.hpp` | shell `navigation_runtime_core` | runtime shell |
| `mission_dynamics.hpp` | `navigation_runtime_policy/mission_dynamics.hpp` | `test_mission_dynamics` |
| `mission_goal.hpp` | shell `navigation_runtime_core` | mission ingress |
| `mission_progress.hpp` | `navigation_mission` | `test_mission_progress` (consumer remains runtime; Q-B3-002) |
| `navigation_runtime_node.hpp` | shell `navigation_runtime_core` | runtime node |
| `path_relative_tracking.hpp` | shell (execution split deferred) | `test_path_relative_tracking` |
| `planner_fsm.hpp` | deferred W3-B4 hook only | `test_planner_fsm` |
| `planning_key.hpp` | `navigation_runtime_policy/planning_key.hpp` | planning worker |
| `planning_supervisor.hpp` | `navigation_runtime_policy/planning_supervisor.hpp` | `test_planning_worker` |
| `planning_worker.hpp` | shell `navigation_runtime_core` | `test_planning_worker` |
| `retained_decision_observation.hpp` | shell `navigation_runtime_core` | `test_retained_decision_observation` |
| `runtime_boundaries.hpp` | `navigation_runtime_policy/runtime_boundaries.hpp` | runtime boundary tests |
| `same_identity_renewal_injection.hpp` | shell until W3-B4 | `test_same_identity_renewal_injection` |
| `trajectory_completion.hpp` | `navigation_mission` | `test_certified_continuation` |
| `world_temporal_assessment.hpp` | `navigation_execution` | `test_world_temporal_assessment` |

Split replacement headers: `navigation_runtime_policy/planning_policy.hpp`, `navigation_execution/execution_decisions.hpp`, and `navigation_mission/waypoint_behavior.hpp`. Deleted aliases `execution_lifecycle_view.hpp` and `execution_recovery_state.hpp` are not part of the 26-row post-A1 manifest.

Line-count evidence: `navigation_runtime_node.cpp` is 9,910 lines at `origin/main` and 9,971 lines at the B3 head; the delta is include/qualification plumbing only. The baseline `include/navigation_runtime/` contains 4,347 header lines across 27 files; the B3 shell root contains 2,354 lines across 15 files, while 997 lines across 7 policy headers were relocated to `include/navigation_runtime_policy/`. The B3 baseline had one extra alias/header relative to the post-A1 26-header manifest; this is tracked in Q-B3-001.

## Findings

| Finding | Status | Commit |
|---|---|---|
| Runtime mission code still linked in `navigation_runtime_core` | FIXED | `12072d9`, `c267a6f` |
| Execution predicates remained in `planner_fsm.hpp` | FIXED | `b83267f`, `c267a6f` |
| Policy target was coupled to `rclcpp` | FIXED by target dependency split; guard PASS | `07081b6` |
| Legacy header using shims | FIXED | `07081b6`, `c267a6f` |
| Policy header paths remained under the shell include root | FIXED | `07081b6` |
| Split package/test metadata incomplete | FIXED | `b83267f` |
| A1/B1 prerequisite objects absent from baseline | PARTIAL / OPEN | `OPEN_QUESTIONS.md` |
| Mission integration test cannot move without reverse shell dependency | PARTIAL / OPEN | `OPEN_QUESTIONS.md` |

## R1 review §3 correction list

The following list is the PR review handoff requested in `REVIEW_R1_20261001.md`; line anchors refer to the corrected post-squash worktree.

| Review item | Correction | `file:line` evidence | Status |
|---|---|---|---|
| #6.1 | Rebase prerequisite checked; #7 is not an ancestor of `origin/main` (`432dc946`), so no unsafe synthetic rebase was performed. | `docs/refactor/W3-B3/OPEN_QUESTIONS.md:8-14` | BLOCKED by remote prerequisite |
| #6.2 | Removed duplicate `completedPassThroughRequiresContinuation`; retained the mission owner and carried the `MissionProgress` comment. | `src/execution/navigation_execution/include/navigation_execution/execution_decisions.hpp:298`; `src/contracts/navigation_mission/include/navigation_mission/waypoint_behavior.hpp:7-16` | FIXED |
| #6.3 | Restored the exact baseline comments, removed the misplaced ordinary-renewal copy, and accepted only the watchdog reflow caused by namespace qualification; no predicate/body/threshold change. | `src/runtime/navigation_runtime/include/navigation_runtime_policy/planning_policy.hpp:193-198`; `src/runtime/navigation_runtime/include/navigation_runtime/planner_fsm.hpp:7-14`; `src/execution/navigation_execution/include/navigation_execution/execution_decisions.hpp:44-70,612-620` | FIXED |
| #6.4 | Made `navigation_mission` test-only and linked it only to `test_certified_continuation`. | `src/execution/navigation_execution/CMakeLists.txt:41-62`; `src/execution/navigation_execution/package.xml:14-15` | FIXED |
| #6.5 | Destination commits are regrouped as mission / execution / policy / shell; each destination has one source commit. | `git log --format='%h %s' origin/main..HEAD` | FIXED |

## Commit log

Source destination commits after the R1 regrouping:

```text
12072d9 refactor(mission): move progress ownership to navigation_mission
b83267f refactor(execution): move runtime decision ownership
07081b6 refactor(runtime): split pure planning policy target
c267a6f refactor(runtime): qualify split shell consumers
```

The review correction report is committed separately from the four destination commits. The branch is `refactor/W3-B3`; no merge was performed. Authoritative Release provenance now passes after submodule initialization; A1/B1 prerequisite comparison and the required post-#7 rebase remain open.
