# Identity Comparison Audit V2

## Scope and provenance

This is a read-only audit of product source at `ba9c15a1214ca8ba54ec8f301f1e4056a3322848` (tree `6fbd807010aa7a88da247e77cab7a71984b7f33d`). RuntimeNode was audited in `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp` and its header. Counts below exclude tests, documentation, and artifacts unless stated.

## Helper inventory before migration

RuntimeNode declares both helpers at `src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp:385-395` and defines them at `navigation_runtime_node.cpp:1735-1773`.

| Helper | Definition | Call sites | Semantics and disposition |
|---|---:|---|---|
| `desiredGoalIdentityMatchesLocked` | 1735 | 2741, 4498, 4642, 4722, 4727, 4992, 5641, 5655, 5717, 5846, 5887, 5922, 8628, 8677 | Mixes desired `(mission, waypoint, request, desired epoch)` with localization. When given a generation it additionally reads active-bundle identity, so this is sometimes a desired predicate plus an ExecutionAuthority predicate. Replace with a bounded desired-intent `matches` predicate and separately ask ExecutionAuthority about the active token where required. |
| `executingCommandIdentityMatchesLocked` | 1755 | 1255, 2434, 2754, 4134, 4170, 4335, 4604, 5673, 7877, 7904, 7919, 8491, 8513, 8564 | Reconstructs active mission/waypoint/request and bundle goal/localization/generation identity from an ExecutionAuthority snapshot. Move this query into ExecutionAuthority or replace with its exact active token API. |

The desired calls at 2741 and 4992 are admission/desired-current checks. Calls at 4498, 4642, 4722/4727 and 8628/8677 participate in terminal completion or handover and must keep desired state separate from the execution witness. Calls at 5846/5887 mix desired currentness with the exact committed bundle and should be split. Calls at 5641 and 5717 guard failure effects and are the conditional fail-close cases below.

Execution helper calls are active execution evidence: world-resume identity at 1255, terminal status at 2434, terminal candidate ownership at 2754, measured-stop/restart/stop identity at 4134/4170/4335, completion witness at 4604, restart completion at 5673, retained-command observations at 7877/7904/7919, and publication hold/activation/safety-role checks at 8491/8513/8564.

## Conditional fail-close findings

### Planner result disposition: lines 5641-5643

The stale-result guard checks only desired goal and localization identity before calling `failClosedLocked()`. For a normal retained hot retarget, `ExecutionAuthority::beginGoal(..., retain_active=true)` preserves an available predecessor (`src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp:123-141`), and `classifyPlannerResult` maps failed/emergency outcomes with an available command to `RetainCommittedCommand` (`src/runtime/navigation_runtime/include/navigation_runtime/planner_fsm.hpp:707-723`). Those common outcomes do not reach this fail-close.

The conditional gap remains reachable for statuses that fall through the classifier while the predecessor is still available. In particular, `kOptimizationFailed` falls through to `FailClosed` at `planner_fsm.hpp:694-731`; the backend maps its optimization failure to that status at `src/planning/navigation_planning_backend/src/planner_facade.cpp:34`. Success/finished without an observed staged commit also falls through, with explicit classifier expectations in `src/runtime/navigation_runtime/test/test_planner_fsm.cpp:1085-1088`. If one occurs for current desired successor N+1 while active predecessor N remains available, the desired-only check can fail-close N.

Required evidence before this callback mutates active execution: the exact active token captured for the planning request still matches the current ExecutionAuthority active record under the transition locks. The desired intent revision remains a separate admission/applicability check. The captured token should bind active bundle generation or pointer, active goal identity and epoch, localization epoch, and the owner snapshot/version needed to reject a changed lineage.

### Stopped-recovery retry timeout: lines 5717-5743

This is reachable during pending-goal promotion after a certified stop. RuntimeNode promotes the pending successor and forces the cycle-local disposition to PlanFromRest at `navigation_runtime_node.cpp:4191-4205` (the same flow appears at 4373-4385). The active predecessor may remain stored as a stopped hold: `ExecutionAuthority::stoppedHold` leaves exposure available and marks the lifecycle phase `kStoppedHold` (`committed_bundle_store.hpp:257-267`), while pending-goal promotion invokes `beginGoal` with the retain decision at 2147-2268. The local PlanFromRest flag plus `kStoppedRecovery` makes `stopped_recovery_retry` true at 5544-5547; the classifier receives command-available false and routes a failed solve to `RetryFromRest` (`planner_fsm.hpp:725-730`).

When the stopped-planning timeout expires, the current code checks only desired identity at 5717-5719 and then calls `failClosedLocked()` at 5743. Therefore a current N+1 solve can fail-close a retained N record in the stopped-recovery case. This is not the ordinary moving hot-retarget path; whether the retained stopped command remains exposed depends on the exact lifecycle phase and publication state. The timeout should prove the retained stopped-hold token is still the active record before revoking it, and document whether that revocation is the intended timeout policy.

### Publisher stale execution-state lease: lines 8308-8345

The first fail-close is unconditional under lifecycle locks at 8315-8319 when the captured execution-state lease is invalid. It is a global publication safety boundary, not a desired-identity-only guard. The callback captured an ExecutionAuthority snapshot at 8287-8294, but does not compare its active token before this transition. Because the freshness result was computed from a lease loaded before the lifecycle lock (8257-8264), a newer valid lease could race that decision; reloading/comparing the lease would distinguish a still-current stale lease from superseded evidence.

After releasing locks to cancel the planner worker, the callback reacquires them and checks desired goal fields/epoch only at 8337-8343 before repeating `failClosedLocked()` and clearing pending goal at 8345-8346. This is a delayed repeat/cleanup, not the first authority mutation. A robust delayed check binds both the captured failed lease sequence/pointer and captured ExecutionAuthority active token to current snapshots; pending-goal cleanup should also be conditional on the exact pending pointer.

## Manual identity comparisons and classification

The RuntimeNode scan found 102 syntactic equality checks over the requested field families (`mission_id`, `waypoint_index`, `request_id`, `goal_epoch`, `bundle_generation`, `route_revision`). This is a search upper bound, not 102 redundant identity sites: it includes protocol validation, owner invariants, route/mission progression and diagnostic comparison.

| Evidence site | Classification | Migration note |
|---|---|---|
| 2171-2178, 2422-2426, 2437-2445 | `EXTERNAL_PROTOCOL` / `MISSION_PROGRESS` | Incoming goal order, accepted waypoint status and route continuation are independent mission/protocol contracts; retain their semantics. |
| 2741-2750, 5846-5889 | `DESIRED_INTENT` + `ADMISSION_FENCE` + `ACTIVE_EXECUTION` | Desired epoch controls stale-result applicability. Bundle generation/request/localization is an ExecutionAuthority token. Do not collapse the roles into one `goal_epoch` predicate. |
| 3364-3369, 4815-4826, 5040-5043 | `LEGITIMATE_COMPOSITE` | Terminal monitoring/renewal requires both current desired state and an exact execution witness plus lifecycle conditions. Keep the dual requirement explicit. |
| 4498-4503, 4601-4618, 4717-4728 | `LEGITIMATE_COMPOSITE` | Completion requires current desired mission target and a current execution completion witness; predecessor completion must not complete successor intent. |
| 5221-5230, 7794-7816 | `DESIRED_INTENT` + `ACTIVE_EXECUTION` | Desired-vs-executing equality is a same-identity optimization/terminal-monitor condition only. Keep distinct predicates so N+1 may coexist with N. |
| 8617-8639, 8664-8689 | `ACTIVE_EXECUTION` plus `DESIRED_INTENT` | Publisher completion logic manually reconstructs current bundle/execution identity and separately checks current desired identity. Move the active tuple check into ExecutionAuthority; retain the explicit BOTH_REQUIRED completion condition. |
| 8337-8346 | `DESIRED_INTENT` guarding delayed cleanup | Does not prove current active execution or current pending-goal pointer. The first global stale-state fail-close at 8318 is a separate safety transition. |

There is no evidence that PX4 or WorldModel protocol comparisons should be migrated into mission/ExecutionAuthority helpers. Those owners must retain their own identity and freshness checks.

## Migration target

Use `DesiredPlanningIntent.matches(goal, revision, ...)` for desired-current/admission applicability. Use an exact `ExecutionAuthority` active token or owner method for active-command identity. Where mission completion or terminal transfer needs both, spell the two witnesses separately and preserve the existing conjunction. A desired identity match alone must not authorize fail-closing a retained active command.
