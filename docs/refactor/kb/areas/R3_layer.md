# R3 navigation_runtime layer (main @ 7e0b850)

## Role
`navigation_runtime` is the product ROS 2 process that owns the whole on-board autonomy loop between LIO and the PX4 External-Mode adapter:
mapping ingestion (via `MappingWorker` + ROG `MappingActor`), world-snapshot publication with execution re-certification, planner scheduling
(`PlanningWorker` around `PlannerFacade`), candidate admission into `navigation_execution::ExecutionAuthority`, retained-command validation
(tracking certificate, BACKUP/EMERGENCY recovery), 50 Hz command sampling/publication (`/navigation/navigation_command`), mission progress
(Core `MissionProgress` when a mission file is configured) and a very large diagnostics/trace surface. It is the only writer of NavigationCommand.
Layer: runtime/execution orchestration (above planning/mapping/world_model/execution libraries, below the PX4 adapter).

## Components
- `NavigationRuntimeNode` (navigation_runtime_node.hpp/.cpp, 9.9k lines) - everything above; one class, ~150 members, 10+ mega-functions (R-09 family).
- `MappingTelemetry` (node.hpp:132-233) - mutex-guarded telemetry snapshot shared by mapping worker and diagnostics.
- `PendingRegisteredScan`, `PendingHeadingRebind` (node.hpp:247-257) - worker hand-off records.
- `PendingGoalHandoffOwner` (planner_fsm.hpp:23-100) - single pending goal while a safety suffix drains (own mutex, redundant with caller locks).
- planner_fsm.hpp free predicates - renewal classification, terminal/stop/backup/emergency admission predicates, tracking certificates, projected anchor bound.
- `DesiredPlanningIntent` (desired_planning_intent.hpp) - desired goal + atomic revision (goal epoch) + one-shot transition {None, NewIntent, HotRetarget}.
- runtime_boundaries.hpp - GoalTransitionKind classifier, fault-injection helpers, `advanceMonotonicId`, `PlannerSolveActivityScope` (watchdog witness), rate/period helpers.
- `PlanningWorker<Planner>` (planning_worker.hpp) - single jthread, one active + one latest pending job, priority/supersession, cancellation.
- `HeadingRebindWorker` (heading_rebind_worker.hpp) - latest-only jthread producing an immutable heading-rebind candidate for the command clock.
- `PlanningSupervisor` (planning_supervisor.hpp) - priority classification and result-currentness check.
- `KinematicDerivativeEstimator` - finite-difference acceleration/jerk for propagated odometry (flagged as estimated).
- `MissionProgress` (+ mission_goal/mission_dynamics) - Core waypoint acceptance / continuation witnesses -> NavigationGoal.
- `ExecutionTraceStore`/`ExecutionTraceSnapshot`, `RetainedDecisionObservation` - diagnostic-only causal records.
- path_relative_tracking.hpp / experimental_tracking.hpp - alternative tracking acceptance models (path projection; SITL experiment).
- certified_continuation.hpp, trajectory_completion.hpp, baseline_refinement.hpp, same_identity_renewal_injection.hpp, world_temporal_assessment.hpp - small typed predicates/witnesses.
- localization_epoch_reset.hpp, mapping_fail_stop.cpp - epoch reset drain helper; abort-on-mapping-failure.

## Processes & threads
One process `navigation_runtime_node`, `MultiThreadedExecutor` with **2 threads** (navigation_runtime_main.cpp:6) plus 3 private threads = 5 threads.
Callback groups: default MutuallyExclusive (registered scan, /lio/health, goal, mode_status, command_admission); `propagated_state_callback_group_` **Reentrant** (odometry);
`planning_callback_group_` MutuallyExclusive (planning wall timer 10 Hz `schedulePlanningCycle`, mission wall timer 20 Hz `tickMissionProgress`);
`command_callback_group_` **Reentrant** (command wall timer 50 Hz `publishCommand`). Private threads: MappingWorker (process_mapping lambda),
PlanningWorker (runCycle jobs), HeadingRebindWorker. All timers are wall timers (steady), while every freshness/lease decision uses ROS time (sim time in SITL).

### Lock inventory (what each mutex guards; documented order: ingress -> localization -> input -> command-transition)
| # | Mutex | Guards | Main holders / heavy work under it |
|---|---|---|---|
| L1 | `localization_epoch_ingress_mutex_` | serialization of epoch transitions from odom/scan/health ingress | onRegisteredScan, onEstimatorHealth, onPropagatedOdometry (whole body); held across `resetForLocalizationEpochLocked` incl. mapping drain (R3-07) |
| L2 | `localization_transition_mutex_` | active epoch transitions, lifecycle owner transaction (first of the triple) | odometry callback body incl. state-store publish (R3-15); every lifecycle transaction below; released during mapping drain |
| L3 | `input_mutex_` | desired_intent_ goal/transition, pending/deferred goal, foreign-mission latch, mission_start_*, mission_progress_ / mode boundary / issued_mission_commands_, plan_from_rest_first_failure_steady_ns_ | goal/status/mission callbacks; runCycle transactions; publishCommand transactions |
| L4 | `command_execution_lease_failure_latch_.transitionMutex()` | execution-authority mutations, lease failure latch, trajectory_completion_witness_, terminal markers | triple held by: mapping publishAndFinalize (1237-1291), admitImmediateCandidate (2885-2929), currentPlanningKey, runCycle (~15 sites), validateRetainedCommand (2x), publishCommand (activation, capture, stopped-hold, completion, **final exposure incl. ROS publish** R3-14), watchdog, planning-worker fatal handler |
| L5 | ExecutionAuthority internal locks (navigation_execution) | timeline/pending/lifecycle snapshot | taken inside L4 (publishIfCurrent, tryCommitIfCurrent, activatePendingIfDueAndFinalize) |
| L6 | `propagated_derivative_mutex_` | KinematicDerivativeEstimator | odom callback (inside L1+L2), reset |
| L7 | `planner_solve_activity_mutex_` | active_planner_solve_generation_, started_ns, witness | PlannerSolveActivityScope (worker), watchdog in publishCommand: **L7 -> L2 -> L3 -> L4** order and cancelActive under L7 |
| L8 | `planner_timeline_activation_mutex_` | queued_planner_timeline_activations_ deque | commit (worker/command), sampler tick |
| L9 | `heading_rebind_mutex_` | pending_heading_rebind_ | heading worker (write), command timer (swap) |
| L10 | PlanningWorker::mutex_ (+ recursive backend_access_mutex_) | active/pending job, snapshot | submit/cancel call `planner_->cancelActiveSolve()` under it -> Planner::solve_commit_mutex_ (R3-16) |
| L11 | HeadingRebindWorker::mutex_ | pending job | submit/run |
| L12 | PendingGoalHandoffOwner::goal_mutex_ | pending goal ptr | always nested inside L3/L4 (redundant) |
| L13 | MappingTelemetry::mutex_ | telemetry snapshot | mapping worker, diagnostics |
| L14 | WorldSnapshotStore / ExecutionStateStore / ExecutionTraceStore internal | immutable pointers | loads everywhere (lock-free or short) |
| (ext) | Planner::solve_commit_mutex_ | staged/retained candidates, cancel flag | worker commit, heading export construction, cancelActiveSolve, discard* from command thread |

Heavy work kept OUTSIDE locks (good): full swept `validateWorld` in mapping callback and retained validation, planner solve, path-relative/phase certificates (prepared before the final transaction).
Heavy/slow work INSIDE locks (bad): ROS command publish + deque copy (R3-14); 2 deep NavigationGoal copies per tick (R3-13); mapping drain under L1 (R3-07); backend cancel under L2-L4 (R3-16); odom derivative + store publish under L1+L2.

### State written by more than one thread
- `trajectory_completion_witness_`, `trajectory_reaches_goal_`, `terminal_bundle_generation_`: command timer, planning worker, mapping worker, goal/status callbacks (atomics or L4) - consistent under L4 but the atomics are also read lock-free for scheduling decisions (terminalHoldIsPending) -> decisions from a torn combination are possible between sites.
- `skip_replan_once_`: mapping worker, worker, callbacks (atomic).
- `plan_from_rest_first_failure_steady_ns_`: worker (under L2-L4) and goal/status callbacks (under L3) - OK.
- `last_planning_timer_expected_steady_ns_`, `last_planning_callback_start_steady_ns_`: timer writes, worker reads, plain int64 (R3-06).
- `last_execution_boundary_rejection_`: written by worker and command thread (commitPlannerCandidate runs on both) - evidence can be attributed to the wrong path.
- PlannerFacade: worker, command timer, planning timer, heading worker (R3-17, R3-01).

## Flows
1. **Mapping**: `/lio/mapping_observation` (RegisteredScan, BestEffort KeepLast1) -> onRegisteredScan (contract checks, epoch reset if newer, sequence monotonic) -> MappingWorker inbox -> validate_mapping (ROS-time freshness vs data_freshness_window 0.5 s) -> process_mapping: decodeCloud -> MappingActor::process -> snapshot -> revalidate pending/active bundle (disjoint-region fast path / full validateWorld / terminal & expired-recovery endpoint exceptions) -> under L2-L4 `publishAndFinalizeDecision` (world publish + execution certificate transition + optional invalidation) -> witnesses on `/navigation/diagnostics` -> optional resume of a freshness-suspended command.
2. **State**: `/lio/odometry_propagated` (PropagatedOdometry; twist in body frame, rotated to world) -> onPropagatedOdometry -> finite-difference A/J -> ExecutionStateStore lease {source_stamp ROS ns, receive_stamp steady ns}.
3. **Goal**: `/navigation/goal` (external, only without mission file) or MissionProgress decision -> applyValidatedGoalLocked -> desired intent revision++ -> ExecutionAuthority::beginGoal (hot retarget keeps bundle) / pending goal while suffix drains / foreign-mission latch.
4. **Planning**: 10 Hz timer -> currentPlanningKey (L2-L4 snapshot) -> optional heading rebind job -> PlanningWorker.submit -> runCycle: diagnostics publish, freshness gates, completion/recovery state machine, renewal classification, anchor reservation, PlannerFacade::plan -> classifyPlannerResult -> commitPlannerCandidate (export, activation window, anchor match, endpoint/route boundary, MAIN reserve, state anchor, stagePending or admitImmediateCandidate) or validateRetainedCommand (tracking certificates -> emergency brake / BACKUP activation / fail closed) -> huge decision trace + JSON path snapshot.
5. **Command**: 50 Hz timer -> world freshness gate -> consumeHeadingRebind -> pending activation -> watchdog -> lease freshness under L2-L4 -> CommandSampler.sample -> stopped-hold/terminal/continuation handling -> NavigationCommand {status READY/COMPLETED/BRAKING/REJECTED, role MAIN/BACKUP/EMERGENCY, PVAJ, yaw, valid_until = now+kCommandStreamTimeoutS} published inside publishIfCurrent; `/navigation/execution_diagnostics` after.
6. **Mission**: `/navigation/mode_status` (transient_local) + `/navigation/command_admission` -> MissionProgress (20 Hz tick) -> `/navigation/mission_progress`, `/navigation/mission_complete`.

## State machines
Owned elsewhere but driven from this node (navigation_execution):
- `ExecutionRecoveryState` {kInitialHold, kTrackMain, kTrackBackup, kEmergencyBrake, kStoppedRecovery, kPx4Hold} with events {kMainCommitted, kBackupActivated, kEmergencyCommitted, kCertifiedStopObserved, kTerminalStopCompleted, kEmergencyCertificationFailed(->PX4Hold)}. Runtime trigger sites: kBackupActivated node.cpp:8488-8489 (retained validation) and :9211 (observeSampledSafetyRole on sampled BACKUP); kEmergencyCommitted via commitPlannerCandidate immediate admission (:3339) and sampled role (:9190-9212); kCertifiedStopObserved :4521-4522 (speed <= 0.15 m/s after completed suffix); kTerminalStopCompleted :4963-4964; kEmergencyCertificationFailed :8416-8418; fail-closed (-> PX4 Hold) via failClosedLocked at ~25 sites (e.g. :1706, :2227, :2369, :4694, :6128, :8412-8493, :8962, :9164, :9390, :9866).
- `ExecutionPhase` {InitialHold, TrackingMain, TrackingBackup, StoppedHold, Px4Hold}, `ExecutionExposure` {Unavailable, Available, Suspended, Failed}, `ExecutionSafetyOwnership` {Nominal, SafetySuffix}, restart request {None, FromRest} - all inside ExecutionAuthority snapshot; suspend :3471 / :5352, resume :1392, stoppedHold :9143.
Runtime-owned:
- `PlanningIntentTransition` {kNone, kNewIntent, kHotRetarget} (desired_planning_intent.hpp:17-21): install :2421-2423, consume :4489,:5005,:6475-6479,:8688-8689, rearm after localization reset :1951, clear :2636.
- `PlanningStartMode` {kStoppedMeasuredState, kCommittedFutureState} chosen :3678-3682 / :5535-5537.
- `PlannerResultDisposition` {CommandReady, RestartFromRest, RetryFromRest, ValidateRetainedCommand, RetainCommittedCommand, FailClosed} (planner_fsm.hpp:703-740) consumed :6102-6482.
- `PlannerRenewalReason` {RetainCertifiedMain, ForcedTransition, NoCommand, SafetyRecovery, InvalidHorizon, RenewalDue, QualityRefinement} (planner_fsm.hpp:229-441; baseline_refinement.hpp:113-121).
- `RetainedDecisionDisposition` (Superseded, EntryRejected, Discarded, FailClosed, EmergencyDelivered, RecoveryBridgePreserved, MainBridgePreserved, CertifiedCommandPreserved) :8393-8495.
- `StaleCommandPublicationDisposition` {DropStale, RetainSuperseding, FailClosed} :9856-9868.
- Latches: foreign_mission_hold_after_stop_ (set :2200, consumed :4524-4536/:4713-4722, cleared :2642), deferred_terminal_status_, pending_goal_owner_, command_execution_lease_failure_latch_ (tryLatch :8955, :9763; reset per new goal :2454, per epoch :1960), skip_replan_once_, trajectory_reaches_goal_/terminal_bundle_generation_, completion witness.
- `PathRelativeTrackingStatus` (10 states), `WorldTemporalReason` (6), `PlanningPriority` (6, 2 unreachable R3-23), `GoalTransitionKind` (6).
- MissionProgress: inactive/active x gate(waypoint,request) x {crossing, continuation, stop confirmation, hold} -> Goal/Complete decisions (mission_progress.cpp:67-265).

## Behavior per branch
- **onRegisteredScan** (:1976-2073): null/sensor-origin/visibility/frame/stamp/pose invalid -> rejected_before_inbox counter, return; older epoch -> reject; newer epoch -> full reset (+drain under ingress lock); non-increasing sequence -> reject; else submitFromWaiting (drops older waiting). Never touches command state directly.
- **process_mapping lambda** (:906-1460): decode failure -> throw -> mappingFailStop -> abort(); pending/active revalidation: disjoint-region proof (1) / full validateWorld (2) / terminal endpoint (3) / expired recovery endpoint (4) / reject (5); publication: epoch mismatch -> Superseded (return, command untouched); commit failure -> invalidate + throw (abort); committed + retained -> optional resume of suspended exposure; committed + rejected -> active invalidated inside finalize (PX4 Hold path).
- **onPropagatedOdometry** (:2087-2161): bad frame -> WARN_THROTTLE drop; newer epoch -> reset; stale epoch/sequence/stamp/nonfinite/child frame -> invalid counter; else A/J estimate + lease publish.
- **applyValidatedGoalLocked** (:2256-2476): foreign latch -> reject; stale same-mission -> reject; moving safety suffix -> foreign mission => latch hold-after-stop, same mission => enqueue pending; cross mission (no suffix) -> transitionForeignMissionLocked(false) fail closed; revision exhausted -> fail closed; else cancel worker, beginGoal (hot retarget keeps bundle), install intent, mission-start anchor, reset lease latch.
- **onModeStatus** (:2478-2652): ACTIVE/BRAKING -> only activation bookkeeping; terminal status for pending -> clear pending unless suffix moving; not matching active -> return; suffix active -> defer; PASS_THROUGH certified continuation -> retain; else cancel + clear goal (admission epoch advanced) -> no command.
- **commitPlannerCandidate** (:2933-3429): 16 typed rejects (each discards planner staged candidate, R3-01), successor -> stagePending with reserved anchor; immediate -> admitImmediateCandidate (re-check under L2-L4 + clock predicate); ACK queue failure -> clearCommandForCurrentIdentity + fail closed.
- **currentPlanningKey** (:3577-3690): no goal/state/world, pending successor, PX4Hold/failed, identity mismatch, terminal hold pending -> nullopt (no solve).
- **runCycle** (:4010-7548): epoch not ready -> return; world stale -> cancel + suspend exposure; state stale -> counters, return (command untouched); BACKUP/EMERGENCY moving -> return; certified stop -> promote pending / deferred terminal clears goal / restart-from-rest; StoppedRecovery+moving -> fail closed (unless emergency settling or terminal hold); invalid route -> clear command; terminal monitor -> validateRetainedCommand only; stationary gate for PlanFromRest; renewal deferred -> return; solve -> disposition switch (see planner_fsm), CommandReady -> commit (+skip_replan_once_), Retain/Validate -> validateRetainedCommand, RetryFromRest -> timeout decides fail closed, FailClosed -> conditional fail closed; always emits decision trace.
- **validateRetainedCommand** (:7564-8587): stale expected snapshot -> Superseded; monitor entry mismatch -> EntryRejected; computes raw vs time-aligned anchor (R3-11), world validation, phase/path/experimental bridges, projected bound (R3-10); emergency authorized -> commitEmergencyBrake (R3-09) + immediate commit; final transaction: discard / superseded / lease failed -> fail closed / emergency failure -> PX4Hold / recovery bridge / MAIN bridge / certified suffix (+BACKUP activation) / fail closed.
- **publishCommand** (:8590-9908): epoch not ready -> return (no message); world stale -> cancel + suspend + return (no message); heading rebind admission; pending activation; watchdog (retain suffix / stopped hold / certified MAIN, else fail closed); no command & not failed -> return; lease superseded while waiting -> return (R3-15); lease invalid -> latch + fail closed + publish REJECTED/EMERGENCY with zero PVAJ; sampler awaiting -> return; sampler no bundle -> clear; stopped hold checks; exposure transaction (freshness/world/lease/hold-anchor) -> publish READY/COMPLETED/BRAKING; exposure failure -> suspend / lease latch / deadline drop / stale classification.
- **tickMissionProgress / onCommandAdmission**: stale/unairborne boundary -> no-op; activation -> deactivate+activate -> Goal; measured sample -> crossing/stop logic; admission receipt -> continuation witness -> Goal/Complete; invalid mission goal -> fail closed.

## Information needs
| Input | Source | Frame | Clock | Freshness bound | Authority |
|---|---|---|---|---|---|
| RegisteredScan (cloud + corrected pose + sensor origin + visibility endpoints) | FAST-LIO `/lio/mapping_observation` | lio_odom (planning_frame_), body base_link | ROS header stamp (ns) | data_freshness_window_s 0.5 s (validate_mapping, ROS-now vs stamp); strictly increasing stamp/sequence | mapping only; defines world snapshot identity/epoch |
| EstimatorHealth | `/lio/health` | - | - | none (only epoch) | epoch reset announcement |
| PropagatedOdometry (P, q, body twist, epoch, sequence) | `/lio/odometry_propagated` | lio_odom / base_link, twist in body rotated to world | source = ROS stamp, receive = steady | 0.5 s: ROS-only at plan/commit (R3-03), ROS+steady at retained/command/mission | sole execution-state lease (anchor, stop, tracking) |
| NavigationGoal | `/navigation/goal` (only without mission file) or MissionProgress | lio_odom ENU | ROS stamp (unused for freshness) | none; ordered by request_id/waypoint/route_revision | desired intent |
| NavigationModeStatus | adapter `/navigation/mode_status` (transient_local) | frame check only | ROS stamp; receive steady | 200 ms for activation boundary (literal) | mission activation/terminal status |
| NavigationCommandAdmission | adapter `/navigation/command_admission` | lio_odom | ROS stamp | must match issued command within its lease (<=100 ms) | continuation witness for MissionProgress |
| World snapshot | WorldSnapshotStore (mapping worker) | lio_odom grid | observation_stamp ROS ns | 0.5 s source age (assessWorldTemporal, no receive leg) | planning/commit/exposure gate |
| Committed bundle / pending | ExecutionAuthority | lio_odom | declared_start/end ns + start_wall_time_s double (N3) | valid_from..valid_until (<= 0.5 s past activation) | only command authority |
| Planner config / mission file | planner.yaml, mission YAML | - | - | startup | immutable dynamics (dynamics_hash) |
| Output NavigationCommand | `/navigation/navigation_command` reliable KeepLast1 | lio_odom ENU/FLU | header = sample time (double path R3-18), valid_until = now + 100 ms | 50 Hz | sole PX4-bound command |

## Bottlenecks
1. Global triple lock (L2+L3+L4) is taken by every callback family: odometry (~IMU/LIO rate), mapping publication, ~20 planner transactions per solve, 3-6 times per command tick; it also covers DDS publish (R3-14), deep goal copies (R3-13) and backend cancellation (R3-16). Command latency = max hold time of any of them.
2. Two executor threads for 5 callback groups; ingress mutex held across the epoch-reset drain can park both threads (R3-07).
3. Command tick drop on lease supersession (R3-15) couples odometry rate to command continuity.
4. Planning worker serial path: after each solve it builds ~400 KeyValue strings + a JSON path snapshot + 2 large INFO logs (runCycle :6540-6633, :6635-7470) before the next job can start; mapping callback builds ~100 KeyValues per update.
5. validateWorld (full swept certificate) runs in mapping callback for active and pending on intersecting updates (outside locks, but on the mapping thread -> map latency) and again in retained validation.
6. Heading-rebind export under Planner::solve_commit_mutex_ blocks cancelActive callers holding lifecycle locks.
7. decodeRouteSnapshot is re-run in ~10 places (command tick, mapping callback, runCycle, scheduling) instead of once per goal.

## Notes for target architecture
Keep (good):
- Immutable CandidateBundle + ExecutionAuthority as single command authority with typed commit tokens; world publication and certificate finalization linearized in one gate (publishAndFinalizeDecision).
- Heavy validation (validateWorld, path/phase certificates, planner solve) prepared outside locks, with a cheap in-lock identity recheck.
- Explicit dual-clock lease (ROS source + steady receive) at command exposure; fail-closed defaults; monotonic ids that never wrap; epoch reset drain protocol; mapping fail-stop.
- Pure predicate headers (planner_fsm, certified_continuation, baseline_refinement) with large unit-test suites; MissionProgress as a separate single-writer Core.
Over-engineered / to simplify:
- One 9.9k-line node with ~150 members and six mega-functions (R-09); ~20 re-implementations of the same "identity still current" check (matchesActive && epoch) inline; duplicated snapshot() calls (episode/timeline taken twice under the same lock).
- Diagnostics/fault-injection surface compiled into product hot paths (7 injection parameters, exact-optimization injection, same-identity renewal injection, decision trace of ~400 fields per solve, witnesses per transaction). Move to an observer that consumes immutable records off the control threads.
- Three alternative tracking acceptance models (time-aligned tube, phase certificate, path-relative, experimental) plus raw anchor; they disagree on time basis (R3-02/R3-10/R3-11). Target: one TrackingAssessment computed once from (lease source stamp, now, bundle) and consumed by all predicates.
- Planner facade reachable from four threads (R3-17); make PlanningWorker the only door, heading rebind an explicit slot with owner tokens.
- Replace the 2-thread executor + reentrant groups with: dedicated real-time command thread (no shared lifecycle lock; reads immutable authority snapshot + lease), ingress thread, and the worker threads; publish outside locks.
- Unify time: int64 ns everywhere (bundle, command header, conversions); one seconds->ns helper (R3-08, N3).
