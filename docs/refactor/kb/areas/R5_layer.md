# R5 layer — PX4 boundary (External Mode adapter + odometry bridges), execution authority, planning/command contracts, mission contract, common helpers

Reviewed at `main @ 7e0b850`. Finding IDs refer to `findings.csv` (R5-01..R5-39).

## Role

- **Lowest ROS layer before PX4.** Converts the Core/runtime's certified LIO-ENU PVA command (`NavigationCommand`) into PX4 NED `TrajectorySetpoint` through a px4_ros2 External Mode (`px4_navigation_external_mode`), and owns PX4 Hold handover.
- **Estimator ↔ PX4 bridge.** `px4_external_odometry_bridge` publishes LIO propagated odometry as PX4 `vehicle_visual_odometry` (ENU/FLU → NED/FRD, covariance → variances, µs stamps, reset_counter). `px4_odometry_bridge` converts PX4 `vehicle_odometry` back into a reset-compensated ROS ENU/FLU odometry (`/px4/estimator_odometry`) for LIO initial prior and judge evidence.
- **Execution authority** (`navigation_execution`): the only owner of the active/pending `CandidateBundle` that may reach the command sampler; linearizes commit, staging, activation, world recertification and exposure under one mutex.
- **Contracts** (`navigation_planning` headers, `navigation_contracts` msgs + header predicates, `navigation_mission`): shared data types plus a large amount of validation logic in headers (bundle validity, role schedules, command contract/temporal lease, freshness, tracking experiment policy loader).
- **Common** (`navigation_common`): ENU/NED + FLU/FRD matrices, checked time conversions, SPSC queue.

## Components

| Class / file | Responsibility |
|---|---|
| `NavigationMode` (navigation_mode.hpp / navigation_mode_node.cpp) | px4_ros2 `ModeBase`: command admission, state/health/PX4 ingestion, LIO→PX4 alignment, `updateSetpoint` (PVA, velocity-only, holds), status, traces |
| `NavigationModeExecutor` (navigation_mode_node.cpp:2584-2701) | px4_ros2 `ModeExecutorBase`: schedules owned mode, retries PX4 Hold (AUTO_LOITER) every 250 ms until confirmed |
| `tracking_adapter::adapt` (px4_tracking_adapter.hpp) | Velocity-only experiment: relative-heading LIO-ENU→PX4-NED rotation, yaw/yaw-rate mapping, timing witness validation |
| `velocity_only::limit` (velocity_only_continuity.hpp) | Dykstra projection onto accel/jerk/velocity/viability balls |
| `evaluateTrackingEnvelope` (tracking_envelope.hpp) | Outer longitudinal/reverse/lateral guard vs 0.75 m anchor limit (R5-16) |
| `localNedTranslationFromStationaryPair` / `lioPositionToLocalNed` (local_frame_alignment.hpp) | Translation-only LIO↔PX4 alignment (R5-17) |
| `CommandAdmissionAssessment`, `assessCommandSessionIdentity` (command_admission_assessment.hpp) | Admission stage/disposition + session identity rules |
| `certified_command_handoff.hpp`, `planner_recovery.hpp`, `command_acceptance_gate.hpp` | Small predicates for retain/commit/invalidate, bounded terminal recovery window, sample ordering |
| `MissionController` (mission_controller.*), `mission_command_identity.hpp` | **Deprecated**; built into `_contract` lib, linked only by `test_mission` (R5-24) |
| `PairedNodeLifetime` | Join state-input thread before releasing nodes |
| `Px4ExternalOdometryBridgeNode` (px4_external_odometry_bridge_node.cpp) | LIO propagated odom + LIO diagnostics → `/fmu/in/vehicle_visual_odometry` with gate |
| `convert_ros_lio_odometry` (external_odometry_conversion.cpp) | ENU/FLU → NED/FRD pose, NED world velocity, FRD body rates, covariance blocks → diagonal variances (symmetry/PSD checked) |
| `observe_geometric_jump_continuity`, `GeometricJumpLatch`, `evaluate_external_odometry_gate` | Jump detection, latch, AND gate (R5-08, R-02) |
| `TimestampConverter` (timestamp_conversion.cpp) | ns→µs, age/regression checks, mapping-mode declaration (no local offset) |
| `Px4OdometryBridgeNode` (px4_odometry_bridge_node.cpp) | PX4 `vehicle_odometry` + reset metadata → `/px4/estimator_odometry`, `/px4/sample_odometry_at_time` service |
| `FrameConverter` (frame_converter.cpp) | NED/FRD → ENU/FLU incl. variances (R5-03, R5-04) |
| `ResetCompensator` (reset_compensator.cpp) | Continuity transform across PX4 resets (R5-13, R5-14) |
| `OdometryRingBuffer` | 2 s / 512-sample history, slerp interpolation, stable-sample gate |
| `TimestampValidator` | PX4 stamp vs ROS now, restart detection (R5-06) |
| `ReferencePointConverter` | Unused lever-arm converter (R5-15) |
| `ExecutionAuthority` (execution_authority.hpp) | Active/pending bundle store; commit/stage/activate/recertify/publishIfCurrent (R5-26..28) |
| `CommandSampler` | Read-only sampling of active bundle, STOPPED_HOLD at declared end |
| `ExecutionStateStore`, `ExecutionStateFailureLatch`, `classifyTimestampFreshness` | Latest epoch-tagged state lease, failure latch, freshness classification |
| `transitionExecutionRecovery` (execution_recovery_state.hpp) | Recovery FSM (R5-29) |
| `CandidateBundle` (candidate_bundle.hpp) | Immutable bundle + validity, role schedule, sampling, handoff certificate (R5-31..34) |
| `PlanningRequest/Key/History`, `PlanningBudget`, `PlanningTimingContract`, `DynamicLimits` | Planning input contracts (R5-35) |
| `navigation_contracts` headers | `assessCommandContract`, `assessCommandTemporalLease`, `evaluateExecutionStateFreshness`, `loadTrackingExperimentPolicy` (R5-36..38) |
| `navigation_mission::Mission`, `RouteProgress`, `ImmutableRouteSnapshot` | Mission YAML contract, polyline arc-length progress, crossing detection |
| `navigation_common` time.hpp / frame_conventions.hpp / bounded_spsc_queue.hpp | Checked conversions (R5-01), fixed basis matrices (R5-02), SPSC trace queue |

## Processes & threads

- **px4_navigation_external_mode process** (main.cpp): (1) main thread `rclcpp::spin(mode node)` — px4_ros2 setpoint timer → `updateSetpoint` at 50 Hz (`setSetpointUpdateRate(50)`), `onNavigationCommand` (reliable, depth 1), `onMissionProgress`, `updateBoundary` wall timer 50 ms, executor `checkHoldHandover` wall timer 50 ms, vehicle_status shared subscription; (2) state-input thread spinning a second node — `onOdometry` (`/lio/odometry_propagated`), `onEstimatorHealth` (`/lio/health`), `onPx4LocalPosition`; (3) trace worker thread draining the 256-slot SPSC queue every 5 ms. All shared state under one `trajectory_mutex_`; px4_ros2 `isArmed()` read cross-thread (R5-23). Lock held across DDS publish in terminal states (R5-19).
- **px4_external_odometry_bridge process**: single-threaded `rclcpp::spin`; LIO odometry (reliable depth 20) and diagnostics callbacks; publishes best-effort to PX4.
- **px4_odometry_bridge process**: single-threaded; VehicleOdometry/LocalPosition/Attitude/TimesyncStatus best-effort subs, sample service, 500 ms wall diagnostics timer; per-sample diagnostics publish (R5-05).
- **ExecutionAuthority** lives in the runtime process (R4); one `std::mutex`; evaluators run outside the lock except destructor teardown (R5-27) and `publishIfCurrent` callback (intentionally inside).

## Flows

1. **LIO → PX4 EKF2 (external vision)**: `/lio/odometry_propagated` (`PropagatedOdometry`, lio_odom ENU / base_link FLU, ns ROS stamp) → identity high-water (epoch, sequence) → `convert_ros_lio_odometry` → float/variance checks → `TimestampConverter` (µs, age vs ROS now ≤ 0.5 s) → geometric jump continuity + latch → gate (needs `/lio/diagnostics` TRACKING + covariance + generation, fresh ≤ 2 s) → `VehicleOdometry{timestamp=now µs, timestamp_sample=LIO stamp µs, POSE_FRAME_NED, VELOCITY_FRAME_NED, reset_counter=diag generation %256}` on `/fmu/in/vehicle_visual_odometry[_vN]`.
2. **PX4 → ROS evidence**: `/fmu/out/vehicle_odometry` (timestamp_sample µs) → `TimestampValidator` → `FrameConverter` → reset metadata association (`vehicle_local_position`, `vehicle_attitude`, ≤ 0.1 s, ≤ 5 ms between) → `ResetCompensator` → startup origin rebase → ring buffer (3 stable samples) → `/px4/estimator_odometry` (px4_odom ENU, base_link FLU twist) + diagnostics; `/px4/sample_odometry_at_time` answers from buffer.
3. **Command → PX4 setpoint**: `/navigation/navigation_command` → contract → temporal lease → terminal ownership → session identity (health epoch, activation id, mission, request, world) → odometry freshness (0.2 s, source ROS + receive steady) → sample ordering → tracking envelope → commit + `/navigation/command_admission`. `updateSetpoint` (50 Hz): terminal/handover stationary → airborne gate (z > 0.5, armed) → odometry lease → health lease → command stale (0.1 s receive and header) + validity window → COMPLETED (position hold + bounded recovery) / velocity-only (`adapt` + `limit`) / PVA: `lioPositionToLocalNed` (translation-only), `enuToNed` v/a, `yawEnuToNed`, `yawRateEnuToNed` → `TrajectorySetpoint` → trace enqueue.
4. **Execution authority**: runtime planner results → `tryCommit*`/`stagePending*` (goal epoch, world identity, transaction watermark, generation monotonic) → command timer `activatePendingIfDueAndFinalize` → `CommandSampler::sample` → `publishIfCurrent(expose)`; map refresh → `publishWorldIdentityIfCurrentAndFinalizeRevocation` (recertify copies or revoke/failClosed).

## State machines

- **ExecutionPhase** (execution_lifecycle.hpp:8-16): InitialHold, TrackingMain, TrackingBackup, StoppedHold, Px4Hold. Set in `committedLifecycleLocked` (execution_authority.hpp:960-988), `observeRetainedCommand` (205-222), `observeSampledSafetyRole` (224-242), `stoppedHold` (287-298), `failClosedLifecycleLocked` (952-958); reset to `{}` in `setAdmissionGoalEpoch`/`beginGoal`/`reset`.
- **ExecutionRecoveryState** (execution_recovery_state.hpp:6-13, transitions 44-94): InitialHold→{TrackMain,TrackBackup,EmergencyBrake}; TrackMain→{TrackBackup,EmergencyBrake,StoppedRecovery(TerminalStopCompleted)}; TrackBackup/EmergencyBrake→StoppedRecovery (CertifiedStopObserved); StoppedRecovery→TrackMain (MainCommitted) only (R5-29); any→Px4Hold on EmergencyCertificationFailed; Px4Hold absorbing.
- **ExecutionExposure**: Unavailable/Available/Suspended/Failed (Failed sticky until goal reset). Safety: Nominal/SafetySuffix. Restart: None/FromRest.
- **Adapter mode flags** (navigation_mode.hpp:223-231): `mode_active_`, `failure_reported_`, `handover_requested_`, `mission_completion_receipt_`, `planner_recovery_pending_`, `px4_local_frame_aligned_`. onActivate (821-875) clears; `safetyStopNavigation`/`failNavigation` (2013-2061) set failure+handover; `onMissionProgress` (1375-1422) sets receipt+handover; onDeactivate (877-905) sets failure. Published projection `NavigationModeStatus.external_mode_state` (355-452).
- **Executor hold handover**: pending/in_flight/confirmed (2651-2686), retry 250 ms steady.
- **Bridge (PX4→ROS)**: `TimestampEvent` (time_validator.hpp:21-29), `ResetObservationStatus` (reset_compensator.hpp:43-53), frame_generation / reset_event_generation / time_generation counters; ring buffer stable gate.
- **Bridge (LIO→PX4)**: `GeometricJumpContinuityReason` (geometric_jump_continuity.hpp:19-32), latch latched/cleared by strictly newer generation (geometric_jump_latch.cpp:14-45), gate reason priority (external_odometry_gate.cpp:24-45).
- **Admission**: `AdmissionStage` × `AdmissionDisposition` (command_admission_assessment.hpp).

## Behavior per branch

- **`NavigationMode::updateSetpoint`** (2063-2582): failure_reported → stationary at safety hold; completion receipt → stationary at completion; handover → stationary at safety/odom position; unarmed or z ≤ 0.5 → zero velocity; odometry lease invalid → `failNavigation`; health missing ≤ 0.5 s after activation → zero velocity, else unhealthy/stale → `failNavigation` (unless experiment bypass, R5-36); command stale/invalid window → `safetyStopNavigation`; STATUS_REJECTED → safety stop; COMPLETED → velocity-only: Hold handover, else position hold + bounded recovery window; velocity-only → `publishVelocityOnlySetpoint` or hold handover; PVA not representable (includes **no alignment**, R5-18) → safety stop; no command within 5 s of airborne → stationary, then safety stop.
- **`onNavigationCommand`** (1143-1352): malformed/expired → reject, retain previous; terminal authority closed → ignore; session mismatch → reject retain; odometry stale → `failNavigation`; non-increasing sample_id → reject retain; tracking envelope exceeded → safety stop; accept → commit + admission receipt + recovery window bookkeeping.
- **`publishVelocityOnlySetpoint`** (1691-1944): role/status not allowed, timestamp contract, quaternion, PX4 stamp, PX4 reset counters changed, `adapt` failure, continuity failure, non-representable → return false (caller requests PX4 Hold); success → velocity + relative yaw.
- **`Px4ExternalOdometryBridgeNode::on_lio`** (215-373): stale identity → reject, gate closed; epoch change → reset baselines + latch observe; conversion/float/variance failure → reject, continuity untrusted; gate closed → count; else publish.
- **`observe_geometric_jump_continuity`** (92-231): invalid config/frame → reseed untrusted; source/generation invalid → reseed untrusted; no baseline/prev invalid/generation change → reseed **trusted**; dt ≤ 0 or dt < min → reseed **trusted** without comparison (R5-08); dt > max → reseed untrusted; else compare → jump/within.
- **`Px4OdometryBridgeNode::on_odometry`** (290-454): invalid/stale/future/regression → reject; restart → new frame generation, clear all; conversion reject; reset transition suppressed → ++reset_event_generation; uncompensable counter change → new frame generation; startup metadata pending → rebaseline; not accepted → suppressed; first output → origin rebase; stable gate; publish.
- **`ResetCompensator::observe`** (147-318): invalid → kInvalidMetadata; counter delta ≠ 1 → kCounterDiscontinuity; metadata invalid/pending; rotation invalid → kInvalidResetRotation; valid reset → update R/T/velocity offset, suppress transition; else apply continuity (velocity offset permanent, R5-13).
- **`ExecutionAuthority::tryCommit*`** (731-921): invalid candidate; no admission goal; failed; goal advanced; world advanced; stale transaction; older generation → predecessor advanced; (IfCurrent) snapshot mismatch / admission rejected; (AndFinalize) finalize failure → full rollback (lineage not bumped).
- **`activatePendingIfDueAndFinalizeLocked`** (1146-1221): failed → no-op; pending without active → clear; not due; token mismatch; pending/active invalid or world mismatch or expired → clear pending; finalize failure → rollback + clear pending; success → lifecycle commit + lineage bump.
- **`publishWorldIdentityIfCurrentAndFinalizeRevocationImpl`** (1011-1144): version changed → superseded; world not advancing → kWorldAdvanced; recertification of active fails → revoke + failClosed + kCandidateRejected (world not consumed); not retained → revoke + failClosed (finalize only if exact owner); pending recertified only if active recertified.
- **`CommandSampler::sampleActive`** (52-109): no active → pending awaiting / none; goal mismatch; before valid_from → awaiting; lease over and past declared end → STOPPED_HOLD endpoint; evaluator null → expired or evaluator failure; exception → evaluator failure.
- **`evaluateTrackingEnvelope`**: speed ≤ 1e-3 → sphere L; else independent long/rev/lat boxes (L + v·window) (R5-16).
- **`loadTrackingExperimentPolicy`**: unknown mode → throw; non-off + zero coefficients → suppress tracking + health (R5-36); non-sim → throw.

## Information needs

| Input | Source | Frame | Clock | Freshness bound | Authority |
|---|---|---|---|---|---|
| Propagated LIO odometry (adapter) | `/lio/odometry_propagated` | lio_odom ENU pose, base_link FLU twist | ROS (source stamp) + steady receive | 0.2 s source & receive (`evaluateExecutionStateFreshness`); run check uses ROS receive only (R5-22) | epoch from typed health; sequence strictly increasing |
| Estimator health | `/lio/health` (`EstimatorHealth`) | — | ROS source + steady receive | 0.2 s; 0.5 s first-sample grace | epoch authority for commands |
| PX4 local position/velocity/heading | `/fmu/out/vehicle_local_position` | PX4 NED, heading NED rad | PX4 µs (timestamp/_sample), steady receive | 0.2 s (steady) for alignment lease | PX4 reset counters (xy/z invalidate alignment) |
| NavigationCommand | runtime `/navigation/navigation_command` | LIO ENU (undocumented, R5-39), yaw ENU | ROS header + valid_until | 0.1 s receive & header; zero future tolerance | Core; mode_activation_id; goal/request/world ids |
| Mission completion receipt | `/navigation/mission_progress` | lio_odom | ROS | ≥ activation time | Core |
| LIO odometry (ext bridge) | `/lio/odometry_propagated` | lio_odom ENU / base_link FLU | ROS stamp → µs | 0.5 s age vs ROS now | (epoch, sequence) high-water |
| LIO diagnostics (ext bridge) | `/lio/diagnostics` string KV (V5) | — | ROS header | 2 s | generation for reset_counter (R5-09) |
| PX4 vehicle_odometry (bridge) | `/fmu/out/vehicle_odometry` | NED/FRD (pose NED only) | PX4 timestamp_sample µs vs ROS now | 0.2 s stale/future; restart ≥1 s regression to <10 s | reset_counter + local/attitude reset metadata (≤ 0.1 s) |
| Active bundle | ExecutionAuthority | lio_odom ENU | ROS ns (valid_from/until, declared_start/end) + double s (N3) | lease valid_until | transaction id, goal epoch, world identity |
| World identity | mapping/world model | — | observation_stamp ns | monotonic (epoch, generation, revision) | world model |

## Bottlenecks

- Adapter `trajectory_mutex_` is shared by 50 Hz setpoint thread, command callback and the state-input thread; held across DDS publish in terminal states (R5-19); every odometry callback also publishes an optional trace and can publish alignment witness under the lock (1537).
- `CandidateBundle::sample()` runs the full contract validation 2× + schedule validation 3× per sample (R5-32).
- ExecutionAuthority destroys bundles/evaluators inside the mutex on most mutators (R5-27); `publishIfCurrent` runs the exposure callback inside the same lock (by design, measured by `lastPublishLockWaitUs`).
- External-vision bridge builds an ostringstream context per sample (R5-10); PX4→ROS bridge builds/publishes a ~35-key DiagnosticArray per sample (R5-05).
- Velocity-only limiter up to 4096 Dykstra iterations per tick (bounded, cheap).
- `RouteProgress::update` two O(segments) scans per update (≤ 4096 waypoints).

## Notes for target architecture

**Keep**
- Single linearization point for command exposure (`ExecutionAuthority::publishIfCurrent`) with exact-pointer + world + goal-epoch checks, transaction watermark, and rollback-safe finalize.
- Typed identities everywhere (localization_epoch, goal_epoch, request_id, bundle_generation, sample_id, mode_activation_id) and strict high-water ordering (bridge source identity, command sample ordering).
- Checked time arithmetic (`checkedDifference`, µs/ns converters that reject 0/overflow) and explicit ROS-vs-steady separation in freshness.
- ENU/NED and FLU/FRD matrices centralized in `navigation_common` and used by both bridges and adapter; the ROS→PX4 odometry conversion math (pose, NED velocity from body twist, FRD rates, full-covariance rotation with symmetry/PSD checks) is correct.
- Bridge never applies its own timesync offset (uXRCE client owns it); fail-closed jump latch that only a strictly newer producer generation clears.
- Separate state-input executor thread for the adapter (px4_ros2 single-thread constraint) with join-before-release lifetime.

**Over-engineered / unnecessary / to fix structurally**
- Two LIO→PX4 frame models (translation-only vs relative-heading) — unify into one latched SE(2) alignment (R5-17, R5-18).
- Velocity-only witness is self-certified (expected = actual, hard-coded validity flags, total bound recomputed with the same formula) — most `adapt()` timing/reset gates cannot fail (R5-20). Either feed independent evidence or delete.
- Const methods that mutate ownership state via `mutable` (R5-26); two admission-scope APIs (R5-28); two cancellation tokens and two deadlines (R5-35); dual seconds/ns time in bundles with dead fallbacks (R5-33, N3); duplicated handoff tolerances (R5-34, N4); parallel lifecycle facets that can disagree (R5-29, N8).
- Contract headers carry heavy logic (CandidateBundle::valid, role schedule, tracking experiment ROS loader). Move logic into compiled, tested units; keep headers as data + small invariants; validate bundles once at construction.
- Deprecated MissionController + identity helpers + 1616-line test still built/exported (R5-24); unused `ReferencePointConverter` (R5-15); dead metrics and enum values (R5-21, R5-39).
- Experiment policy implies safety suppressions from zero coefficients (R5-36); make every bypass an explicit, recorded parameter.
- PX4→ROS reset compensation: velocity offset and position rotation on heading reset need PX4-semantics fixtures before being trusted as evidence (R5-13, R5-14); output quaternion sign continuity (R5-03).
