# Runtime safety decision and temporary-debt ledger

## Purpose

This file is the mandatory history for safety gates, runtime budgets, temporary
bypasses, and cross-layer design decisions in the navigation stack. It prevents
a local workaround from becoming an undocumented product contract. Read it
before modifying FAST-LIO, WorldModel/ROG, SUPER, trajectory control, PX4
External Mode, runtime orchestration, or their validation thresholds.

The target architecture remains:

`FAST-LIO estimator -> product-owned ROG-backed WorldModel -> SUPER main plus
certified backup -> immutable committed bundle -> trajectory controller/OMMPC
-> PX4 ROS 2 External Mode`.

Registration data is not a planning map. UNKNOWN and OUT_OF_MAP semantics belong
to WorldModel. A failed candidate never mutates the committed generation.

## Required entry schema

Every new or changed gate and every temporary bypass must record:

- ID, date, owner, status, affected layer and source/config anchors.
- Physical meaning, units, comparison direction, and authoritative owner.
- Why the gate exists and which unsafe outcome it prevents.
- Derivation or provenance of the value; "works in one simulation" is invalid.
- False-accept and false-reject consequences.
- Runtime cost and expected p50/p95/p99 effect.
- Evidence: unit/characterization, dataset replay, SITL repetitions, sanitizer,
  and hardware certificate where applicable.
- Removal/review condition and the exact command or artifact that closes it.

Every gate has exactly one class:

- `SAFETY_INVARIANT`: fail closed and never tune from mission PASS rate.
- `NUMERICAL_TOLERANCE`: derive from resolution/conditioning and verify scale sensitivity.
- `PERFORMANCE_POLICY`: derive from latency/load distributions and hardware budget.

Allowed statuses are `PROVISIONAL`, `CERTIFIED`, `TEMPORARY_BYPASS`, `REJECTED`,
and `REMOVED`. A `TEMPORARY_BYPASS` may not be closed by deleting its entry.

## Current hard-gate audit

| ID | Gate and owner | Current value | Status | Evidence and risk | Required closure |
|---|---|---:|---|---|---|
| HG-001 | SUPER solve deadline (`PERFORMANCE_POLICY`) | A* attempt 40 ms, A* total 80 ms, solve 180 ms, future-state lead 200 ms | PROVISIONAL | Absolute nesting is unit-tested and prevents multiplied fallback latency. Structured SITL still alternates between A* timeout and later-stage failures, so the values are not certified distributions. | Collect per-stage p50/p95/p99 on repeated structured SITL and dense dataset shadow planning; prove p99 fits 180 ms without starving fallback. |
| HG-002 | EXP corridor plane certificate (`NUMERICAL_TOLERANCE`) | max normalized plane violation 0.01 m | PROVISIONAL | The gate correctly rejected historical 0.2 m excursions. Its config is now independent of optimizer `smooth_eps`, but its provenance and continuous-trajectory coverage remain provisional. Rejections at 0.011-0.014 m must not be handled by silently raising it. | Validate continuous swept trajectory against normalized planes and inflated map; run scale sensitivity on map resolution and conditioning. |
| HG-003 | Vertical guide envelope | guide min/max plus 5 x `smooth_eps` (0.05 m), 20 samples/piece | PROVISIONAL | Prevents artificial altitude excursions, but fixed sampling is not a continuous polynomial certificate and the factor five has only incident-derived rationale. | Replace with analytic extrema or an adaptive bound; test climb/descent, stitched boundary PVAJ, and lateral-obstacle altitude. |
| HG-004 | Vehicle dynamic and flatness envelope | V/A/J 12/12/30, body rate 5 rad/s, thrust acceleration 6-25 m/s2 | PROVISIONAL | Product config says X500-derived, but controller/PX4/hardware provenance is incomplete. Backup missions may lower these limits. | Link airframe/controller evidence and dataset/SITL distributions; certify continuity and actual PX4 tracking before hardware. |
| HG-005 | Planning radius invariant | 0.35 + 0.25 + 0.05 + 0.10 + 0.05 = 0.80 m | PROVISIONAL | Ownership and sum are explicit and config-validated. Component error budgets are not yet tied to measured distributions. | Derive tracking/localization/mapping p99 independently and preserve the sum invariant. |
| HG-006 | Runtime input freshness | cloud/corrected/propagated maximum age 0.5 s; pairing skew 0.1 s | PROVISIONAL | Fail-closed works, but 0.5 s is far larger than a 0.2 s planning lead. Recent runs show health exits from scheduling gaps around this boundary. | Express per-stream deadlines from rates and braking envelope; measure dataset and loaded-SITL gap distributions. |
| HG-007 | Safety suffix anchor | maximum state-to-command anchor error 0.75 m | PROVISIONAL | Prevents retaining a geometrically detached suffix, but value is independent of speed, stopping distance and localization confidence. | Replace with speed/covariance-aware contract or derive a certified worst case. |
| HG-008 | Planner watchdog | 1.0 s | PROVISIONAL | Protects command publication from a hung solve, but exceeds the internal 0.18 s solve deadline by 5.6x and invalidates all command availability on expiry. | Align watchdog with cancellability and measured worst-case stage latency; test cancellation and immutable commit under load. |
| HG-009 | Goal connectivity | SUPER uses 2 map voxels; runtime completion uses 0.20 m | PROVISIONAL | Endpoint is now resolved before corridor construction, but two different tolerances remain and can disagree in 3-D. | Introduce one explicit XY/Z goal contract shared by connected_goal, NO_NEED and mission completion. |
| HG-010 | Retained-suffix swept validation | spatial step 0.5 inflated-map resolution, time step clamped 2-50 ms | PROVISIONAL | Adaptive segment checks fail closed for OCCUPIED and OUT_OF_MAP. Maximum step and map revision stability are not yet recorded in the certificate. | Attach map revision/generation and segment certificate to the committed bundle; test obstacle between legacy 50 ms samples. |
| HG-011 | Hardware Mid-360 visibility | hardware blocked unless explicit certificate flag is true | CERTIFIED | Fail-closed deployment gate exists. No real-flight visibility certificate exists yet. | Keep hardware blocked until mounting, FOV/blind zones, accumulated observations and motion envelope are certified. |
| HG-012 | CIRI overlap/seed tolerances (`NUMERICAL_TOLERANCE`) | overlap threshold plus hard-coded 0.01 m and 0.25 ratio; seed clearance `robot_r - 0.01 m` | PROVISIONAL | Different meanings currently share unnamed constants and can reject dense geometry inconsistently. | Name each quantity, record units/reason codes and test resolution scaling before changing values. |
| HG-013 | Backup corridor certificate (`SAFETY_INVARIANT`) | position penalty violation accepted up to 0.2 | PROVISIONAL | This differs from the 0.01 m EXP certificate and can reduce the margin on the trajectory intended to be safest. | Replace penalty-derived authorization with the same normalized continuous geometric certificate used by EXP; preserve certified seed fallback. |

## Temporary-bypass register

| ID | Owner/date | Scope | Safety impact | Evidence | Removal condition | Status |
|---|---|---|---|---|---|---|
| TB-001 | planning / 2026-08-24 | EXP jerk objective disabled; analytic V/A/J hard gate remains authoritative | False-reject and runtime-tail risk; no false accept if hard gate is complete | Open-space SITL and trajectory unit tests | Objective-enabled A/B on structured SITL and dense snapshot replay has equal certificates and bounded p99 | TEMPORARY_BYPASS |
| TB-002 | planning / 2026-08-24 | Corridor-center attractor disabled after slalom/feasibility regression | May reduce convergence margin but does not bypass the corridor certificate | Incident comments and nominal SITL | Deterministic optimizer A/B proves a replacement improves feasibility without path drift | TEMPORARY_BYPASS |
| TB-003 | mapping/planning / 2026-08-24 | CIRI limited to one iteration and obstacle skip 2 | May reduce corridor volume or omit redundant surfaces; inflation/collision gates remain authoritative | Current SITL only | Dense snapshot benchmark proves constraint coverage, feasible rate and p99 across the iteration/skip matrix | TEMPORARY_BYPASS |

The following are not bypasses and must not be removed as cleanup: certified
emergency brake, transactional candidate commit, stale-solve gating,
endpoint-only UNKNOWN profile, OUT_OF_MAP fail-closed checks, and hardware
visibility blocking. Add any new bypass before or in the same commit and
reference its ID from the code/config comment.

## Performance and orchestration review contract

Correctness and performance are coupled: a slow map update can stale propagated
state, consume the solve budget, and trigger a valid but avoidable safety stop.
Each required scenario therefore reports CPU, resident memory, callback gaps,
map update, A*, CIRI, EXP, backup, total solve, and command-sampling p50/p95/p99.
Report both success and fail-closed generations; averages alone are insufficient.

Optimization order:

1. Remove duplicate conversions, copies, map queries and repeated failed solves.
2. Bound input density at a documented evidence resolution without deleting
   obstacle geometry required by inflation or corridor construction.
3. Bound each stage with one absolute deadline and explicit remaining budget.
4. Profile before algorithmic changes; preserve test vectors and safety output.
5. Change A*/CIRI/MINCO mathematics only after behavior and benchmark baselines
   are frozen.

Known orchestration risks:

- Mapping and planning currently execute sequentially in the same runtime
  callback. Dense clouds can consume planner budget; long planning can delay
  observation consumption. Do not compensate by merely increasing deadlines.
- The observation owner currently retains one pending cloud, so replacement
  rate must be reported by load bucket as well as a cumulative counter.
- Optimizer feasibility retries and backup refinement do not yet receive the
  absolute solve deadline. Record retry count, reason, best violation and
  remaining budget before deciding whether to retain or redesign them.
- CIRI work can approach quadratic growth with obstacle points in a corridor.
  Bucket latency by input points, occupied voxels, local CIRI points, planes and
  corridor count; mean cloud rate is not a sufficient scale metric.

## Dataset validation contract

Recorded Mid-360 and IMU data cannot prove controller tracking or closed-loop
avoidance by itself, but it is essential evidence for the parts simulation
under-stresses:

- FAST-LIO synchronization, deskew, corrected/propagated state and health.
- Cloud density, non-finite handling, observation pairing and timestamp gaps.
- ROG update cost, memory growth, occupancy/inflation density and map revision.
- CIRI constraint count and latency on vegetation, thin branches, clutter and
  repeated surfaces.
- Shadow planning from recorded propagated states and scripted goals. Commands
  must not control hardware; record candidate result, certificate, latency and
  generation only.

Required dataset classes are open outdoor, dense trees/vegetation, thin
structures, narrow passage, texture-poor geometry, aggressive yaw, altitude
change, reordered/missing observations and prolonged runs. Preserve raw sensor
rates; also run a documented downsample matrix to locate the accuracy/runtime
frontier. Dataset PASS never substitutes for closed-loop SITL or hardware gates.

## Decision history

### 2026-08-25 - WM-0 coherent observation lifecycle accounting

- Added a mutex-coherent lifecycle state machine for the single-latest mapping
  inbox. Ingress accept/reject, pending replacement/discard, mapping
  start/publish/failure, pending/in-flight gauges and illegal-transition
  violations are captured as one consistent diagnostic snapshot.
- Added steady-clock observation pair wait, input-lock wait and planning
  scheduling-gap distributions. Source timestamp pairing/freshness remains in
  ROS time; durations never use bag/simulation time.
- Dataset acceptance now verifies the lifecycle conservation equation and
  compatibility between `dropped_cloud_count` and the explicit replacement
  counter. Accepted input without a terminal disposition can no longer PASS.
- Independent review found a separate behavior defect: mapping currently occurs
  after planner execution-state and completed-mission early returns. This WM-0
  batch accounts those paths as failures but does not reorder behavior; the
  following P0 mapping-owner batch must make corrected observation integration
  independent of propagated-state/mission availability.
- The same review found freshness uses absolute ROS-time difference, which
  conflates future timestamps with stale past timestamps. HG-006 remains
  PROVISIONAL; future, rewind and generation handling must be split before its
  value is certified. No freshness value changed in WM-0.
- Canonical 1x evidence:
  `.artifacts/runtime/dataset-20260824T172807-65510` PASS with 2,756 accepted,
  started and published observations; zero replacement/discard/failure,
  pending/in-flight or accounting violation. All mapping timing distributions
  contain 2,756 samples. Pair-wait p99 is 65.128 ms; mapping callback p99 is
  17.144 ms (45.440 ms maximum), and map-slide p99 is 17.057 ms. These are
  characterization values, not new acceptance thresholds.

### 2026-08-25 - P0-A corrected mapping is independent of planner state

- Reordered the existing single-threaded cycle so a valid cloud plus corrected
  pose is integrated and accounted before propagated-state and mission-FSM
  gates. Missing/stale/invalid propagated state or a completed trajectory can
  no longer suppress the current corrected mapping observation.
- Corrected mapping pose now requires finite position/quaternion and normalizes
  the quaternion before ROG. Signed freshness classifies INVALID, STALE and
  FUTURE without changing the existing 0.5 s magnitude.
- Closed-loop artifact
  `.artifacts/runtime/external-mode-check-20260824T173726-67414` completed its
  mission with 74/74 complete solve records and no mapping failure,
  in-flight/pending, stale, future or invalid-state counters. It replaced 133
  of 380 accepted clouds because mapping and planning remain one 10 Hz
  callback. After applying the same mapping-integrity gate to dataset and
  closed-loop reports, its authoritative verdict is FAIL with reason
  `mapping replaced an unconsumed cloud: 133`; it does not close the independent
  scheduling/snapshot gate.

### 2026-08-25 - Stage immutable WorldModel ownership before concurrency

- Accepted ADR-011 after the 2x dataset overload evidence. ROG-Map is mutable
  and SUPER currently reaches 19 vendor methods across 49 call sites; moving
  mapping to another callback group now would introduce races rather than fix
  ownership.
- The migration order is product query boundary, explicit mapping revision,
  genuinely immutable snapshot, then independent scheduling. A map lock held
  for the full solve, an unbounded queue, or two divergent mutable maps are
  explicitly rejected.
- Independent orchestration review additionally requires single-latest inbox
  accounting, one pinned revision per solve, a short latest-world
  revalidation/commit gate, reset-driven world generations, bounded snapshot
  lifetime and deterministic worker shutdown. These are acceptance contracts,
  not optional performance enhancements.
- No runtime behavior or hard-gate value changes in this decision batch.

### 2026-08-24 - Establish mandatory gate and bypass governance

- Added explicit per-generation SUPER stage/deadline/result telemetry and report
  ingestion before further threshold changes.
- Rejected raising the 0.01 m corridor threshold after one failure. The issue is
  tracked as HG-002 because optimizer smoothing and geometric certification are
  conflated.
- Rejected an unproven bounded penalty retry after its SITL run failed earlier
  in a different corridor stage; the experiment was removed rather than left as
  dormant behavior.
- Fixed terminal-goal ownership by resolving the endpoint before corridor
  construction; no corridor or safety gate was bypassed.
- Structured SITL remains open because failures vary among A*, backup/MINCO and
  external-odometry health. Per-generation trace is now the authority for the
  next optimization decision.
- Independent orchestration review found that dataset replay currently stresses
  FAST-LIO and ROG but publishes no goals, so it does not execute A*/CIRI/MINCO.
  Dataset planner claims require a deterministic shadow-planning harness over
  immutable recorded WorldSnapshots.

### 2026-08-25 - Make recorded time authoritative during dataset replay

- The first full `aist-mid360-drive` replay processed all 2,772 lidar scans and
  55,435 IMU messages in FAST-LIO with no estimator input drops, but produced no
  ROG update. Every navigation snapshot was correctly rejected as roughly
  74 million seconds stale because the 2024 sensor headers were compared with
  the 2026 host wall clock.
- Fixed the harness time-domain contract: dataset nodes use ROS simulated time
  and rosbag publishes `/clock`. The live 0.5 s freshness gate is unchanged;
  no dataset-only freshness bypass or threshold relaxation was introduced.
- Artifact before the fix:
  `.artifacts/runtime/dataset-20260824T165921-59007`. This artifact is a valid
  estimator/load baseline and a harness FAIL, not mapping/planning evidence.
- The corrected 1x replay artifact
  `.artifacts/runtime/dataset-20260824T170750-60656` passed observation
  ownership (2,756 received and accepted, zero drop/mismatch/stale/exception),
  but exposed a telemetry defect: ROG timings existed internally and were not
  published. Mapping diagnostics now export aggregate per-update timing and
  scale counters; the PASS artifact remains insufficient for CIRI/MINCO because
  the dataset workflow published no goals.
- A subsequent 2x replay was incorrectly reported PASS while the single-slot
  observation owner replaced 1,378 of 2,756 accepted clouds before mapping.
  Artifact: `.artifacts/runtime/dataset-20260824T171418-61902`. Dataset
  acceptance now fails on any mapping replacement, stale/mismatched pair,
  invalid execution state, processing exception, or received/accepted
  accounting mismatch. This is an overload characterization, not authority to
  add a queue or discard input silently; mapping/planning ownership separation
  remains the architectural closure.
