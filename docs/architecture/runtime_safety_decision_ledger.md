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

### 2026-08-25 - WM-3B exact candidate and latest-world commit authorization

- SUPER now constructs the exact executable position/yaw polynomial bundle,
  including inherited and new BACKUP role intervals, before taking any commit
  lock. All five normal commit paths and the emergency-brake path use one
  product-owned `WorldCommitAuthorizer`; runtime result inspection is no
  longer treated as authority after an internal command swap.
- Authorization validates only the unexecuted suffix against one latest
  immutable snapshot, then uses the short snapshot publication gate to prove
  that revision is still current while the swap occurs. The lock order is
  publication gate, solve-commit mutex, command-bundle mutex. A generation
  reset or a publication between validation and the gate rejects the candidate
  without changing command/history state; there is no retry loop.
- HG-010's existing adaptive profile remains authoritative in this refactor:
  spatial step is half the inflated resolution with 0.02 m minimum and the
  velocity-derived time step is clamped to 2-50 ms. OCCUPIED and OUT_OF_MAP
  fail closed and every adjacent segment is ray-checked. These values remain
  PROVISIONAL and were not tuned in WM-3B.
- BACKUP is deliberately not redefined as persisted occupancy KNOWN_FREE in
  this batch. With endpoint-only observation and raycasting disabled, current
  SUPER backup provenance means visibility/obstacle-free certified while
  occupancy UNKNOWN remains traversable. Requiring KNOWN_FREE here would be an
  undocumented behavior change and a likely false-reject source. A stronger
  product visibility certificate is a separate behavior batch and remains a
  hardware-flight prerequisite.
- Focused evidence passes 18 trajectory/builder/sweep tests, five snapshot-store
  tests (including a real candidate rejected after WORLD_ADVANCED), and all
  eight runtime test targets. Independent post-change review found no remaining
  P0 source blocker for a sequential-only checkpoint.
- The 1x dataset artifact
  `.artifacts/runtime/dataset-20260824T191351-105860` remains FAIL: 2,581 of
  2,756 accepted observations were published and 175 pending observations were
  replaced. Accounting is valid, processing exceptions are zero, mapping
  callback p99 is 17.436 ms and snapshot export p99 is 7.000 ms. The dataset
  publishes no goals, so it exercises neither the new authorization path nor
  A*/CIRI/MINCO; it is evidence that the known single-timer inbox scheduling
  defect persists, not permission to queue, downsample or relax a gate.
- WM-3C mapping worker scheduling and closed-loop SITL authorization evidence
  remain explicitly out of scope for this checkpoint. This commit is not
  authority for concurrent mapping publication.

### 2026-08-25 - WM-3A revisioned snapshot store and solve-local pin staging

- Added a product-owned `WorldSnapshotStore` with acquire/release publication,
  monotonic generation/revision enforcement and a short authorization gate.
  Mapping publication and a future final trajectory commit can be linearized
  without holding a map lock during A*/CIRI/MINCO or candidate revalidation.
- Each sequential solve now loads one coherent `PinnedWorldSnapshot`, propagates
  that immutable pointer to SUPER/A*/corridor on the planning thread and emits
  pinned generation/revision/observation timestamp in its decision trace.
  Mapping no longer pushes three planner pointers as part of publication.
- Deterministic tests prove null/invalid/non-monotonic publication rejection,
  immutable pin lifetime, stale-identity authorization rejection and that a
  publication cannot interleave the authorized commit callback. The store is
  deliberately non-copyable.
- This batch does not yet authorize SUPER's internal trajectory swaps; WM-3B
  must revalidate candidates on a newer revision and route every internal
  commit site through the store gate. Strict `pinned == latest` rejection and
  cancel-on-every-map-update were explicitly rejected because a 10 Hz map can
  permanently starve a non-trivial solve.
- Sequential dataset artifact
  `.artifacts/runtime/dataset-20260824T185157-96889` remains FAIL with nine
  pending-cloud replacements: 2,747/2,756 observations published, no mapping
  failure/exception/accounting violation, export p99 8.793 ms and callback p99
  18.461 ms. This confirms the store did not add a latency tail but also proves
  the timer-phase/inbox scheduling defect is nondeterministic and still open.
  No queue, gate relaxation or threshold change was introduced to alter the
  verdict.


### 2026-08-25 - WM-2B immutable revisioned WorldModel snapshot

- Runtime now exports and publishes a detached `RogWorldSnapshot` after each
  successful corrected observation. SUPER, A* and corridor construction pin
  that product-owned view; no planner query aliases mutable ROG storage. A
  publication advances exactly one revision and carries the scan timestamp.
- Virtual-plane values are semantic provenance from ROG, not newly tuned
  gates. Evidence classification uses the configured raw ground/ceiling and
  inflated classification uses `inflation_resolution * (1 + inflation_step)`.
  Legacy segment traversal intentionally differs: inflated OCCUPIED uses the
  raw planes, while inflated UNKNOWN uses one inflation-resolution band.
  Exact and plus/minus epsilon parity tests lock both contracts; they must not
  be collapsed into one shortcut without an explicit behavior-change batch.
- Boundary testing exposed that WM-2A exported raw probability classification
  and therefore omitted ROG's index-domain virtual-plane semantics. Export now
  stores the public `getGridType(index)` result. ROG export and runtime parity
  suites pass after the correction.
- Snapshot construction validates finite positive geometry, voxel-count
  overflow, detached array sizes, virtual ordering and generation identity
  before publication. A mutable ROG update or subsequent snapshot export that
  throws is fail-stop: the node rethrows after a FATAL diagnostic rather than
  retaining the old snapshot and later relabelling a partially mutated map.
  This is a safety invariant, not a retry bypass; automatic generation rebuild
  remains future work and requires deterministic fault-injection tests.
- Diagnostics now report world generation/revision, detached byte size and
  snapshot export latency separately from ROG update time. No deadline,
  freshness value, downsampling policy, UNKNOWN policy or planner mathematics
  changed. Dataset p50/p95/p99 and RSS evidence are still required before
  enabling independent mapping/planning workers.
- First 1x Mid-360 replay artifact
  `.artifacts/runtime/dataset-20260824T181711-85406` is correctly retained as
  FAIL: 2,521 of 2,756 observations published and 235 pending observations were
  replaced. The 4,019,364-byte snapshot export dominates mapping with
  p50/p95/p99 16.504/22.367/27.789 ms versus ROG update
  0/0.842/2.884 ms; mapping callback p99 is 31.258 ms. These measurements are
  characterization, not thresholds. Do not hide this overload by relaxing
  integrity/freshness gates or adding an unbounded queue. Optimize immutable
  representation/copy work and repeat 1x/2x plus dense vegetation evidence.
- O1 representation optimization shares the detached immutable nearest-offset
  table and precomputes circular-axis mappings while retaining logical x/y/z
  cell order. It changes no map/query policy. Intermediate artifact
  `.artifacts/runtime/dataset-20260824T182847-89816` remained FAIL but improved
  export p50/p95/p99 to 6.059/11.110/13.694 ms and replacements from 235 to
  178. After removing the remaining per-voxel modulo operations, artifact
  `.artifacts/runtime/dataset-20260824T183449-91366` PASSed exact-once mapping:
  2,756 received/started/published, zero replacement/failure/violation and
  revision 2,756. Export p50/p95/p99 is 6.038/9.554/10.876 ms and callback p99
  is 22.020 ms. Peak live snapshot count is two with 6,472,320 owned bytes;
  the single shared metadata table is 783,204 bytes. This one PASS is screening
  evidence only; repeatability, 2x overload and dense vegetation remain open.
- Pre-commit adversarial gates delete snapshot copy/move operations so lifetime
  counters cannot underflow, and exhaustively compare logical base/inflated
  exports after signed x/y/z map slides. Release suites pass; the focused ROG
  ASan suite passes all nine tests including the slide matrix. Navigation ASan
  remains part of WM-3 because its isolated invocation lacked the required
  ASan-built dependency overlay; this failure did not indicate a sanitizer
  finding and is not recorded as a PASS.
- Verification:
  `ctest --test-dir build/rog_map_vendor --output-on-failure` and
  `ctest --test-dir build/navigation_runtime --output-on-failure`.

### 2026-08-25 - WM-2A detached compact planning-grid export

- Rejected copying `ROGMap`: mutex/queue state is not cloneable, submaps are
  shared pointers, and a shallow copy would falsely label mutable aliases as a
  snapshot. Raw probability/counter cloning was also rejected because it
  retains roughly five times the storage the planner consumes.
- Added one owner-only vendor export that materializes base semantic state,
  inflated occupancy and optional inflated UNKNOWN into detached byte arrays
  in logical global x/y/z order. It preserves circular-index resolution and
  nearest-neighbor offset order while exposing no mutable buffer to the
  product contract.
- The small fixture exports 969,710 bytes in 1,432 us in one characterization
  run. This is not a performance threshold or certification. Product config is
  expected to produce about 3.24 MB per snapshot; repeated dataset p50/p95/p99
  and RSS slope are required before publication is enabled as runtime policy.
- Export tests exhaustively compare base/inflated semantics away from virtual
  planes (which remain explicit snapshot metadata) and prove an earlier value
  does not alias a later update. No map update, query threshold, planner
  behavior or callback scheduling changed in this batch.

### 2026-08-25 - WM-1 product-owned query boundary without behavior tuning

- Added the Eigen/STL-only `navigation_world_model::WorldModelView` contract
  and a runtime-owned ROG adapter. A*, corridor construction and SUPER now use
  named evidence/inflated layers and UNKNOWN policies rather than mutable
  `ROGMapROS` pointers, ROG config objects and ambiguous line-query booleans.
- The adapter delegates ROG ray traversal, coordinate quantization,
  nearest-cell tie breaking and occupied-point iteration order exactly. No
  threshold, UNKNOWN policy, OUT_OF_MAP behavior, A*/CIRI/MINCO mathematics or
  callback scheduling changed in this batch.
- Map mutation remains exclusively in `navigation_runtime`; the dead planner
  `getMap()` and `updateROGMap()` escape hatches were removed. Successful
  integration advances adapter revision metadata, but the backing ROG storage
  remains mutable and sequential. This is explicitly not an immutable
  snapshot and creates no authority for concurrent map update and planning.
- Compile and adapter parity tests cover product geometry/identity,
  classification, both coordinate layers, all layer/UNKNOWN-policy segment
  combinations, local-box clamping and exact occupied-point ordering. Dense
  fixture, map-slide, algorithm-level and SITL parity remain required before
  Batch 1 is certified and before WM-2/WM-3 concurrency work starts.
- No bypass or hard-gate value was added or changed. The remaining direct
  `rog_map_vendor` build dependency supplies legacy neutral utility types and
  geometric helpers; removing that baggage belongs after behavioral freeze.

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

### 2026-08-25 - WM-3C-1 give mutable ROG one mapping worker owner

- Moved mutable ROG update/export out of the planning timer into one explicitly
  started and joined `MappingWorker`. Cloud plus exact corrected-odometry pairs
  enter a bounded latest-only READY slot; one WAITING, one READY and one
  IN_FLIGHT observation are the maximum lifecycle population. SUPER only pins
  immutable snapshots from `WorldSnapshotStore`.
- Pair promotion and WAITING-to-READY transfer are linearized under the input
  mutex. A strictly increasing source-stamp watermark rejects duplicates and
  reordered observations before mutable map processing. No FIFO, dataset-only
  bypass, input downsampling, deadline relaxation or freshness-value change was
  introduced.
- Mutable update/export/publication exceptions are fail-stop. Shutdown stops
  acceptance, cancels planning, terminally disposes WAITING/READY observations,
  allows an already-mutating update to finish, and joins the worker before map,
  store or planner destruction.
- Mapping performance diagnostics now originate once, immediately after the
  observation reaches terminal `PUBLISHED`. They are no longer replayed by the
  planning timer. Point-cloud decode, pair wait, ROG stages, detached snapshot
  export and total mapping latency therefore have one sample per published
  revision.
- Canonical 1x evidence:
  `.artifacts/runtime/dataset-20260824T194937-117639` PASS with 2,756 received,
  accepted, started and published observations; revision sequence 1 through
  2,756 is unique and strictly increasing; replacement, discard, failure,
  pending, in-flight, nonmonotonic and accounting-violation counts are zero.
  Each mapping timing distribution contains exactly 2,756 samples. Mapping
  callback p50/p95/p99 is 3.271/5.941/17.089 ms (46.477 ms max), and detached
  snapshot export p50/p95/p99 is 3.158/5.026/5.546 ms (11.128 ms max). These
  values characterize this dataset/host; they do not create or relax a gate.
  Planning-timer input-lock wait and scheduling-gap samples remain planning
  cadence metrics and are not counted as mapping-publication latency.
- Focused build, eight direct C++ test binaries and 80 runtime-contract tests
  pass. The current shell's `ctest` wrapper still cannot import
  `ament_cmake_test`; direct binaries are the source evidence until the ROS
  Python harness environment is repaired. TSan/ASan, 2x/dense vegetation and
  closed-loop SITL remain required before WM-3 is certified complete.

### 2026-08-25 - WM-3 sanitizer screening and concurrency stress

- Full, mutually isolated ASan, UBSan and TSan overlays were built from the
  product dependency closure. Sanitizer binaries were not linked against a
  Release workspace. `ldd` confirms the focused TSan binaries load
  `libtsan.so.2`.
- ASan exposed a real 6,912-byte leak in the yaw interpolation path:
  stack-local `BandedSystem` buffers allocated by `create()` had no destructor.
  `BandedSystem` now owns destruction through RAII, initializes its dimensions,
  and rejects copying so the raw allocation cannot be double-owned. The ASan
  package gate then ran ten CTest entries with zero error, failure or skip.
- A compiler-detected undefined return path in `SUPER_RET_CODE_STR()` now
  returns a deterministic diagnostic containing the unknown numeric code. It
  is not coerced to SUCCESS or an existing planner disposition. The UBSan
  package gate ran the same ten entries with zero error, failure or skip.
- Added deterministic concurrency stress for MappingWorker producer admission,
  READY replacement, one IN_FLIGHT observation and two simultaneous shutdown
  callers. It passed 50 ASan repeats, 50 UBSan repeats and 200 focused TSan
  repeats with exact lifecycle conservation and no sanitizer report.
- Added cross-component stress for immutable world publication, latest-world
  authorization, atomic `CmdTraj` commit and concurrent command-bundle reads.
  Each execution performs 10,000 publish/authorize/sample iterations and checks
  that trajectory marker, certificate identity, role and generation never mix.
  The final rendezvous-enforced form passed one ASan, one UBSan and one focused TSan
  execution. The existing CmdTraj commit/read stress additionally passed 20
  focused TSan repeats of 500 writer and 500 reader operations.
- The package-wide TSan command remains BLOCKED, not PASS: other test entries
  completed their assertions, but the system `libOpenNI2.so` constructor reports
  an unlock-of-unlocked-mutex warning, while the 10,000-iteration
  cross-component test did not complete before the default 60-second CTest
  timeout under TSan. Focused runs use
  `halt_on_error=1` plus a temporary `mutex:libOpenNI2.so` suppression only for
  the externally attributed constructor warning; no product stack or race is
  suppressed.
- This evidence closes the focused MappingWorker/WorldSnapshotStore/CmdTraj
  race screen only. A ROS `MultiThreadedExecutor` node-destruction test with an
  in-flight mapping barrier, fail-stop subprocess coverage, dataset 2x/dense
  vegetation, and closed-loop SITL remain required before WM-3 is certified.
  No queue, timeout/freshness relaxation, input downsampling or scheduling
  bypass was introduced to make the sanitizer runs pass.

### 2026-08-25 - Astar node ownership exposed by full-node shutdown ASan

- The first ASan execution of the real `SuperNavigationNode` destruction path
  reported a direct leak of 1,419,661,656 bytes in 25,351,101 allocations. The
  allocation authority was `Astar::grid_node_buffer_`: its constructor created
  one heap `GridNode` per configured search voxel while its destructor owned no
  corresponding release. A second out-of-local-map search branch also created
  a temporary heap node that was converted to value positions and then lost.
- `grid_node_buffer_` now uses `std::unique_ptr<GridNode>` while retaining the
  same indexed vector, individual node allocation, stable heap addresses,
  hashes and raw non-owning pointers used by queues and `father_ptr`. Copy and
  move are explicitly rejected. This closes normal destruction and partial
  constructor failure without changing A* mathematics, map dimensions or
  search policy. The one-shot search node is stack-owned and remains alive
  through the synchronous path-to-position conversion.
- The full-node shutdown test then passed under the coherent ASan overlay with
  `halt_on_error=1:abort_on_error=1:detect_leaks=1`; LeakSanitizer reported no
  remaining leak. Measured process evidence was 13.22 seconds elapsed and
  3,326,428 KiB peak RSS on this host. These values are evidence, not a new
  timeout or memory gate.
- Performance debt remains explicit: the product config still constructs more
  than 25 million separately allocated A* nodes and later releases them one by
  one. Contiguous/lazy storage and smaller search-domain representations require
  separate behavior-parity and benchmark work; no map-size reduction,
  downsampling or test-only bypass was used to close this ownership defect.

### 2026-08-25 - WM-3 real-node mapping shutdown certificate

- Added a notification-only `MappingLifecycleObserver` dependency seam. It has
  no mutable map, store, worker or node handle, returns no decision, and its
  callbacks are `noexcept`. The production constructor delegates with no
  observer, so mapping validation, update, export, publication, fatal handling,
  planner behavior and configured gates are unchanged.
- The integration test runs the real node library, product SUPER/ROG config and
  a two-thread ROS executor. An exact corrected-odometry/cloud pair reaches the
  real mutable ROG update; the observer blocks after `updateMap` and before
  detached export. After executor quiescence and node removal, destruction is
  proven to wait for the in-flight worker. Releasing the barrier permits one
  publication and shutdown completes with received, accepted, started and
  published all equal to one; failure and WAITING/READY/IN_FLIGHT/pending/
  violation gauges are zero.
- Topics and the driver node use a per-process suffix so concurrent sanitizer
  jobs cannot satisfy discovery or lifecycle assertions for one another.
  Executor cancellation and observer release both have unconditional RAII
  cleanup paths; the test does not sleep in place of mapping or expose a test
  handler that can bypass production behavior.
- Evidence: Release passed five consecutive repetitions. The coherent ASan
  overlay passed with leak detection enabled after the separately recorded
  Astar ownership correction; UBSan passed with halt-on-error. Focused TSan
  passed under ASLR-disabled execution with `halt_on_error=1` and the existing
  narrow `mutex:libOpenNI2.so` constructor suppression; no product stack was
  suppressed. Full package TSan and fail-stop subprocess certification remain
  separate gates.

### 2026-08-25 - WM-3 mapping fail-stop subprocess certificate

- The default `mappingFailStop` production symbol is exercised directly in a
  standalone subprocess death test. A standard exception must emit the stable
  post-mutation FATAL prefix plus its injected reason and terminate by
  `SIGABRT`; a non-standard exception must emit the catch-all FATAL reason and
  terminate by the same signal. Normal return, a different signal or missing
  reason fails the test.
- GoogleTest's thread-safe death-test mode re-executes the child instead of
  relying on a fork from sanitizer runtime helper threads. No production fatal
  handler, environment gate or nonfatal test policy was added. The runtime node
  continues to pass this exact symbol directly to `MappingWorker`.
- Release, ASan with `detect_leaks=1`, and UBSan with halt-on-error each executed
  both branches and passed 2/2 without sanitizer findings or option changes.
  This certifies the default termination and reason contract. Deterministic
  exception injection through every real map update/export/publication stage
  and the package-wide TSan gate remain separate work; this subprocess result
  does not promote WM-3 or closed-loop flight to complete.

### 2026-08-25 - Dataset ingress count contract and 2x screening

- Closed a dataset-report false-PASS path: the runner previously drained after
  only 90 percent of expected raw messages and the monitor observed every topic
  with BEST_EFFORT QoS. Artifact
  `.artifacts/runtime/dataset-20260824T220548-252204` therefore reported PASS
  while its monitor saw only 2,759 of 2,772 LiDAR messages. Product mapping was
  exact in that run, but raw-ingress evidence was incomplete.
- Only the dataset monitor's raw IMU and LiDAR subscriptions now use RELIABLE
  QoS, with bounded depths taken from the configured product ingress capacities
  (4,096 and 16). Derived odometry, diagnostics, mapping, SITL and PX4 monitor
  streams remain BEST_EFFORT. The runner persists normalized expected counts
  from validated bag metadata and waits for exact raw counts within the existing
  drain timeout. The final report rejects missing, short, duplicate, zero or
  malformed source-count evidence. The existing 90-percent expected-rate
  verdict gate remains unchanged for stream-rate health; exact metadata counts
  are additionally authoritative for raw dataset ingress.
- Runner provenance now includes bag source duration and replay-process wall
  start/end. Reported achieved rate is a conservative process-envelope metric
  because it includes process startup; it is characterization and does not add
  or relax a hard threshold.
- Dirty screening artifact
  `.artifacts/runtime/dataset-20260824T221409-255299` requested 2x and passed:
  IMU 55,435/55,435, LiDAR 2,772/2,772, mapping received/accepted/started/
  published/revision 2,756, and zero replacement, failure, pending, in-flight or
  accounting violation. Source duration 277.167 seconds replayed in 141.807
  seconds, achieved rate 1.955x (97.73 percent of requested). Mapping callback
  p50/p95/p99 was 3.134/5.286/16.984 ms and snapshot export was
  3.035/4.435/4.741 ms; each distribution had exactly 2,756 samples. This is a
  harness screening result because source provenance records a dirty worktree,
  not the required clean 3/3 throughput certificate.
- The prepared catalog still contains only `aist-mid360-drive`; it is not
  identified as dense vegetation and has no ground truth. Dense vegetation and
  thin-branch mapping evidence remain BLOCKED pending a prepared bag with
  mounting/extrinsic/timestamp/source checksum provenance. Dataset replay also
  has no goals, so it does not certify A*, CIRI or MINCO; recorded-world shadow
  planning or closed-loop scenarios remain necessary.
- Clean post-checkpoint 2x throughput evidence passed 3/3 at commit `89dfb60`:
  `.artifacts/runtime/dataset-20260824T221835-256982`,
  `.artifacts/runtime/dataset-20260824T222102-257362`, and
  `.artifacts/runtime/dataset-20260824T222328-257732`. Every run observed exact
  IMU 55,435/55,435 and LiDAR 2,772/2,772, published exactly 2,756 mapping
  revisions, and ended with zero replacement, failure, pending, in-flight,
  rejection or accounting violation. LIO ended TRACKING/navigation-valid with
  zero rejected corrections. Achieved process-envelope rates were
  1.95458/1.95454/1.95454x. Mapping callback p99 was
  16.794/17.084/17.071 ms and snapshot-export p99 was
  4.679/4.763/4.783 ms. Host load and pre-existing swap residency were recorded
  around the sequential run; no concurrent product runtime, RViz, build,
  downsampling, queue, threshold or deadline change was used.
- This closes the exact-ingress 2x AIST throughput repeatability gate only.
  Clean 1x repeatability, PID-aware CPU/RSS, dense vegetation/thin branches,
  recorded-world planning and all Gazebo closed-loop/speed gates remain open.
