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
| HG-009 | Goal connectivity | Shared 3-D completion/connectivity tolerance 0.20 m | PROVISIONAL | Planner endpoint resolution and runtime completion now use one product-owned value; scale/provenance and mission distributions remain provisional. | Validate goal acceptance/rejection across map resolutions and 3-D endpoint cases; retain one shared owner. |
| HG-010 | Retained-suffix swept validation | spatial step 0.5 inflated-map resolution, time step clamped 2-50 ms | PROVISIONAL | Adaptive segment checks fail closed for OCCUPIED and OUT_OF_MAP. Maximum step and map revision stability are not yet recorded in the certificate. | Attach map revision/generation and segment certificate to the committed bundle; test obstacle between legacy 50 ms samples. |
| HG-011 | Hardware Mid-360 visibility | hardware blocked unless explicit certificate flag is true | CERTIFIED | Fail-closed deployment gate exists. No real-flight visibility certificate exists yet. | Keep hardware blocked until mounting, FOV/blind zones, accumulated observations and motion envelope are certified. |
| HG-012 | CIRI overlap/seed tolerances (`NUMERICAL_TOLERANCE`) | overlap threshold plus hard-coded 0.01 m and 0.25 ratio; seed clearance `robot_r - 0.01 m` | PROVISIONAL | Different meanings currently share unnamed constants and can reject dense geometry inconsistently. | Name each quantity, record units/reason codes and test resolution scaling before changing values. |
| HG-013 | Backup corridor certificate (`SAFETY_INVARIANT`) | position penalty violation accepted up to 0.2 | PROVISIONAL | This differs from the 0.01 m EXP certificate and can reduce the margin on the trajectory intended to be safest. | Replace penalty-derived authorization with the same normalized continuous geometric certificate used by EXP; preserve certified seed fallback. |
| HG-014 | Typed estimator-health control gate (`SAFETY_INVARIANT`) | `TRACKING` plus navigation/covariance/observability/correction/propagation flags all true | PROVISIONAL | PX4 now consumes the typed `/lio/health` contract when present; a false accept could expose External Mode during an invalid estimator or public-frame transition, while a false reject causes a bounded hold/handover. Callback work is constant-time and expected below 1 ms at p99. | Repeat typed-health negative tests, loaded SITL and recorded-data health timing; make typed health mandatory in every launch profile, then remove the DiagnosticArray fallback TB-004. |

## Temporary-bypass register

| ID | Owner/date | Scope | Safety impact | Evidence | Removal condition | Status |
|---|---|---|---|---|---|---|
| TB-001 | planning/runtime; intro `1661386` | EXP jerk objective disabled; explicit `--tb001-exp-jerk-penalty 5e8` re-enable candidate; analytic V/A/J hard gate remains authoritative | False-reject and runtime-tail risk; no false accept if hard gate is complete | Disabled vs explicit enabled `external-mode-check` and `dataset-check` A/B; reports are uncertified | Objective-enabled A/B on structured SITL and dense snapshot replay has equal certificates and bounded p99 | TEMPORARY_BYPASS |
| TB-002 | planning; intro `30ca02c` | `traj_opt.exp_traj.penna_attract`: disabled `-1e7`, enabled reference `5e8`; no implicit runner switch yet | May reduce convergence margin but does not bypass the corridor certificate | Config-patched nominal/structured SITL A/B | Deterministic optimizer A/B proves a replacement improves feasibility without path drift | TEMPORARY_BYPASS |
| TB-003 | mapping/planning; intro `30ca02c` (`iris_iter_num`), `1661386` (`obs_skip_num`) | `super_planner.iris_iter_num/obs_skip_num`: current `(1,2)`, enabled reference `(2,1)`; no implicit runner switch yet | May reduce corridor volume or omit redundant surfaces; inflation/collision gates remain authoritative | Config-patched dense snapshot and SITL A/B | Dense snapshot benchmark proves constraint coverage, feasible rate and p99 across the iteration/skip matrix | TEMPORARY_BYPASS |
| TB-004 | PX4/runtime; intro `2026-08-25` | DiagnosticArray health remains a compatibility fallback only until the first valid typed `/lio/health` message is received | Legacy string health can control safety before typed health arrives; no intentional fail-open behavior, but the old parse path remains a contract risk | PX4 package build and existing health-gate tests; no closed-loop evidence yet | All launch profiles publish typed health before External Mode command exposure; negative epoch/flag tests pass; remove `onLioDiagnostics` safety fallback | TEMPORARY_BYPASS |
| TB-005 | runtime/mapping; intro `2026-08-25` | Legacy cloud plus corrected-odometry pairing remains a compatibility fallback until the first valid `/lio/mapping_observation` is accepted | Retains the old pairing/skew and duplicate-input risk for legacy-only launches; typed observation path is atomic and fail-closed | Runtime build, world-store tests, shutdown test; no end-to-end typed-stream SITL evidence yet | Every supported launch uses atomic `RegisteredScan`; remove legacy mapping subscriptions and `input_pairing.hpp` after dataset/SITL migration evidence | TEMPORARY_BYPASS |

Register anchors for the remaining rows are explicit, but TB-002 and TB-003
still have open switch debt rather than supported runner switches. TB-002 was
introduced by the `30ca02c` change from `traj_opt.exp_traj.penna_attract=5e8`
to `-1e7`; its disabled/enabled A/B must use that exact config key and values
in the generated planner file. TB-003 has split provenance: `iris_iter_num`
changed from 2 to 1 in `30ca02c`, while `obs_skip_num=2` entered with the
initial SUPER integration `1661386`; its A/B must restore
`(iris_iter_num=2, obs_skip_num=1)` versus the current `(1,2)` in the same
config. Until explicit switches exist, patch the generated config manually,
archive enabled/disabled artifacts, and keep both rows open. No implicit
environment switch exists for either row.

Bypass lifecycle is mandatory. Every `TEMPORARY_BYPASS` must name an owner,
the introducing commit, the exact configuration/code switch used to re-enable
the original behavior, a reproduction test with the bypass enabled, an A/B
test with it disabled, and an explicit removal condition. An open bypass is
never a certification or contract-freeze result. When the underlying defect is
fixed, first run the original behavior with the bypass disabled and archive the
result; only then change the register to `REMOVED` and delete the bypass. A
test or report must not silently convert an open bypass into a permanent
baseline.

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

### 2026-08-25 - 6 m/s reverse-overshoot evidence and command provenance

- Clean closed-loop artifact
  `.artifacts/runtime/external-mode-check-20260824T222754-258858` requested
  `long_three_pillars_speed` at 6 m/s and ended BLOCKED. The mission accepted
  only waypoint zero, entered `PAUSED_SAFETY_STOP`, and handed over to PX4 Hold
  without collision or PX4 failsafe. This is failure characterization, not a
  certified 6 m/s result and not authority to attempt or promote 8 m/s.
- The immediate rejection was the existing External Mode tracking-envelope
  guard: the UAV had advanced more than the strict 0.75 m reverse/geometric
  allowance beyond a forward-moving command anchor. The prior log printed only
  clamped forward and lateral error, which made this valid fail-closed decision
  appear unexplained. Upstream evidence includes repeated backup optimization
  failure, `new_ts_TT < committed_ts_TT`, emergency-bundle replacement and a
  backward command-anchor transition. Estimation remained tracking-valid and
  the observed minimum clearance was 4.435 m. No guard, jerk limit, backup
  requirement, deadline or controller threshold is changed here.
- `PositionCommand` now distinguishes the immutable committed-bundle generation
  and exact sampled trajectory time from the per-message `trajectory_id`.
  SUPER reports the exact established trajectory time used for PVA/yaw/role
  sampling (terminal samples remain pinned to the trajectory duration);
  runtime forwards generation and time with the final outgoing MAIN/BACKUP
  role. A rejected command now reports forward, reverse and lateral errors and
  limits, measured and commanded ENU position, command velocity, message ID,
  generation, role, trajectory time, status and source stamp from the same
  message. Synthetic no-command terminal messages use generation/time zero and
  never reuse stale provenance.
- The same artifact exposed an independent report merge skew: a planner-owned
  ingress diagnostic showed received/accepted 304 while the later world-model
  terminal diagnostic showed started/published/revision 305. World-model
  publication now emits received and accepted from the same mutex-coherent
  `ObservationAccounting::Snapshot` as started/published/failure and gauges.
  Report owner precedence therefore uses a complete producer-owned lifecycle
  event; no maximum, one-count tolerance or timestamp guess masks a genuinely
  inconsistent snapshot.
- Verification: the coherent Release overlay rebuilt all 19 product dependency
  packages after the ROS message change. Selected runtime, SUPER and PX4
  packages executed 64 tests with zero errors, failures or skips; the runtime
  report contract suite passed 86 tests. Focused reverse-error and message-field
  tests distinguish forward lag, reverse overshoot, sample ID, bundle generation
  and trajectory time. Gazebo reproduction with the new trace, sanitizer runs
  for the changed interface, and the full speed/map matrix remain open gates.

### 2026-08-25 - Speed-ladder target contract and 2 m/s control run

- Clean control artifact
  `.artifacts/runtime/external-mode-check-20260824T224256-268652` requested the
  same `long_three_pillars_speed` world at 2 m/s. It traversed almost the full
  140 m route without collision or PX4 failsafe, but remained BLOCKED: only
  waypoint zero was accepted and a final MAIN command at generation 419 crossed
  the forward tracking envelope by approximately 2 mm
  (`1.055 > 1.053 m`). Reverse and lateral checks passed. The guard correctly
  handed over to PX4 Hold. This narrow crossing is evidence for commit-anchor/
  execution-state timing analysis, not authority to raise the 0.75 m geometric
  allowance or tracking-lag term.
- The decisive command was sampled at TT 0.128 s from a generation whose
  execution-state age had reached 196 ms; its candidate began already close to
  the controller envelope. One stale execution-state rejection was recorded.
  Mapping was coherent at received/accepted/started/published/revision 1,133
  with zero replacement, failure or accounting violation. A prior generation
  406 emergency brake had already recovered to normal committed generations and
  is not treated as the immediate terminal trigger.
- The artifact also exposed an independent ladder-harness defect. Both speed
  profiles previously inherited a fixed 5 m/s measured-speed minimum; explicit
  caps below 5 could never pass, while 6 or 8 m/s requests could pass after
  reaching only 5 m/s. For the two dedicated 140 m speed-certification profiles
  only, an explicit `SPEED_CAP_MPS` is now both the setpoint maximum and the
  measured p95 target. The existing fixed 0.10 m/s tolerance remains unchanged:
  caps 2/6/8 therefore require at least 1.9/5.9/7.9 m/s. Default missions without
  an override retain their declared 5 m/s target. General obstacle missions do
  not reinterpret a velocity cap as a minimum.
- The speed checker now has a pure contract test proving the setpoint upper
  bound and measured attainment gates are independent, including exact
  tolerance, below-bound, missing-sample and excessive-setpoint cases. The
  runtime contract suite passed 88 tests. This harness correction does not
  retroactively change the artifact verdict: its measured p95 was 1.733 m/s,
  below the corrected 1.9 m/s target, and the safety stop remains independently
  terminal.

### 2026-08-25 - Atomic commit-splice and controller-reject provenance

- The 2 m/s terminal crossing could not be attributed safely from the previous
  trace because the solve input, candidate start, prior committed sample and
  controller's prior accepted command were owned by different threads and were
  not captured at their respective atomic boundaries. This batch adds
  observability only; it does not change A*, CIRI, MINCO, command sampling,
  authorization, the tracking envelope, freshness limits or any deadline.
- `CmdTraj` now records one `CommitDiagnostics` object under the same mutex as
  the immutable trajectory/certificate swap. It contains the new and prior
  generations, candidate start wall time and PVAJ/yaw state, the prior command
  evaluated at the candidate start wall time, and position/velocity/
  acceleration/jerk/yaw/yaw-rate splice residuals. Prior trajectory time is
  clipped only to its executable bounds; yaw residual is wrapped to
  `[-pi, pi]`. Failed candidate construction or authorization cannot mutate
  these diagnostics. A lock-owning lightweight generation accessor avoids
  copying trajectory polynomials before every solve; the full coherent snapshot
  is copied only after solve where endpoint, certificate and diagnostics are
  actually consumed.
- Runtime records the propagated execution position, world-frame velocity and
  source stamp used by SUPER. `state_age_at_solve_ms` is sampled with the ROS
  clock immediately before `PlanFromRest`/`ReplanOnce`; trace age is sampled
  separately after the solve. Commit diagnostics are emitted only when the
  committed generation changed during that solve cycle and diagnostics match
  the resulting generation, so a failed solve cannot inherit a prior commit's
  splice evidence. Diagnostic vectors are serialized with 17 significant
  digits and the report adapter parses them fail-closed.
- On tracking-envelope rejection, External Mode copies the exact evaluated
  odometry stamp/receive time, measured position/velocity and previous accepted
  `PositionCommand` under `trajectory_mutex_`, then logs outside the lock. The
  trace distinguishes header and receive age, reports current and previous
  generation/TT/PVAJ, an explicit previous-valid bit, saturated signed
  generation delta and P/V/A/J command deltas. A rejected command never becomes
  previous accepted history. These values remain diagnostics and do not affect
  the fail-closed decision.
- Verification on the final dirty source: SUPER, navigation runtime and PX4
  External Mode targets rebuilt successfully. `test_trajectory` passed 23/23,
  planner attribution tests passed 13/13, PositionCommand/reject provenance
  tests passed 6/6, tracking-envelope tests passed 5/5, and the runtime Python
  contract suite passed 101/101 including structured PVAJ vector round-trip and
  malformed vector rejection. The C++ trajectory suite includes the yaw-wrap
  boundary case. Gazebo reproduction, sanitizer coverage and full ROS
  callback interleaving tests remain evidence gates before diagnosing or
  changing the high-dynamics behavior.
- A candidate-start tracking-envelope margin is intentionally deferred. A
  truthful margin requires moving the pure envelope contract and its launch-
  owned geometric/lag profile to a product-owned package shared by runtime and
  External Mode. This batch does not duplicate the current `0.75/0.25` values,
  does not call the raw splice residual a safety margin, and does not use any
  new diagnostic in commit authorization.

### 2026-08-25 - Dual-clock execution-state lease at command ownership boundaries

- Coherent-overlay artifact
  `.artifacts/runtime/external-mode-check-20260824T231559-283215` separated the
  high-speed symptom from trajectory stitching. Generation 351 to 352 had zero
  P/V/A/J/yaw/yaw-rate splice residual, but propagated odometry and world
  revision then stopped advancing for about 0.7 s while command trajectory time
  advanced by 0.404 s. External Mode correctly rejected the resulting forward
  envelope error. This batch addresses stale execution ownership; it does not
  tune SUPER, the tracking envelope, deadlines, QoS or executor thread count.
- The runtime and controller now classify an execution-state lease with two
  independent clocks: the source header is aged against ROS time, while receipt
  is aged against steady time. Frozen or delayed `/clock` therefore cannot hide
  callback/DDS/executor starvation. The existing configured freshness limits
  are reused (currently 0.5 s in the product profiles); this batch does not
  certify or relax those values. Missing, future, source-stale and
  receive-stale states fail closed. Propagated odometry is also rejected before
  ownership when its P/V/quaternion is non-finite or degenerate.
- Immediately before command sampling, runtime snapshots the propagated lease,
  validates both ages, and serializes that decision with final solve exposure.
  A stale transition cancels the solve once, clears nominal/safety-suffix
  availability unconditionally and latches the logical goal. The explicit EMER
  terminal command remains repeatable until External Mode acknowledges the
  failure; later fresh odometry cannot resurrect the same goal. Only a new
  logical goal resets the latch. The transition mutex also covers the sample
  and publication decision, preventing a solve completion from racing a stale
  callback into a nominal PVA publication. It is the owner of executable
  availability/failure/suffix transitions for execution-lease failure,
  hot-failure fallback, watchdog timeout, new-goal reset, terminal mode status
  and final solve exposure. The lock order is `input -> transition`; no code
  holds transition while acquiring input or while running a planner solve.
  A monotonic goal epoch is captured by each solve and stored with executable
  command ownership. Final exposure revalidates that epoch, while non-hot goal
  changes and terminal status clear it atomically; an old solve or delayed
  command callback therefore cannot republish a prior goal's committed bundle.
  Epoch mismatch is discard-only: it cannot clear or fail a valid command that
  PASS_THROUGH deliberately transferred to the newer goal.
- External Mode owns a separate odometry lease. Staleness takes precedence over
  duplicate command IDs and the tracking envelope, and the same check runs
  before active hold or cached-command setpoint output. The deliberate
  `waitingForAirborne()` stationary warm-up remains the only active-mode
  exception because navigation has not started and no trajectory is executed.
  Failure provenance reports source and steady receive ages and hands over to
  PX4 Hold through the existing fail-closed path.
- Verification on the dirty source: navigation runtime and PX4 External Mode
  rebuilt successfully. All 13 focused runtime/PX4 test binaries passed. The
  dual-clock/failure-gate suite passed 9/9, including exact freshness boundaries,
  frozen ROS time, invalid/future inputs, one-shot latch/reset and both
  serialized solve-exposure/stale-invalidation orderings and atomic new-goal
  reset/old-command clearing, hot-failure/watchdog exposure rejection and
  cross-goal epoch rejection;
  controller contract
  tests passed 7/7 and tracking-envelope tests 5/5. `git diff --check` passed.
  Sanitizers, real executor starvation injection and the clean 2 m/s Gazebo
  3/3 gate remain open. No dataset, speed-ladder or full-map acceptance is
  claimed by this checkpoint.

### 2026-08-25 - SITL Gazebo bridge isolation screening checkpoint

- Artifact `.artifacts/runtime/external-mode-check-20260824T234947-296974`
  exposed a 0.7--0.8 s common wall-arrival blackout on `/clock`, raw IMU,
  LiDAR and Gazebo ground truth while independent PX4 XRCE odometry continued.
  FAST-LIO propagation then stopped at the existing 0.5 s steady receive-age
  contract. The shared `ros_gz parameter_bridge` process, rather than ROG or
  propagated-odometry computation, was therefore the first common ownership
  boundary requiring isolation.
- The canonical GZ-to-ROS manifest is split into a low-bandwidth control bridge
  (`/clock`, IMU and evaluation-only ground truth) and an independent
  high-bandwidth PointCloud bridge. Both remain pure GZ-to-ROS transports with
  unchanged topics, types and source stamps. `use_sim_time` is intentionally
  absent from these transport nodes: they do not own time-based product logic,
  and the control bridge must not subscribe to the same `/clock` it publishes.
  FAST-LIO, navigation runtime, controller and PX4 ingress retain simulation
  time. No QoS, queue, sensor rate, downsampling or freshness value changed.
- `/clock` is now a first-class monitor stream. Its active wall-arrival gaps
  are derived directly from consecutive recorded callbacks, independent of
  stale-timer scheduling; queued source stamps with small deltas cannot erase
  an outage. Startup and post-observation gaps are excluded by the existing
  active-window policy. The 0.5 s limit is the existing execution-state lease
  budget, reused as evidence for the same loss interval rather than introduced
  as a clock-specific tuned threshold. Both bridge processes are lifecycle-
  owned, and an early exit now fails readiness immediately. Per-process CPU,
  RSS and context-switch evidence remains open: the `ros2 run` registry PID is
  a launcher parent rather than the actual bridge worker, so this checkpoint
  does not publish misleading zero-CPU resource metrics.
- The intermediate split-only artifact
  `.artifacts/runtime/external-mode-check-20260825T000029-301769` isolated the
  failure further: LiDAR continued while clock/IMU/ground truth in the control
  bridge shared a 0.54 s blackout and correctly triggered `RECEIVE_STALE`.
  After removing the transport TimeSource, dirty screening artifact
  `.artifacts/runtime/external-mode-check-20260825T000631-304227` ran to sim
  time 118.4 s with 31,436 clock, 25,149 IMU and 6,287 ground-truth samples,
  zero active control-stream wall gap over 0.5 s and no propagated-odometry
  stale event. It later stopped for an independent, genuine SUPER
  cross-generation WT/TT discontinuity. This is positive one-run A/B screening
  evidence only; clean 2 m/s 3/3, LiDAR/mapping-loss gates, full Gazebo matrix
  and CPU/RSS certification remain open.
- Verification on the dirty harness: runtime contract tests passed 93/93,
  simulation asset tests passed 8/8 and `git diff --check` passed. The arrival-
  gap tests cover a 700 ms callback gap with continuous source timestamps and
  no intervening monitor tick, plus exclusion of a pre-TRACKING startup gap.

### 2026-08-25 - SUPER NO_NEED non-commit and monotonic command-time checkpoint

- Screening artifact
  `.artifacts/runtime/external-mode-check-20260825T000631-304227` later exposed
  a separate command-ownership defect after the Gazebo transport blackout was
  removed. Generation 397 replaced generation 396 with candidate start wall
  time 117.440 s, earlier than the already committed 118.248 s start. The
  controller consequently sampled the new bundle at trajectory time 0.976 s
  and observed a 1.222 m cross-generation position jump before correctly
  failing its tracking envelope. No freshness, envelope, jerk, deadline or
  dynamic-limit value is changed by this checkpoint.
- `generateExpTraj()` deliberately returns the historical EXP snapshot on
  `NO_NEED`; that snapshot is planner history, not a newly solved executable
  candidate. `ReplanOnce()` now returns `NO_NEED` before visualization, backup
  generation, authorization or history/command mutation. The runtime assigns
  this outcome a distinct retained-command validation disposition. A valid
  latest-world retained bundle keeps its availability, goal epoch, suffix,
  failure and finished state unchanged; an invalid retained suffix still fails
  closed. `SUCCESS` and `FINISH` can expose `CommandReady` only when the solve
  actually advanced the immutable committed generation.
- `CmdTraj::commitCandidate()` independently rejects a finite candidate whose
  start wall time is strictly earlier than the current bundle start while
  holding the command mutex and before diagnostics or any bundle field is
  modified. Equality is allowed. There is no epsilon, rebasing, truncation or
  compatibility bypass: rejection leaves position, yaw, role ownership,
  certificate, diagnostics and generation unchanged.
- Focused Release verification rebuilt `super_planner_vendor` and
  `navigation_runtime`; trajectory tests passed 26/26, SUPER configuration and
  braking tests passed 7/7, and planner FSM tests passed 16/16. Sanitizers and
  the clean 2 m/s Gazebo 3/3 reproduction remain open, followed by the mandatory
  Gazebo scene matrix and speed ladder. This checkpoint does not certify the
  previously unstable 6 m/s point.

### 2026-08-25 - Runtime closure status and evidence-provenance gate

- HEAD `3eb7dcc` is a near-P0 source-correctness checkpoint, not closure of the
  complete S-01--S-23 review and not flight certification. Focused source and
  test evidence closes S-01--S-12, S-14 and S-15. S-13, S-16--S-18, S-20 and
  S-21 remain partial. S-19 (Mid-360 hardware FOV/visibility certificate),
  S-22 (legacy `flag_whole_known_free_` terminology/API), and S-23 (legacy
  `min_stop_dis` / `v^2/(2a)` braking path) remain explicitly open. Continuous
  vertical certification, inter-generation yaw acceleration/jerk continuity,
  optimizer/backup latency distributions, full current-tree sanitizers,
  repeated SITL and hardware validation are required before structural refactor
  or flight authority.
- The EXP jerk objective disable, corridor-center attractor disable, and bounded
  CIRI iteration/obstacle-skip changes remain `TEMPORARY_BYPASS`. They are not
  acceptance mechanisms and must be reviewed again against dataset, dense-scene
  shadow planning and closed-loop evidence. No bypass may silently become the
  permanent contract merely because another test passes.
- Artifact `.artifacts/runtime/external-mode-check-20260825T002714-310960`
  exposed a provenance false-positive: its report read Git HEAD `3eb7dcc` at
  render time while the installed runtime executable predated that source.
  Runtime acceptance therefore requires an authoritative full Release build
  manifest. The build records a deterministic tracked/untracked source
  fingerprint plus SHA256 and resolved target identity for launch-critical
  product executables, libraries and harness scripts. The runner validates the
  same source and artifacts before starting any process and copies immutable
  evidence into the session. Missing/partial/stale/tampered builds fail closed;
  package-select and sanitizer builds cannot overwrite the authoritative
  manifest.
- `/clock` freshness remains a hard, direct consecutive-arrival contract, but
  its evidence owner moves into `StreamStats.update()` before the latest arrival
  is overwritten. Exact gap count, times, maximum and bracketing source stamps
  are persisted in `monitor.json`; the high-rate normal clock callbacks are no
  longer serialized one-by-one into `samples.jsonl`. Timer evidence remains for
  a terminal outage with no returning callback, and legacy artifacts retain the
  raw-sample fallback. This removes observer I/O perturbation without changing
  product QoS, sensor rate, queue, executor, freshness threshold or scheduling.
  A clock gap is still fail-closed observer-or-transport evidence, not proof of
  a specific bridge root cause.
- This harness checkpoint does not close S-13/S-16--S-23, dataset vegetation,
  CPU/RSS, A*/CIRI/MINCO shadow planning, clean 2 m/s 3/3, the full Gazebo map
  matrix, the 2--8 m/s speed ladder, or hardware flight. Those gates remain in
  that order before behavioral freeze and deep refactor.
- Final focused verification passed 122/122 runtime Python tests and
  `git diff --check`; the full Release product build completed 19 packages and
  the product test result contained 64 tests with zero failure/error/skip.
  The authoritative manifest expands from the explicitly named executables to
  the installed product runtime closure (workspace shared/static libraries,
  package executables and installed Python runtime/launch scripts), including
  ROSIDL/Livox typesupport and `libpx4_ros2_cpp.so`. Clock interval percentile history
  is intentionally disabled only for the high-rate clock stream to avoid an
  unbounded list and repeated sort; whole-run mean/source maximum, timestamp
  regressions and direct wall-gap ownership remain exact. Gap records are
  bounded to 1024, while total and overflow remain exact and any overflow makes
  the report fail closed.
- Active clock freshness uses the stale interval
  `[previous_arrival + budget, next_arrival)`: any intersection with the
  TRACKING observation window is a violation, including an outage whose
  threshold crossing preceded TRACKING. A delayed terminal stale timer records
  that deterministic threshold crossing rather than its dispatch time, so an
  executor delay cannot move an active failure past observation shutdown.
  FlightReview reads commit/dirty identity from the captured manifest source;
  it does not fall back to the repository state at render time.
- Preflight-to-exec TOCTOU and exact discovered-artifact-set equality remain
  P1 provenance work: this checkpoint proves the validated state immediately
  before launch, not an immutable post-exec identity. A shared build/runtime
  lock or `/proc/<pid>/exe` capture is required before strengthening that claim.
  Hash-preflight latency must be characterized across repeated runs. After this
  source checkpoint is committed, a new full Release build and manifest
  validation are mandatory because the commit itself changes the authoritative
  source fingerprint.

### 2026-08-25 - Coherent mapping lifecycle report correction and 2 m/s screening

- Clean screening artifact `.artifacts/runtime/external-mode-check-20260825T011353-349116`
  is valid causal evidence for the clean `8ec2811` build manifest, but it is not
  acceptance evidence: the PX4 checkout was dirty and the run failed closed on
  a multi-second common Gazebo/GZ-to-ROS ingress stall. `/clock`, IMU, ground
  truth and the separately bridged LiDAR all stalled for roughly 3.8--4.6 s,
  while PX4/XRCE odometry remained continuous. SUPER and External Mode therefore
  rejected stale execution state as designed; no freshness threshold or safety
  gate was relaxed.
- The report had falsely added an accepted-observation conservation reason by
  merging `received/accepted=248` from an older world-model PUBLISHED event
  with `discarded/published` counters from a later planner event (`251` was the
  coherent lifecycle snapshot). `_navigation_mapping_summary()` now selects one
  newest diagnostic event across both streams for all lifecycle counters, while
  retaining owner-specific world identity/telemetry. It never synthesizes a
  lifecycle snapshot by combining timestamps from different events.
- Compatibility validation now compares the legacy `dropped_cloud_count` with
  the aggregate replacement lifecycle (`replaced_waiting + replaced_ready`),
  falling back to the legacy aggregate when phase-specific fields are absent.
  Real replacements and stale-input counts remain report reasons; this change
  removes only the false merge/compatibility failure.
- Regression coverage includes the exact `248` world then `251` planner event
  ordering, legacy counter shape, and a truncated final-sample case that must
  fail conservatively. Runtime Python contract tests pass 111/111 and
  `git diff --check` passes. The report was rerendered for the
  screening artifact and now shows coherent `received=accepted=251`,
  `published=174`, `discarded_pending=76`, `discarded_ready=1`, with product
  accounting valid and zero violations. The artifact remains failed for its
  genuine transport/safety/mission reasons.
- Next diagnostic is deliberately bounded to one reproduction: capture a
  Gazebo-native clock/stats stream plus low-rate process scheduler/CPU/PSI data
  for Gazebo, bridges, FAST-LIO, runtime, monitor and PX4/XRCE. Do not split
  bridges again, tune the 0.5 s lease, add queues, or reinterpret this artifact
  as evidence against SUPER/controller behavior.

### 2026-08-25 - TB-001 EXP jerk objective A/B harness switch

- Owner: planning/runtime harness. Scope: Python runtime runner and report
  artifacts only; no product C++, default YAML, SUPER safety gate, freshness
  threshold, deadline, QoS, queue, or acceptance threshold is changed.
- Default behavior remains fail-closed with `traj_opt.exp_traj.penna_jerk`
  negative, so the EXP jerk objective stays disabled and the analytic V/A/J
  hard gate remains authoritative. The only opt-in path is the explicit
  `--tb001-exp-jerk-penalty` argument on an External Mode or dataset-check
  harness command; hidden environment activation is deliberately unsupported.
  The argument rejects missing, nonnumeric, nonpositive and nonfinite values.
- Provenance: the disabled EXP objective is present from the initial SUPER
  integration (`1661386`); the repository does not contain an authoritative
  historical positive EXP value. This checkpoint therefore registers `5e8`
  only as a provisional characterization candidate, not as recovered upstream
  behavior. Reproduction is
  `python3 tools/runtime/runner.py external-mode-check --map-profile
  long_three_pillars_speed --speed-cap-mps 2 --tb001-exp-jerk-penalty 5e8`,
  paired with the same command without that argument. Neither run can certify
  flight while TB-001 remains open. Dense replay A/B commands are explicitly:
  enabled: `python3 tools/runtime/runner.py dataset-check --dataset
  aist-mid360-drive --rate 1.0 --tb001-exp-jerk-penalty 5e8`; disabled:
  `python3 tools/runtime/runner.py dataset-check --dataset aist-mid360-drive
  --rate 1.0`. Neither run is certification while TB-001 remains open.
- When enabled, the generated session-local `super_planner.yaml` receives that
  finite positive EXP jerk penalty and `runtime.json` records bypass ID
  `TB-001`, the selected value, `harness-only experiment`, and
  `uncertified_experiment`. The report copies this metadata and adds a reason
  that prevents an experiment run from being treated as flight certification.
- Safety impact: this is an A/B observability mechanism for the existing
  temporary bypass, not a removal or certification of TB-001. It may expose
  optimizer feasibility/runtime effects, but it cannot authorize flight or
  close the bypass while the report marks the run uncertified.
- Removal condition: structured SITL and dense snapshot replay A/B show equal
  geometric certificates and bounded p99 with the objective enabled, then run
  the disabled default again and update TB-001 to `REMOVED` in a separate
  behavior change. Verification command for this harness change:
  `python3 -m unittest tools.runtime.tests.test_runtime_contract -v`.

### 2026-08-25 - Opt-in Gazebo-native ingress diagnostic

- The runner accepts `--gazebo-native-diagnostic` for an explicitly bounded
  screening session. It is off by default and is not started for the normal
  SITL matrix. The helper subscribes directly through `gz.transport13` to
  native `/world/<world>/stats` and `/world/<world>/clock`, records bounded
  arrival-gap/source/iteration evidence, and samples process-group descendants
  plus CPU/memory/I/O PSI at low rate.
- This observer is diagnostic-only: it adds no acceptance reason, safety gate,
  QoS, bridge, queue, executor, threshold or product scheduling change. A
  valid result requires both native streams and a completed summary; no result
  can certify a mission. The first run remains a single 2 m/s reproduction,
  followed by classification of Gazebo/host scheduling versus GZ transport or
  bridge delivery. Do not run the TB-001 A/B or speed matrix until that
  classification is complete.
- Verification before a live run: runtime/HTML tests, Python compilation, and
  `git diff --check`. Live native observer evidence is pending and must be
  archived with the session identity and authoritative Release manifest.

### 2026-08-25 - Native Gazebo ingress screening result (diagnostic only)

- Artifact: `.artifacts/runtime/external-mode-check-20260825T020321-400183`,
  `long_open_featured_speed`, requested speed cap 2 m/s, one run only. The
  session is a screening artifact, not a speed, controller, hardware, or
  certification result; the External Mode/PX4 checkout was dirty.
- Native Gazebo Transport observed 11,114 `/clock` samples and 540 `/stats`
  samples. Maximum wall-arrival gaps were 3.247 s and 3.269 s respectively;
  source time and iteration values were monotonic with no duplicate or
  regression events. Native real-time factor reached 0.0036. ROS IMU, ground
  truth, LiDAR and propagated odometry showed aligned multi-second gaps, and
  the product correctly failed closed on stale execution state. This narrows
  the cause toward Gazebo update/source scheduling or a common GZ boundary;
  it does not distinguish Gazebo publisher starvation from transport or host
  scheduling and does not authorize a code or threshold change.
- The native observer remains opt-in and diagnostic-only. Its process/PSI
  rows are raw evidence, not liveness certification; the post-run summary now
  also records process sample/role counts and PSI sample count. Because the
  Python binding returns no subscription status, the summary is `OK` only
  after both native callbacks deliver samples; otherwise it is `UNAVAILABLE`
  (observer fix after `f1388b5`). The
  artifact above predates that summary-field patch, so it is not evidence for
  those new fields.
- Next gate: rebuild the authoritative Release manifest after all committed
  source changes, then run one bounded native diagnostic with valid summary
  fields and correlate native Gazebo gaps against per-process scheduling/PSI.
  Do not run TB-001 A/B, the 6 m/s ladder, or tune freshness/jerk/queues until
  that causal branch is reviewed. Clean 3/3 speed and full scene/SITL gates
  remain open.

### 2026-08-25 - Native observer rerun after manifest refresh

- Artifact `.artifacts/runtime/external-mode-check-20260825T021353-412079`
  started after the authoritative Release manifest refresh. The native
  summary is complete: `status=OK`, 7,996 native clock samples, 389 stats
  samples, 40 process samples, 40 PSI samples, and role counts covering
  Gazebo, bridges, LIO, mapping, monitor, PX4 ingress and External Mode.
- The run remains `BLOCKED`, not certified: native clock/stats maximum arrival
  gaps were 3.861 s and 3.906 s, native RTF reached 0.0033, and the ROS report
  independently recorded stale clock/input/execution-state failures and early
  safety termination. This repeats the causal screening signal; it is not a
  2 m/s acceptance result and gives no authority to tune lease, jerk, queue or
  controller limits. External PX4 is still dirty, so this remains causal
  screening rather than clean closed-loop evidence.

### 2026-08-25 - Closed-loop report observability checkpoint

- Commit `ebfc99f` extends the report-only evidence surface with recorded PVA
  commands, trajectory generation/time/role, LIO/PX4/ground-truth stream
  coverage, planner/LIO timing and health tables, waypoint/vehicle replay
  state, and separate main/backup path rendering. It does not change planner,
  controller, acceptance or safety decisions.
- Path grouping is generation-aware: samples from different committed
  generations cannot be joined into a visually continuous trajectory. Legacy
  messages without generation are kept in one explicitly labelled
  waypoint/role trace instead of treating every per-command message ID as a
  separate one-point bundle. They remain legacy evidence and are not claimed
  to provide committed-generation continuity.
- The scenario/report view now records an observed application `SAFETY_STOP`
  separately from `ModeCompleted` and shows the requested PX4 Hold handover
  interval. This is report-only observability: it does not change the
  controller envelope, handover decision, mission outcome, or acceptance
  gates. Verification: runtime/HTML contract tests and `git diff --check`.
- Verification: Release build 19 packages PASS; release check 64/64 PASS;
  runtime/HTML contract tests 150/150 PASS; `git diff --check` PASS. This is
  static/report evidence only; browser rendering, clean PX4 provenance,
  repeated SITL, speed ladder, hardware FOV and sanitizer certification remain
  open.

### 2026-08-25 - Planner stage timing instrumentation (observability only)

- The latest open-map benchmark identifies EXP L-BFGS as the dominant planner
  hot-path cost: `exp_opt_ms` p95 about 117 ms versus total solve p95 about
  133 ms and a 203 ms maximum against the 180 ms solve budget. Mapping callback
  p95 is about 24 ms and backup optimization p95 about 17 ms. These values are
  evidence from one blocked screening artifact, not approval to tune a gate.
- Runtime decision traces now publish `exp_frontend_us`, `exp_opt_us`,
  `backup_frontend_us`, and `backup_opt_us` with explicit microsecond units.
  The report aggregates them as planner timing distributions. No optimizer
  penalty, deadline, cancellation policy, queue, safety threshold, or speed
  limit changed. This closes a P1 evidence gap so the next benchmark can
  distinguish EXP cost from mapping and backup cost.
- Verification: runtime Python contract suite and planner-trace tests; a fresh
  Release rebuild is required before using these fields in a new SITL artifact.
  Algorithmic optimization remains deferred until repeated open-map/dense
  measurements and SUPER-parity comparison are available. TB-001..003 remain
  temporary bypasses and must not be enabled implicitly.

### 2026-08-25 - Steady-clock optimizer cancellation

- `AbsoluteDeadline` retains the existing ROS/simulation-time checks used for
  trajectory timestamps, and now also owns a monotonic steady-clock deadline.
  EXP and backup L-BFGS progress callbacks stop on cancellation or steady-budget
  expiry. A cancelled optimizer returns failure and clears its candidate; no
  partial trajectory can reach `CmdTraj` commit or replace the certified bundle.
- This addresses the measured failure mode where simulation time freezes while
  an optimizer continues running, and bounds load-dependent tail work without
  increasing the solve deadline or weakening any dynamic/world certificate.
  It is not a speed or success-rate claim; the next benchmark must measure
  timeout counts, cancellation stage, p95/p99 latency, and retained-command
  behavior under the same safety contracts.
- Verification: Release build of `super_planner_vendor` and `navigation_runtime`,
  package tests (10/10), and the `AbsoluteDeadline` steady-clock unit assertion.
  ASan/UBSan/TSan and repeated SITL remain open. No TB-001/TB-002/TB-003
  bypass was enabled by this change.

### 2026-08-25 - Lazy A* node ownership

- A* retains the same indexed `unique_ptr<GridNode>` table and hash mapping, but
  now constructs a node only when a search first touches that voxel. The prior
  eager construction materialized roughly 25 million nodes for the configured
  `[500,500,100]` map and dominated startup RSS. Lazy construction does not
  change neighbor expansion, scores, father pointers, rounds, or safety flags;
  it only removes untouched heap objects.
- This is a memory/initialization optimization, not a map-resolution or timeout
  change. Peak touched-node count, search latency, RSS, and failed-search
  behavior must be measured on recorded data and repeated SITL before claiming
  acceptance. No temporary bypass was enabled.
- Verification: Release package build and existing SUPER tests; ASan/UBSan/TSan
  plus repeated SITL and RSS measurements remain open.

### 2026-08-25 - Explicit solve failure provenance and goal contract

- Planner timeout/cancellation now use distinct return codes from genuine backup
  generation failure. Safety remains fail-closed and committed CmdTraj is not
  changed; this only prevents timeout diagnostics from being mislabeled as a
  backup-geometry failure.
- The unused `ExpTraj::flag_whole_known_free_` API and the unused legacy
  `min_stop_dis` braking calculation were removed. No caller consumed either
  value, so no planning decision changed.
- Planner endpoint connectivity, planner NO_NEED/goal shortcuts, and runtime
  mission completion now share the inclusive
  `navigation_world_model::kGoalCompletionToleranceM = 0.20 m`. This is an
  intentional behavior correction from the former resolution-scaled planner
  heuristics, not a claim that 0.20 m is certified; scale sensitivity and
  end-to-end mission evidence remain required.
- Verification required: current SUPER/runtime build and focused tests,
  runtime contract tests, and `git diff --check`. No bypass was enabled, but the
  goal-contract correction must be revalidated on repeated open-map SITL before
  acceptance; sanitizer, hardware and full goal-distribution evidence remain
  open.

### 2026-08-25 - Fail-closed optimizer warm-start validation

- `BackupTrajOpt::optimize` now rejects empty, mismatched, non-finite, or
  non-positive warm-start durations/points before any `.back()` or MINCO setup.
  `ExpTrajOpt::optimize` likewise rejects empty, non-finite, negative, or
  non-monotonic guide timestamps. These are input-integrity checks only; valid
  trajectories and safety gates are unchanged.
- This closes an obvious undefined-behavior path that could otherwise surface
  as a planner crash during a failed/reordered replan. Verification is the
  current SUPER build and focused tests; sanitizer and repeated SITL evidence
  remain open.

### 2026-08-25 - Canonical ROS runtime and stale-install prevention

- The supported build/test/replay environment is the system interpreter
  `/usr/bin/python3` after sourcing `/opt/ros/jazzy/setup.bash` and the
  workspace `install/setup.bash`. Make targets explicitly unset `VIRTUAL_ENV`
  and `PYTHONHOME`; direct use of another interpreter fails closed. A Python
  virtualenv remains available for unrelated tooling only and must not launch
  ROS, build, dataset, or SITL workflows.
- Release `build/`, `install/`, and `log/` are the single canonical product
  cache. Release package-select builds invalidate the authoritative manifest;
  only a full Release build recreates it. The manifest now requires exact
  equality between its recorded runtime artifact set and the currently
  discovered workspace binaries/libraries/scripts.
- A repository-shared build/runtime lock prevents a build or test from
  replacing the canonical install while a dataset/SITL process is running (and
  prevents a runtime from starting during a build). `make clean` takes the same
  lock and preserves both lock inodes while removing generated sessions, so a
  concurrent operation cannot recreate a lock path and bypass serialization.
  This is provenance and lifecycle protection only; it does not change
  planner/controller gates, queues, QoS, thresholds, or bypasses.
- Verification: canonical Python/manifest/lock contract tests and
  `git diff --check`. A fresh full Release build is mandatory after this
  change before any runtime artifact may be used; the current HTML/report
  working-tree edits remain separate and are not overwritten. Sanitizer,
  repeated SITL, speed, dense vegetation, hardware FOV, and bypass-removal
  evidence remain open.

### 2026-08-25 - Goal semantic split and direct near-goal segment check

- Planner completion, planner endpoint connection, and near-goal A* shortcut
  now have separate product-owned policy names while retaining the current
  provisional 0.20 m values. This is a semantic ownership split, not threshold
  tuning.
- The near-goal shortcut and terminal endpoint snap now require a direct
  inflated-world segment check. The main exploratory path continues to use
  `UnknownPolicy::kAllowUnknown`; the WorldModel still fails closed for
  occupied or out-of-map segments. Backup certification remains independent.
- No bypass, optimizer weight, CIRI parameter, or acceptance threshold changed.
  Removal condition: keep the split until separate completion/connection/
  shortcut distributions justify distinct values and contracts.
- Verification: SUPER build and `test_trajectory` 30/30; navigation runtime
  build; direct runtime FSM/clock/mission tests 20/20. Full sanitizer,
  repeated SITL, goal-distribution and hardware evidence remain open.

### 2026-08-25 - EXP optimizer retry and budget observability

- `ExpTrajOpt` now records per-solve L-BFGS attempt count, bounded-feasibility
  retry count and violation mask, return codes, cancellation, normalized
  dynamic-violation values, retry stop reason, and remaining steady-time
  budget. The runtime planner trace emits these fields for correlation with
  solve/module timing; the values are diagnostic-only and do not alter
  candidate selection, retry limits, weights, deadlines, or safety gates.
- The reset-before-solve boundary prevents a solve that bypasses EXP from
  inheriting a previous generation's optimizer metrics. This is intended to
  identify whether EXP/retries dominate the observed solve budget before any
  optimizer tuning or bypass removal is attempted.
- No temporary bypass was enabled or re-enabled in this change. The EXP jerk,
  corridor-attractor, and CIRI/obstacle-skip bypasses remain explicitly open
  debt with their existing removal conditions; they must be restored only in a
  separately recorded, measured A/B batch.
- Verification: Release package build, `test_trajectory` 30/30, planner-FSM
  17/17, observation-accounting 16/16, and `git diff --check`. Runtime timing
  distributions, sanitizer overlays, repeated SITL, and full bypass-removal
  certification remain open.

### 2026-08-25 - A* guide-time monotonicity and legacy speed-map resolution

- The legacy `speed` profile now resolves to its existing `open.sdf` asset,
  matching the runner's mission/collision alias contract; it no longer fails
  before simulator startup by trying to read a nonexistent `speed.sdf`.
- A* returns points in start-to-goal order while the time allocator reports
  remaining-time coordinates. The planner now converts those values to
  monotonic elapsed guide times before EXP/MINCO validation. This fixes an
  obvious clear-route failure where multi-point speed routes were rejected as
  non-monotonic. The allocator also clamps only round-off-level negative
  quadratic discriminants at its switching boundary; materially invalid roots
  remain rejected. No speed limit, jerk limit, deadline, or safety gate
  changed.
- Verification: speed alias regression test; full Release build/check; one
  post-fix `MAP_PROFILE=speed SPEED_CAP_MPS=2` screening run. The run now
  reaches all five waypoints and records the requested speed, but remains
  `FAIL` because actual post-first-waypoint cross-track p95 is about 0.98 m;
  this is retained as controller/turn-tracking evidence, not hidden by a
  relaxed threshold. The report excludes only the initial takeoff/reposition
  before waypoint 0 is accepted, which is not mission-polyline tracking.
  Repeated speed ladders, dense scenes, sanitizers, and certification remain
  open.

### 2026-08-25 - Point-aligned guide timing and one resolved mission contract

- The earlier remaining-time conversion still associated `dis[i]` with the
  previous A* point: the terminal point retained the last segment distance and
  the first nonduplicate point could receive zero elapsed time. EXP/MINCO could
  therefore receive a badly conditioned initial time vector and commit an
  abnormally long local bundle. Guide timing now uses cumulative travelled
  distance from the current guide endpoint, removes consecutive duplicate
  points, and requires finite strictly increasing elapsed time for every
  retained point. The endpoint time must equal the complete point-mass profile.
- A session-owned `resolved_mission.yaml` is now the single mission dynamics
  input consumed by SUPER, the PX4 External Mode node, the scenario and the
  report. An explicit speed-cap request changes only that copied contract; the
  source mission is not modified. This closes the previous split where the
  generated planner boundary could use the requested cap while the controller
  loaded the original mission. SUPER also receives the exact resolved mission
  path through its node parameters.
- Planner diagnostics now expose guide length/duration and EXP initial/final
  duration so an abnormal allocation is visible in the runtime artifact. These
  fields are diagnostic-only. No freshness, jerk, acceleration, envelope,
  deadline, QoS, queue, fallback, or bypass contract changed.
- Focused verification: runtime Python contract suite 135/135; SUPER trajectory
  tests 33/33 including a nonuniform route at 2/5/6/10 m/s; mission-dynamics
  tests 2/2; planner-FSM tests 17/17. A full authoritative Release rebuild and
  one bounded low-speed native-Gazebo diagnostic screening are required before
  using runtime evidence. Repeated 3/3 SITL, the 10 m/s obstacle-avoidance
  ladder, representative dataset timing, sanitizer overlays and hardware
  certification remain open and may not be replaced by that screening run.

### 2026-08-25 - Bounded dataset shadow-planning goal

- `make dataset-check` now publishes one synthetic STOP goal 5 m ahead of a
  fresh propagated state after two seconds of recorded source time. SUPER uses
  the normal goal subscriber, immutable WorldSnapshot, A*/corridor/optimizer,
  commit and command publication paths. The helper observes READY commands for
  two additional source seconds, then publishes an explicit
  `FAILED/OPERATOR_TAKEOVER` status to remove the synthetic goal. That status
  is harness teardown, not a mission failure or completion claim; recorded
  odometry never executes the generated command, so this is planner/runtime
  evidence only and cannot certify PX4 tracking or obstacle avoidance.
- A shadow-planning failure cancels the synthetic goal immediately but does
  not truncate rosbag replay. The report remains FAIL for the planner result
  while raw-ingress, mapping conservation, and timing evidence continue to the
  end of the source bag; this prevents a fast planner rejection from being
  misreported as dataset transport loss or from contaminating the remainder
  of the run with repeated terminal commands.
- The teardown publisher uses the same reliable/transient-local durability
  contract requested by the runtime status subscriber and waits for discovery
  before sending the terminal identity. A volatile publisher is incompatible
  with that subscription and would leave the synthetic goal latched, causing
  repeated EMER publications for the rest of the replay.
- Owner: runtime validation harness and dataset report. Scope: dataset replay
  only. Default goal distance is 5 m; mapping-only characterization remains
  available explicitly with `DATASET_SHADOW_GOAL_M=0`. No planner threshold,
  optimizer weight, deadline, QoS, queue, fallback or safety gate is changed.
  Report evidence fails closed when the goal is missing, no committed READY
  command appears, EMER is emitted, or planner timing/runtime trace is absent.
- The preceding AIST 1x/2x artifacts exposed a separate invalid benchmark
  state: only 336/2,756 ROG updates had nonzero update time. The effective ROG
  virtual ground was -0.4 m and the dataset descended below it; 2,330 updates
  returned as below-ground and 90 as slide-only. Those artifacts remain useful
  transport/timing evidence but are not full-route ROG correctness evidence.
  The synthetic goal is intentionally injected before that boundary and does
  not waive the need for a dataset-specific vertical-frame/bounds contract.
- Verification command: `/usr/bin/python3 -m unittest discover -s
  tools/runtime/tests -p 'test_*.py' -v`, full authoritative Release rebuild,
  then `make dataset-check DATASET=aist-mid360-drive RATE=2.0`. Compare against
  `DATASET_SHADOW_GOAL_M=0` using exact raw/mapping counts and per-stage planner
  timing. Repeated dense snapshot planning, full-route vertical validity,
  sanitizer and closed-loop SITL gates remain open.
- Screening artifact `dataset-20260825T092443-180098` completed the full 2x
  bag after the fail-closed planner result: raw IMU 55,435/55,435, LiDAR
  2,772/2,772, and mapping received/accepted/started/published/revision all
  2,756 with no mapping lifecycle failure or discard. Seven PlanFromRest
  decisions consumed 0.357 ms total (0.171 ms maximum) and never entered EXP
  or backup optimization. Every attempt classified the execution state as
  inflated OCCUPIED and reported the corridor entirely inside inflated
  occupancy; no READY generation was committed.
- The artifact confirms a dataset/profile contract mismatch rather than a
  compute timeout: the goal was injected at z=0.063 m while the effective ROG
  virtual ground was -0.4 m and four 0.2 m inflation steps plus the occupied
  cell place the inflated virtual-floor envelope near +0.6 m. The same run
  retained the full-route ROG anomaly (2,330 below-ground early returns and 90
  slide-only resets). This is not authority to lower a safety plane from one
  run; the dataset needs an explicit frame/bounds contract before shadow
  planning can benchmark the optimizer. The result is FAIL evidence, not a
  planning acceptance or flight certificate.

### 2026-08-25 - Arbitrary-origin vertical map contract and truthful update outcomes

- Owner: ROG-Map vendor boundary, immutable world-model adapter,
  `navigation_runtime` mapping lifecycle, SUPER corridor geometry and runtime
  report. Scope: product planning frame `lio_odom` plus the generic upstream
  ROG configuration surface.
- `lio_odom.z=0` is an estimator-origin convention, not surveyed terrain or
  takeoff-relative clearance. The product config therefore sets
  `rog_map/virtual_ground_ceiling_en: false`; runtime rejects startup if an
  absolute virtual plane is re-enabled in `lio_odom`. The upstream-compatible
  default remains `true` for explicitly datum-aligned users. This is a frame
  semantic correction, not a relaxed altitude threshold or temporary bypass.
- With planes disabled, base/inflated classification, ray clipping and export
  no longer synthesize occupied absolute-Z planes. Observed/inflated occupancy
  remains authoritative and points outside the finite sliding map remain
  `OUT_OF_MAP`. UNKNOWN inside the local window continues to follow the
  planner's explicit unknown policy; disabling the synthetic plane alone is
  not a hardware ground-visibility certificate. Corridor bounds are refreshed
  from every pinned immutable snapshot, so a vertical map slide cannot retain
  the initial Z window as a frozen floor/ceiling.
- `ROGMap::updateMap` now returns a typed outcome. The first valid cloud after
  a map slide is processed instead of discarded. Runtime requires callback
  ownership disabled and `batch_update_size=1`, publishes a new immutable
  revision only for a world-advancing outcome, preserves the revision's source
  stamp across rejected attempts, and fail-stops any unexpected non-advancing
  outcome. Diagnostics count every outcome; report validation requires the
  full contract, exact outcome/lifecycle conservation, and revision equality.
  Missing fields fail as stale-binary evidence instead of silently accepting
  an old install.
- Safety impact: removes a false occupied floor that rejected valid descent
  and caused 2,330 no-op dataset updates, while retaining finite local-map and
  measured-obstacle failure semantics. Removal/reversal condition: only after
  the planning frame is explicitly tied to a certified physical vertical
  datum and representative dataset, repeated SITL and hardware visibility
  evidence justify physical planes. No optimizer, deadline, QoS, queue,
  controller, collision-envelope or UNKNOWN-policy value changed.
- Focused verification: Release builds of `rog_map_vendor`,
  `navigation_world_model`, `super_planner_vendor` and `navigation_runtime`;
  package tests with ROG enabled/disabled plane, nonempty vertical slide,
  immutable export/adapter and real-node shutdown coverage; runtime Python
  contract tests including stale/malformed outcome evidence. Final gate is a
  full authoritative Release rebuild followed by
  `make dataset-check DATASET=aist-mid360-drive RATE=2.0` with the bounded
  shadow goal. It must show exact raw counts, all 2,756 mapping observations
  as advancing `UPDATED`, no plane rejection/slide-only outcome, a committed
  READY command and planner timing. One replay remains screening only; dense
  data, sanitizer, closed-loop SITL and hardware certification remain open.
- Screening artifact `dataset-20260825T100311-203927` was produced after the
  full 19-package authoritative Release rebuild and rerendered PASS with no
  report reasons. Raw IMU was 55,435/55,435, LiDAR 2,772/2,772, and mapping
  received/accepted/started/published/revision were all 2,756. All 2,756 map
  outcomes were `UPDATED`; accumulated, slide-only, empty, callback-owned,
  below-ground and above-ceiling were zero. The bounded shadow goal produced
  51 READY commands across five committed generations, zero EMER commands and
  ten complete planner trace records; maximum observed solve latency was
  1.278 ms. The report initially emitted a false failure because its shadow
  gate required the unused aggregate `planning_total_us` field even though the
  runtime supplied exact per-cycle `planning_latency_ms` trace values. The
  gate now accepts finite runtime trace timing while continuing to fail when
  neither aggregate nor trace timing exists, with a focused regression.
- This corrected full-map run achieved 1.9306x requested replay rate over
  143.565 s. Mapping callback mean/p95/p99/max were
  5.742/10.308/13.509/25.287 ms and immutable export mean/p95/p99/max were
  4.103/6.176/7.582/14.247 ms. These values are higher at the center of the
  distribution than the earlier invalid 2x baseline because that baseline
  skipped most ROG probability/inflation work below its synthetic floor; it
  is not a valid optimization comparison. The new maximum remains below the
  50 ms wall-time LiDAR period at 2x in this one run, with zero loss or
  replacement. This is screening evidence only, not a new latency threshold
  or repeatability/flight certificate.
- Final adversarial review also closed two evidence/efficiency defects. The
  report now preserves the published world's `observation_stamp_ns` separately
  from `last_update_attempt_stamp_ns` and fails closed if a nonempty published
  history lacks that source stamp or orders it after the last attempt. ROG's
  first observation outside an empty/local window no longer invokes
  `slideAllMap` twice at the same pose; a nonempty vertical-slide regression
  requires exactly one slide application. Neither correction changes map
  thresholds, occupancy policy, timing budgets or planner behavior.

### 2026-08-25 - Typed observation, health and localization-epoch migration

- Owner: FAST-LIO ROS boundary, navigation runtime and PX4 External Mode.
  Scope: add `RegisteredScan`, `EstimatorHealth` and `NavigationCommand` message
  contracts; publish `/lio/mapping_observation` and `/lio/health`; carry
  `localization_epoch` through immutable world identity and commit authorization.
- The new safety identity is `{localization_epoch, world_generation, revision,
  observation_stamp_ns}`. Epoch zero is invalid for publication. A higher epoch
  causes the sole mapping worker to reset its mutable ROG owner, publish an
  empty new-epoch snapshot, and then publish revisions from the new map. Old
  snapshots remain immutable but cannot authorize a commit because identity
  comparison includes epoch.
- `RegisteredScan` is atomic: outer and inner cloud frame/timestamp must match,
  the corrected pose is carried in the same message, and `scan_sequence` must
  increase within one localization epoch. Runtime accepts typed observations as
  the product path; old cloud
  plus corrected-odometry pairing is only the explicitly registered TB-005
  fallback and is disabled after the first valid typed observation.
- `EstimatorHealth` is the typed control-plane gate. PX4 requires TRACKING,
  navigation validity, covariance validity, observability validity, fresh
  correction and valid propagated state. DiagnosticArray remains only under
  TB-004 until all launch profiles are migrated. No numeric threshold, map
  resolution, planner deadline, unknown policy or dynamic limit changed.
- False-accept risk: stale or cross-epoch estimator state could keep External
  Mode active or authorize a stale world/command. False-reject risk: a missing
  typed message or transient invalid flag causes hold/handover. Runtime cost is
  bounded constant-time message construction/validation; p50/p95/p99 latency
  evidence and closed-loop epoch-reset evidence are still open.
- Focused verification completed: navigation interface typed-contract tests
  3/3; FAST-LIO ROS build and tests 12/12; SUPER trajectory tests 33/33;
  PX4 External Mode tests 3/3 targets (43 tests); world snapshot store 7/7;
  runtime epoch/store, shutdown and mapping fail-stop checks 3/3. The first
  full runtime test pass was blocked by an outdated three-field test identity
  initializer, which deadlocked the expected commit callback; the initializer
  was corrected and the isolated concurrent test passed in 0.25 s.
- Removal/review condition: complete typed command provenance integration,
  migrate all launch/configuration profiles, run repeated SITL plus dataset
  shadow evidence, and then remove TB-004/TB-005 only after their exact
  negative tests and verification commands pass.

### 2026-08-25 - Batch 1B localization reset barrier and epoch-ordered ingress

- Owner: navigation runtime mapping worker and SUPER runtime boundary. Scope:
  reject stale `RegisteredScan` epochs before worker admission, consume typed
  estimator-health epoch transitions, make the mapping worker drain READY and
  finish IN_FLIGHT work before a reset, reset the timestamp order key per
  localization epoch, and keep planning/legacy command exposure fail-closed
  until the first new-epoch world snapshot is published.
- A higher epoch now invalidates pending legacy cloud/pose pairing, resets the
  per-epoch scan sequence, and marks the planner epoch unavailable. A solve
  that races with the transition is discarded before command exposure when
  either its epoch or the new-epoch snapshot readiness no longer matches.
  `body_frame_id` is an explicit `base_link` contract in the supported runtime
  configuration.
- False-accept risk addressed: an old observation or a pre-reset solve can no
  longer enter the new worker epoch or expose a command after the reset gate.
  False-reject risk: an observation arriving during the bounded reset barrier
  is discarded and must be retransmitted by the estimator; no planner or
  safety threshold is relaxed.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-select navigation_runtime --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && ctest --test-dir build/navigation_runtime --output-on-failure`.
  The new worker reset regression and the full runtime package suite must
  pass before this checkpoint is treated as stable.
- Remaining condition: typed command provenance, runtime health epoch
  subscription/monotonic validation, observed-free backup certification and
  closed-loop reset evidence remain open; TB-004/TB-005 are not removable.

### 2026-08-25 - Product-owned NavigationCommand command boundary

- Owner: navigation interfaces, navigation runtime and PX4 External Mode.
  Scope: replace the runtime/PX4 `mars_quadrotor_msgs/PositionCommand` boundary
  with the product-owned `navigation_interfaces/NavigationCommand` while
  preserving PVA sampling, main/backup behavior, terminal fail-closed output
  and the existing topic name `/navigation/super_command`.
- `NavigationCommand` carries localization epoch, goal epoch and mission
  identity, world generation/revision/observation stamp, committed bundle
  generation, per-sample ID, propagated-state source stamp and validity window.
  PX4 rejects malformed identities/status-role combinations, health-epoch
  mismatches and regressed world/state provenance before command acceptance.
  The shared contract lives in `navigation_command_contract.hpp`; runtime
  validation tools consume the typed message directly. Legacy
  `PositionCommand` is no longer a product dependency of runtime/PX4.
- False-accept risk addressed: a command from an older localization/world
  identity or an untyped legacy producer can no longer become the authoritative
  PX4 command on this boundary. False-reject risk: malformed/missing typed
  provenance causes the existing hold/handover path. No planner, mapping,
  tracking-envelope, freshness or dynamic-limit numeric value changed; the
  existing command header/receive freshness gate remains authoritative while
  `valid_until` is currently syntax-validated for forward compatibility.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-select navigation_interfaces navigation_runtime px4_navigation_external_mode --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && ctest --test-dir build/navigation_interfaces --output-on-failure && ctest --test-dir build/navigation_runtime --output-on-failure && ctest --test-dir build/px4_navigation_external_mode --output-on-failure && python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'`.
  Current focused evidence is interface 1/1, runtime 10/10, PX4 3/3 and
  Python runtime 168/168.
- Removal/review condition: migrate all remaining report/artifact schemas
  from compatibility `trajectory_*` names to explicit command provenance,
  make typed estimator health mandatory in every launch profile, certify the
  validity-window policy on repeated SITL/recorded data, and remove TB-004/TB-005
  only after their exact negative tests and migration evidence pass.

### 2026-08-25 - Product-owned tree and shared utility cutover

- Owner: navigation contracts/common utilities, estimator ROS boundary,
  runtime and PX4 adapters. Scope: rename the active contract package to
  `navigation_contracts`, move shared frame/time helpers to
  `navigation_common`, remove the unused legacy command package, and rename
  the runtime node, configuration namespace, command topic and diagnostics
  source to product-owned names.
- The active product path has one contract package and one command type. ROS
  timestamp conversion now validates malformed fields and range in one helper;
  ENU/NED and FLU/FRD matrices are also defined once and consumed by both PX4
  bridges and External Mode. PX4's external ABI and pinned upstream backend
  namespaces are unchanged at their adapters.
- Safety impact: no numeric gate, map policy, planner deadline, tracking
  envelope or dynamic limit changed. The internal ROS package/topic/config
  names changed atomically, so stale generated overlays are rejected by the
  runner rather than silently mixed with the active workspace.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to fast_lio_ros navigation_runtime px4_navigation_external_mode px4_odometry_bridge --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && colcon test --packages-select navigation_common navigation_contracts fast_lio_ros px4_odometry_bridge --event-handlers console_start_end+ --return-code-on-test-failure`.
  The cutover build completed; the four selected package test runs completed
  without package failures. The broader workspace test-result directory still
  contains unrelated historical upstream results and is not used as evidence.
- Removal/review condition: extract mutable mapping ownership into a
  product-owned `navigation_mapping` package, then place the current planner
  behind a product-owned backend adapter. Do not add compatibility aliases or
  a second product implementation during that migration.

### 2026-08-25 - Mapping lifecycle primitive extraction and common time boundary

- Owner: `navigation_mapping`, `navigation_common`, runtime and PX4 adapters.
  Scope: move the bounded latest-only mapping worker, exact observation
  accounting, and immutable world-snapshot store out of the runtime package;
  centralize ROS time and PX4
  microsecond conversion in `navigation_common`; use the ROS helper at the
  External Mode command/odometry boundary.
- Safety impact: no planner, mapping, freshness, tracking, or dynamic-limit
  threshold changed. The extraction preserves the existing single-owner,
  reset-barrier, monotonic-order, shutdown, and fail-stop behavior. Malformed
  ROS time and conversion overflow remain rejected.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to navigation_runtime --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && colcon test --packages-select navigation_mapping navigation_runtime --event-handlers console_start_end+ --return-code-on-test-failure && colcon test-result --test-result-base build/navigation_mapping --verbose && colcon test-result --test-result-base build/navigation_runtime --verbose`.
  Result: navigation_mapping 57/57 and navigation_runtime 101/101; common
  time/frame tests 5/5; PX4 odometry bridge 68/68; External Mode 78/78.
- Remaining condition: mutable backend map ownership and world-snapshot
  construction/publication,
  observed-free evidence, and planner commit/execution ownership remain in the
  runtime/backend boundary. Continue with a product-owned backend adapter only
  after preserving these invariants and adding adversarial evidence.

### 2026-08-25 - Planner backend package and configuration vocabulary cutover

- Owner: planning backend boundary, runtime, build provenance and report tools.
  Scope: replace the product-facing planner package, artifact library path,
  planner configuration root, adapter class/file names, and parity-tool name
  with neutral product vocabulary. The imported implementation remains behind
  `navigation_planning_backend/planner.hpp`; its upstream namespaces and
  provenance files are not reinterpreted as product APIs.
- Safety impact: no trajectory, freshness, collision, backup, or dynamic-limit
  value changed. The configuration parser and generated runtime config now use
  the same `planner` root, preventing a split configuration vocabulary.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to navigation_runtime --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && colcon test --packages-select navigation_planning_backend navigation_runtime --event-handlers console_start_end+ --return-code-on-test-failure && colcon test-result --test-result-base build/navigation_planning_backend --verbose && colcon test-result --test-result-base build/navigation_runtime --verbose`.
  Result: planning backend 42/42 and runtime 108/108.
- Remaining condition: the vendored implementation still contains its
  upstream namespaces and license/provenance text. Do not rewrite those
  internals without a separate parity-preserving extraction and audit.

### 2026-08-25 - Pure planning contract and execution commit boundary

- Owner: `navigation_planning` and `navigation_execution`; runtime remains the
  transitional integration owner. Scope: add C++20-only `PlanningRequest`,
  `PlanningOutcome`, `PlanningBudget`, typed `KinematicState`, immutable
  `CandidateBundle`, `CommittedBundleStore` and `CommandSampler`.
- Safety impact: candidate commit compares localization/world/goal/transaction
  identity under a narrow critical section; sampling only evaluates an already
  committed immutable candidate and never calls map/planner code. This is a
  new fail-closed boundary and does not change any existing planner, map,
  freshness, collision, backup or dynamic-limit threshold.
- Removal condition: remove the transitional backend's direct commit/sample
  path only after runtime is wired to these types and repeated recorded-data,
  SITL and sanitizer evidence proves equivalent safety behavior.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to navigation_execution --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && colcon test --packages-select navigation_planning navigation_execution --event-handlers console_start_end+ --return-code-on-test-failure`.
  Current focused result: planning contract tests 3/3 and execution tests
  14/14.

### 2026-08-25 - Typed execution state and PX4 command provenance enforcement

- Owner: `navigation_execution`, runtime propagated-state adapter, navigation
  contracts and PX4 External Mode. Scope: publish propagated state through an
  epoch-tagged `ExecutionStateStore`, clear it on localization reset, use it
  for planner freshness and command source provenance, reject commands whose
  validity window is expired, and bind accepted commands to the active
  mission/request/waypoint identity.
- Safety impact: rejects stale command windows, cross-mission late DDS samples,
  and propagated state surviving a localization epoch transition. The state
  adapter explicitly converts body-frame velocity into world-frame velocity;
  no numeric threshold or UNKNOWN policy changed.
- False-reject risk: a missing child-frame ID, malformed quaternion, missing
  active mission identity, or epoch transition clears command exposure and
  enters the existing hold/handover path. This is intentional fail-closed
  behavior.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to navigation_runtime --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && colcon test --packages-select navigation_contracts navigation_execution navigation_runtime px4_navigation_external_mode --event-handlers console_start_end+ --return-code-on-test-failure`.
  Current focused result: contracts 5/5, execution 14/14, runtime 107/107,
  External Mode 78/78. End-to-end SITL and hardware evidence remain open.

### 2026-08-25 - Product kinematic-state boundary for planner input

- Owner: `navigation_planning`, `navigation_planning_backend` adapter and
  runtime state ingestion. Scope: make `navigation_planning::KinematicState`
  the planner input contract, calculate yaw once at the propagated-odometry
  boundary, and remove the runtime's intermediate vendor `RobotState` adapter.
- Safety impact: source/receive timestamps, localization epoch, world/body
  frames, quaternion validity and yaw are validated in one typed state. No
  planner deadline, collision policy, tracking envelope or numeric gate was
  changed. The backend still converts internally at its private implementation
  boundary.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to navigation_runtime --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && colcon test --packages-select navigation_planning navigation_planning_backend navigation_runtime --event-handlers console_start_end+ --return-code-on-test-failure`.
  Current result: build completed; CTest passed 1/1 planning test executable
  (4 cases), 2/2 planner-backend executables (40 cases), and 8/8 runtime test
  executables. The 85-case XML found in the planning build cache was stale and
  moved to `.artifacts/test-results-cleanup-20260825/` before recounting.
- Removal/review condition: replace the backend's remaining trajectory,
  result-code and map/vendor types in the runtime API with product contracts;
  verify repeated recorded-data/SITL behavior and remove the transitional
  path only after equivalent safety evidence exists. Backend compile warnings
  remain a cleanup item and are not treated as acceptance evidence.

### 2026-08-25 - Product execution command authority

- Owner: `navigation_execution`, runtime command publication and the planner
  adapter. Scope: export the backend's committed trajectory as an immutable
  `navigation_planning::CandidateBundle`, commit it through
  `CommittedBundleStore`, and sample only through `CommandSampler`; preserve
  role, completion, trajectory-time and world certificate metadata.
- Safety impact: the ROS command timer no longer reads or locks the backend
  trajectory directly. Candidate export rejects invalid time/provenance and
  execution-store rejection leaves command exposure disabled. Goal and
  localization transitions clear or explicitly retain the current bundle for
  the existing hot-retarget continuity contract. No planner threshold,
  collision policy, UNKNOWN policy or dynamics value changed.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to navigation_runtime --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && ctest --test-dir build/navigation_planning --output-on-failure && ctest --test-dir build/navigation_execution --output-on-failure && ctest --test-dir build/navigation_planning_backend --output-on-failure && ctest --test-dir build/navigation_runtime --output-on-failure`.
  Current result: build completed; planning 1/1 executable, execution 2/2,
  backend 2/2, and runtime 8/8 passed.
- Remaining condition: retained-command revalidation, runtime vendor-type
  removal from diagnostics/tests, sanitizer evidence, and repeated
  recorded-data/SITL validation remain open. Do not claim end-to-end
  acceptance from this focused result.

### 2026-08-25 - Finite corridor input gate before trajectory optimization

- Owner: `navigation_planning_backend` trajectory optimizer. Scope: validate
  corridor half-space coefficients, plane-normal magnitudes and initialized
  segment durations before normalizing or invoking MINCO; reject the solve when
  these inputs are non-finite or non-positive and report the exact corridor
  index.
- Safety impact: prevents invalid geometry or duration data from entering the
  optimizer and producing an unexplained `NaN`/`Inf` objective. This is a
  fail-closed input validation gate; it does not relax collision, UNKNOWN,
  dynamic-limit or deadline policy and changes no numeric threshold.
- Evidence: SITL artifact
  `.artifacts/runtime/external-mode-check-20260825T163927-167116` observed
  `OptimizationExpTrajInPolytopes` ending with a non-finite MINCO objective;
  focused planning-backend CTest passed 2/2 after the guard.
- Removal condition: none; retain as a permanent finite-input contract. The
  SITL open-route failure still needs a new run with the diagnostic path to
  identify whether the source is generated corridor geometry or optimizer
  arithmetic.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && source install/setup.bash && ctest --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-26 - Shared finite corridor normalization boundary

- Owner: `navigation_planning_backend` trajectory optimizers. Scope: validate
  every half-space matrix before EXP geometry simplification and before backup
  MINCO setup; normalize each row with a stable positive normal norm; reject
  empty, malformed, non-finite, zero-normal, or overflowed normalized matrices.
  The helper is scalar-generic and does not expose a vendor matrix type in its
  product API.
- Physical contract: rows are `[normal_x, normal_y, normal_z, offset]` and the
  half-space is `normal dot position + offset <= 0`. Division by a positive
  norm preserves the inequality and its units; no geometric safety threshold
  or optimizer penalty value changed.
- Safety impact: prevents malformed corridor coefficients from reaching
  `SimplifySFC`, geometric queries, or MINCO. Validation is fail-closed and
  leaves the caller's matrix unchanged when normalization cannot produce a
  finite result. It does not certify backup geometry; HG-013 remains
  provisional because the backup terminal check still uses aggregate penalty.
- Evidence: `python3 tools/runtime/build.py build --packages
  navigation_planning_backend` and `python3 tools/runtime/build.py test
  --packages navigation_planning_backend`; focused tests cover finite positive
  scaling, NaN/Inf/zero normals, invalid offset, normalization overflow,
  empty/wrong-shape matrices and no-mutation-on-failure. Full product and
  repeated dataset/SITL evidence remain required.
- Removal/review condition: retain permanently as the finite-input contract;
  review the backup geometric certificate separately under HG-013. Exact
  verification command is the focused build/test command above.
- Post-commit verification: at `2e916c4`, Release build completed 21/21
  packages, Release product tests completed 13/13 packages, `build.py check`
  reported 68 tests with 0 errors/failures/skips, and ASan build/test completed
  21/21 and 13/13 packages with all executed CTest suites passing and no
  sanitizer report. This confirms the finite-input boundary at source,
  product-test and sanitizer levels; it does not close repeated SITL/dataset
  acceptance or HG-013.

### 2026-08-26 - Retained command revalidates the post-solve execution lease

- Owner: `navigation_runtime` retained-command and emergency-brake validation.
  Scope: after a planner solve returns, reload the immutable propagated-state
  lease instead of reusing the state snapshot captured before the solve.
- Safety impact: retained and measured-state emergency paths now require the
  same finite-state, ROS source-age and steady receive-age contract as command
  publication. A delivery gap cannot be hidden by a fresh ROS header. No
  freshness limit, braking value, collision policy or fallback gate changed;
  invalid state remains fail-closed.
- Evidence: `navigation_execution` focused test now covers a 20 ms source age
  with a 514.760 ms receive age and expects `RECEIVE_STALE`; Release build
  passed for `navigation_execution` and `navigation_runtime`, with 2/2 and
  8/8 CTest executables passing. Repeated dataset/SITL evidence is still
  required to diagnose transport, executor or estimator delivery gaps.
- Review condition: retain the post-solve reload and add an executor/transport
  starvation reproduction before adjusting any runtime scheduling or QoS
  policy. Do not claim this local correction closes the SITL regression.

### 2026-08-26 - Isolate propagated-state ingress from bulk sensor callbacks

- Owner: `navigation_runtime` ROS callback ownership. Scope: assign the
  propagated-odometry subscription to a dedicated reentrant callback group;
  cloud, registered-scan, corrected-odometry and mission callbacks keep their
  existing ownership. The node still uses the existing executor and QoS.
- Safety impact: reduces the chance that synchronous point-cloud decoding or a
  bulk sensor callback prevents the execution-state lease from being refreshed.
  This is a scheduling isolation change only: no freshness limit, queue depth,
  QoS reliability, estimator policy or safety fallback was relaxed. If the
  producer/bridge still stops delivering samples, the steady receive-age gate
  continues to fail closed.
- Evidence: the two advisors identified the default subscription group and
  synchronous registered-scan/cloud decode as a concrete executor-starvation
  risk. Release `navigation_runtime` build passed; its 8/8 CTest executables
  and all 168 runtime Python contract tests passed. A fresh SITL run is required
  to distinguish executor contention from bridge/clock/propagator gaps.
- Review condition: retain only with repeated representative SITL evidence
  showing callback arrival and receive-age distributions; do not increase
  executor threads or alter QoS as a substitute for that evidence.

### 2026-08-26 - Immutable execution-state lease ownership

- Owner: `navigation_execution` and `navigation_runtime`. Scope: publish one
  immutable `ExecutionStateLease` containing the typed kinematic state and an
  accepted-ingress sequence; make that lease the source for planner, retained
  command and command diagnostics; remove the runtime's duplicate propagated
  odometry/receive/sequence fields and the unused legacy planner-state header.
- Safety impact: state and diagnostic sequence now come from the same accepted
  lease under one store lock, so callback reordering cannot expose a new state
  with an old sequence. The sequence is provenance only; source/receive
  freshness, epoch checks, frame checks and fail-closed command behavior are
  unchanged. No threshold, QoS, executor count or fallback policy changed.
- Known limitation: propagated `nav_msgs/Odometry` still lacks producer-owned
  localization epoch provenance; the runtime-assigned epoch remains an open
  safety item and this change must not be cited as closing it.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && source install/setup.bash && ctest --test-dir build/navigation_execution --output-on-failure && ctest --test-dir build/navigation_runtime --output-on-failure`.
  Current result: Release focused build completed for 2 packages; execution
  2/2 and runtime 8/8 CTest targets passed. Runtime Python contracts passed
  141/141.
- Review condition: add producer-owned epoch/generation or an equivalent
  ingress barrier before claiming localization-reset safety is complete; then
  repeat representative dataset/SITL and sanitizer evidence.

### 2026-08-26 - Canonical mapping lifecycle diagnostics

- Owner: `navigation_runtime` diagnostics producer and `tools/runtime/report.py`.
  Scope: serialize the complete `ObservationAccounting::Snapshot` in both
  mapping and planner diagnostic events; use accounting counters rather than
  duplicate cloud counters; classify lifecycle events by schema so
  `DECISION_TRACE` cannot replace a lifecycle snapshot; never treat missing
  counters as zero.
- Safety impact: removes a validation false-positive caused by combining
  `accepted=309` with `published=300` while silently dropping the valid
  `replaced=9` disposition. A real incomplete or inconsistent lifecycle
  remains visible as an evidence failure; no mapping queue, replacement,
  acceptance or safety gate was relaxed.
- Evidence: artifact
  `.artifacts/runtime/external-mode-check-20260825T185357-297920/report.json`
  had `observation_accounting_valid=1`, `violation_count=0` while the old
  report emitted the conservation failure. New report tests cover the exact
  `TRACKING -> DECISION_TRACE -> world PUBLISHED_UPDATED` ordering, rejected
  input conservation and incomplete-schema handling; Python contracts passed
  141/141.
- Verification command:
  `python3 -m unittest tools.runtime.tests.test_runtime_contract && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ctest --test-dir build/navigation_runtime --output-on-failure`.
- Review condition: rebuild the authoritative Release manifest, rerun dataset
  and SITL, and confirm genuine replacement/discard evidence is preserved. The
  simultaneous simulator/transport blackout and mission safety-stop remain
  open and are not hidden by this report correction.
