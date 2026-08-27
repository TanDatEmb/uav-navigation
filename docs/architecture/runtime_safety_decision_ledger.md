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
| HG-002 | EXP corridor plane certificate (`NUMERICAL_TOLERANCE`) | max normalized plane violation 0.01 m | PROVISIONAL | The gate correctly rejects historical 0.2 m excursions and now evaluates polynomial extrema continuously per plane, independent of optimizer `smooth_eps`. Numerical conditioning and the relation to the final world swept certificate remain provisional. Rejections at 0.011-0.014 m must not be handled by silently raising it. | Validate continuous swept trajectory against normalized planes and inflated map; run scale sensitivity on map resolution and conditioning. |
| HG-003 | Vertical guide envelope | removed | REMOVED | The guide is now a route-reference quality input only. Corridor containment, dynamic/flatness gates and the final world-model swept certificate remain authoritative. The former fixed 20-sample/0.05 m gate could false-reject a feasible trajectory inside the corridor. | No reintroduction as a route-reference gate; any future altitude/terrain constraint needs a separately owned safety contract and evidence. |
| HG-004 | Vehicle dynamic and flatness envelope | V/A/J 12/12/30, body rate 5 rad/s, thrust acceleration 6-25 m/s2 | PROVISIONAL | Product config says X500-derived, but controller/PX4/hardware provenance is incomplete. Backup missions may lower these limits. | Link airframe/controller evidence and dataset/SITL distributions; certify continuity and actual PX4 tracking before hardware. |
| HG-005 | Planning radius invariant | 0.35 + 0.25 + 0.05 + 0.10 + 0.05 = 0.80 m | PROVISIONAL | Ownership and sum are explicit and config-validated. Component error budgets are not yet tied to measured distributions. | Derive tracking/localization/mapping p99 independently and preserve the sum invariant. |
| HG-006 | Runtime input freshness | cloud/corrected/propagated maximum age 0.5 s; exact timestamp pairing at the typed observation boundary | PROVISIONAL | Fail-closed works, but 0.5 s is far larger than a 0.2 s planning lead. Recent runs show health exits from scheduling gaps around this boundary. The removed legacy pairing-skew knob is no longer an independent authority. | Express per-stream deadlines from rates and braking envelope; measure dataset and loaded-SITL gap distributions. |
| HG-007 | Safety suffix anchor | maximum state-to-command anchor error 0.75 m | PROVISIONAL | Prevents retaining a geometrically detached suffix, but value is independent of speed, stopping distance and localization confidence. | Replace with speed/covariance-aware contract or derive a certified worst case. |
| HG-008 | Planner watchdog | 1.0 s | PROVISIONAL | Protects command publication from a hung solve, but exceeds the internal 0.18 s solve deadline by 5.6x and invalidates all command availability on expiry. | Align watchdog with cancellability and measured worst-case stage latency; test cancellation and immutable commit under load. |
| HG-009 | Goal connectivity | Shared 3-D completion/connectivity tolerance 0.20 m | PROVISIONAL | Planner endpoint resolution and runtime completion now use one product-owned value; scale/provenance and mission distributions remain provisional. | Validate goal acceptance/rejection across map resolutions and 3-D endpoint cases; retain one shared owner. |
| HG-010 | Retained-suffix swept validation | spatial step 0.5 inflated-map resolution, time step clamped 2-50 ms | PROVISIONAL | Adaptive segment checks fail closed for OCCUPIED and OUT_OF_MAP. Maximum step and map revision stability are not yet recorded in the certificate. | Attach map revision/generation and segment certificate to the committed bundle; test obstacle between legacy 50 ms samples. |
| HG-011 | Hardware Mid-360 visibility | hardware blocked unconditionally until an immutable certificate and verifier exist | CERTIFIED | No boolean can turn an unverified FOV/mounting assumption into evidence. No real-flight visibility certificate exists yet. | Keep hardware blocked until mounting, FOV/blind zones, accumulated observations and motion envelope are certified and verified at runtime. |
| HG-012 | CIRI overlap/seed tolerances (`NUMERICAL_TOLERANCE`) | overlap threshold plus hard-coded 0.01 m and 0.25 ratio; seed clearance `robot_r - 0.01 m` | PROVISIONAL | Different meanings currently share unnamed constants and can reject dense geometry inconsistently. | Name each quantity, record units/reason codes and test resolution scaling before changing values. |
| HG-013 | Backup corridor certificate (`SAFETY_INVARIANT`) | same normalized continuous plane certificate as EXP | PROVISIONAL | Backup now uses the same independent normalized plane certificate rather than a penalty-derived 0.2 m threshold. Final world swept validation and numerical conditioning evidence remain open. | Validate the shared continuous certificate against the final inflated-map/world certificate and preserve the certified seed fallback. |
| HG-014 | Typed estimator-health control gate (`SAFETY_INVARIANT`) | `TRACKING` plus navigation/covariance/observability/correction/propagation flags all true | PROVISIONAL | PX4 now consumes typed `/lio/health`; invalid health fails closed immediately, source timestamps must advance, and the message carries the latest propagated-state timestamp used as the localization-reset barrier. A false accept could expose External Mode during an invalid estimator or public-frame transition, while a false reject causes a bounded hold/handover. | Repeat typed-health negative tests, loaded SITL and recorded-data health timing; certify the typed health lease and reset barrier from repeated evidence. |

## Temporary-bypass register

| ID | Owner/date | Scope | Safety impact | Evidence | Removal condition | Status |
|---|---|---|---|---|---|---|
| TB-001 | planning/runtime; intro `1661386` | Historical EXP jerk objective experiment; no longer exposed by product runner and no longer changes generated planner configuration | Historical false-reject and runtime-tail risk; no active bypass remains | Focused planner/runtime tests after removal; repeated dataset/SITL performance evidence remains open | None for the removed switch; retain history for traceability | REMOVED |
| TB-002 | planning; intro `30ca02c` | Historical `traj_opt.exp_traj.penna_attract` switch was removed; the product-owned waypoint quality term is `traj_opt.exp_traj.objective.waypoint_attraction_weight` and is not a bypass switch | Historical convergence/conditioning risk; no active bypass remains | Product source scan shows no `penna_attract` consumer; planner configuration tests cover the replacement objective field | None for the removed switch; any future objective experiment must be an external, explicitly selected evaluation, not a flight parameter | REMOVED |
| TB-003 | mapping/planning; intro `30ca02c` (`iris_iter_num`) | `planner.iris_iter_num`: current `1`, enabled reference `2`; the unused obstacle-skip setting was removed | May reduce corridor volume; inflation/collision gates remain authoritative | Config-patched dense snapshot and SITL A/B | Dense snapshot benchmark proves constraint coverage, feasible rate and p99 across the iteration matrix | TEMPORARY_BYPASS |
| TB-004 | PX4/runtime; intro `2026-08-25` | Historical DiagnosticArray health fallback was removed; only typed `/lio/health` controls External Mode | Historical compatibility risk; no active fallback remains | PX4 source scan has no DiagnosticArray health subscription/parser; External Mode CTest passes | None for the removed fallback; keep the historical record and typed-health negative/replay evidence | REMOVED |
| TB-005 | runtime/mapping; intro `2026-08-25`, removed `2026-08-27` | Legacy cloud plus corrected-odometry pairing was a compatibility fallback before the atomic `RegisteredScan` contract was made mandatory | Retained pairing/skew and duplicate-input risk for legacy-only launches | Typed-only runtime source, canonical profile, shutdown test publishing `RegisteredScan`, sourced runtime CTest and Python contract tests | None; retain this historical row for traceability | REMOVED |

Register anchors for the remaining rows are explicit. TB-003 is limited to
`iris_iter_num`, which
changed from 2 to 1 in `30ca02c`. The previously loaded `obs_skip_num` had no
consumer in corridor generation and was removed rather than preserved as a
no-op configuration surface. Its A/B must restore `iris_iter_num=2` versus the
current `1` in the same config. Until explicit switches exist, patch the
generated config manually,
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

### 2026-08-26 - Invalidate execution bundle on world revision advance

- Owner: `navigation_execution::CommittedBundleStore`; scope: the mapping
  publication to command-sampling boundary in `publishWorldIdentity()`.
  When a strictly newer immutable world identity is accepted, the store clears
  the committed candidate before exposing the new identity.  An exact-world
  candidate can be committed again only after the planner has produced and
  validated it against that identity.
- Safety impact: `SAFETY_INVARIANT`, fail closed.  A candidate certified on a
  previous map revision must not remain executable after a newer observation
  changes the world.  The deliberate false-reject consequence is a bounded
  command gap while replanning; retaining the candidate would permit a false
  accept if a newly observed obstacle intersects it.  Goal transitions and
  localization resets continue to invalidate independently.
- Derivation and cost: no threshold or new timing value was introduced.  The
  identity equality/advance relation already owned by
  `navigation_world_model` remains authoritative.  Clearing one shared
  pointer is constant-time; measure command-gap and replan p50/p95/p99 on
  repeated SITL and recorded-data runs before considering a certificate-horizon
  optimization.
- Evidence: `test_committed_bundle_store` now proves that a committed bundle
  is unavailable after world advance, an old token is rejected, and a fresh
  candidate for the new identity can commit.  Full runtime, dataset and SITL
  evidence remain open.
- Removal/review condition: do not remove this invalidation until an explicit
  execution-side certificate proves the in-flight bundle safe against the
  newer world and has regression, sanitizer, dataset and repeated SITL
  evidence.  Verification command:
  `colcon test --packages-select navigation_execution navigation_runtime
  --event-handlers console_direct+`.

### 2026-08-26 - Make backup corridor authorization independent of optimizer penalties

- Owner: `navigation_planning_backend::BackupTrajOpt`; scope: both backup
  refinement entry points and the shared corridor-plane validation helper.
  Plane rows are normalized before optimization, and the final trajectory is
  sampled against those planes to compute a maximum geometric violation in
  metres.  `penalty_log` remains diagnostic only; `penna_pos` no longer enables,
  disables, or scales the safety decision.
- Safety impact: `SAFETY_INVARIANT`, fail closed.  The previous gate used
  `0.2` and `penna_pos * 0.05`, which coupled a safety certificate to an
  optimizer weight and could accept an out-of-corridor backup when the weight
  was disabled or when the log was stale.  The change may increase false
  rejects for a sampled excursion above the configured tolerance; it removes
  this false-accept path.  Continuous extrema between samples remain an open
  limitation recorded by HG-013.
- Derivation and cost: the comparison is against the existing
  `corridor_plane_tolerance_m` in metres after row normalization; no new
  threshold was introduced.  The final pass adds the existing
  `integral_reso` samples per piece and a constant amount of state; measure
  backup p50/p95/p99 and rejection reasons before changing sampling policy.
- Evidence: `test_planner_config` now verifies the geometric violation is
  invariant under plane scaling.  The backend build and focused test are
  required before closing this entry; dataset, repeated SITL, sanitizer and
  hardware evidence remain open.
- Removal/review condition: replace sampled authorization only with an
  analytic or bounded continuous certificate that has scale, dataset,
  sanitizer and repeated SITL evidence.  Verification command:
  `colcon test --packages-select navigation_planning_backend --event-handlers
  console_direct+`.

### 2026-08-26 - Make A* search deadlines absolute and heap entries immutable

- Owner: `navigation_planning_backend::Planner::PathSearch` and
  `path_search::Astar`. Scope: the escape, preferred-altitude, unrestricted,
  and probability-map alternatives now share one stage budget derived from the
  existing absolute solve deadline. A* measures cancellation with
  `std::chrono::steady_clock`; a caller-supplied zero budget is an immediate
  timeout, not a request to restore the configured default. The open set stores
  an immutable `(node, score, round)` entry and discards stale entries after
  pop, because mutating a node through an existing `priority_queue` pointer
  does not implement decrease-key.
- Safety impact: performance/liveness and algorithmic correctness. A frozen
  simulation clock can no longer keep A* running past its wall-time budget, and
  fallback attempts cannot multiply the A* stage latency. The change does not
  relax occupancy, UNKNOWN, OUT_OF_MAP, diagonal-segment, corridor, or
  trajectory safety gates. A false timeout can reject a feasible route; a
  missing timeout or stale heap ordering can block the callback or return a
  non-shortest route.
- Derivation and cost: the stage budget remains the configured A* total limit
  intersected with the caller's absolute solve budget; no new threshold was
  tuned. The extra queue entry score/round is constant-size overhead. Measure
  A* p50/p95/p99 and timeout rates on repeated open, detour, no-path, and frozen
  `/clock` cases before changing any budget.
- Evidence: sourced C++20 backend build succeeded and
  `test_planner_runtime_context` passed 3/3, including immutable open-set
  ordering. End-to-end SITL remains unavailable in the sandbox and is not
  represented as acceptance evidence.
- Removal/review condition: retain until repeated dataset shadow planning and
  structured SITL show the stage distribution and no timeout multiplication;
  review if the queue implementation or deadline owner changes.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/navigation_planning_backend --target test_planner_runtime_context -j1
  && build/navigation_planning_backend/test_planner_runtime_context
  --gtest_color=no`.

### 2026-08-26 - Make degenerate CIRI tangent construction fail closed

- Owner: `navigation_planning_backend::CIRI`. Scope:
  `findTangentPlaneOfSphere` now rejects non-finite/inside-sphere/tangent-limit
  inputs and uses a least-aligned coordinate axis for a collinear seed instead
  of normalizing a zero cross product. Its caller propagates the failure as a
  corridor failure; no NaN plane is allowed to continue to optimization.
- Safety impact: numerical safety invariant. This prevents invalid planes,
  invalid rotations, and square roots of negative tangent radicands from
  entering corridor authorization. A false reject is possible for geometry at
  the numerical tangent boundary; false acceptance of a non-finite plane is
  explicitly prohibited. No obstacle margin or occupancy policy was relaxed.
- Derivation and cost: the tolerance is scale-aware machine precision
  (`128 * epsilon * max(1, geometry scale)`), not a mission-tuned clearance.
  The fallback axis is deterministic and constant-time.
- Evidence: sourced backend compilation passed through `ciri.cpp`; the focused
  runtime-context test passed 3/3. A dedicated CIRI collinear/near-tangent
  characterization and repeated planner replay remain required before this
  numerical behavior is certified.
- Removal/review condition: replace the sampled tangent construction only after
  an analytic sphere-plane certificate and equivalent geometry test vectors are
  available.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/navigation_planning_backend --target test_planner_runtime_context -j1
  && build/navigation_planning_backend/test_planner_runtime_context
  --gtest_color=no`.

### 2026-08-26 - Parse shared YAML configuration once per loader

- Owner: `navigation_math::YamlLoader`. Scope: load the YAML document in the
  constructor and traverse the cached immutable document for all parameter
  reads. Nested lookup uses const node access so a missing key cannot mutate
  the cached tree. Existing default/required semantics are intentionally
  unchanged in this batch.
- Safety impact: startup performance and configuration consistency. Repeated
  file parsing previously multiplied initialization I/O and allowed a path
  lookup to observe a separately parsed document. This change does not make a
  permissive default into a safety authorization; strict schema, units, and
  cross-field validation remain a separate contract phase.
- Derivation and cost: one parse and one document lifetime per configuration
  object; lookups remain proportional to path depth. No runtime planner gate
  or numeric threshold changed.
- Evidence: `test_rog_map_vendor` passes 14/14, including nested reads and a
  required-missing fail-closed case. The existing map fixture behavior remains
  unchanged after fixing const traversal.
- Removal/review condition: retain as the sole shared loader. Before changing
  defaults or adding strict unknown-key rejection, add typed schema tests for
  mapping, planning, and execution configuration with explicit units.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/rog_map_vendor --target test_rog_map_vendor -j1 &&
  build/rog_map_vendor/test_rog_map_vendor --gtest_color=no`.

### 2026-08-26 - Validate stale solve candidates on the latest immutable world

- Owner: `navigation_planning_backend::Planner::authorizeAndCommit`. Scope:
  remove the pre-validation exact-identity rejection between the solve-pinned
  view and the latest published view. Candidate swept validation now runs on
  the latest immutable view; `WorldSnapshotStore::commitIfCurrent()` remains
  the short publication/commit linearization point and rejects a publication
  racing that validation.
- Safety impact: liveness correction with fail-closed commit semantics. A
  fresh obstacle or OUT_OF_MAP segment in the latest view still rejects the
  candidate. The change prevents unrelated map revisions from starving every
  60-100 ms solve under a 10-20 Hz mapper. It does not permit a candidate to
  commit without latest-world geometric validation.
- Derivation and cost: no gate value changes. Validation cost remains the
  existing swept spatial/time sampling contract and is paid once on the
  latest view rather than after an identity equality branch.
- Evidence: source review identified the pre-check as the liveness boundary;
  the existing world-store tests continue to require rejection when a
  publication occurs during the short authorized commit. A focused planner
  test with a stale pinned view and a safe advanced latest view, plus loaded
  dataset and repeated SITL evidence, remains required before certification.
- Removal/review condition: retain until candidate validation and commit
  certificates expose both pinned and validated identities in all runtime
  reports and the stale-solve acceptance matrix is repeated.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/navigation_planning_backend --target navigation_planning_backend -j1`;
  then run the planner facade/world-store CTest targets and the repeated
  dataset/SITL stale-world scenarios.

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
- Historical note: the earlier WM-3B contract allowed a BACKUP visibility
  result to remain traversable through occupancy UNKNOWN. That contract is
  superseded by the 2026-08-26 safety-certificate change below. Current
  product authorization requires BACKUP samples and swept segments to provide
  KNOWN_FREE evidence; raycasting-disabled or endpoint-only evidence is not a
  hardware-flight certificate.
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
- Default behavior remains fail-closed with `traj_opt.exp_traj.objective.jerk_penalty_weight`
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
- Historical correction (2026-08-26): the indexed table described above was
  subsequently removed. The current implementation derives asymmetric local
  extents from the pinned `WorldGeometry` and uses a sparse touched-node map;
  this correction does not change the ownership result or retroactively turn
  the earlier verification into a performance acceptance claim.

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
  correction and valid propagated state. DiagnosticArray is observability-only
  and no longer controls External Mode. No numeric threshold, map
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
  finite result. It does not by itself certify backup geometry; the separate
  known-free backup certificate below remains authoritative.
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

### 2026-08-26 - Post-lease validation evidence

- Dataset artifact `.artifacts/runtime/dataset-20260825T192106-320079` on
  `68879a0` had full source coverage, propagated-odometry max gap `29.804 ms`,
  mapping accounting valid with zero violations, and shadow planning 100
  ready/zero emergency commands. This is recorded-data health evidence only;
  `flight_acceptance=false` remains by contract.
- SITL artifact
  `.artifacts/runtime/external-mode-check-20260825T192608-321637` remained
  `BLOCKED`: `main_minco` produced a non-finite objective, no candidate/PVA
  was published, safety stopped and mission stopped at waypoint `[0]`.
  Mapping accounting was valid and the previous report conservation
  false-positive did not recur. IMU and external-odometry arrival gaps reached
  `398.735 ms` and `404.068 ms`; no freshness gate was relaxed.
- Cleanup evidence: both validation sessions ended `STOPPED` with
  `cleanup=PASS`; no runner, ROS, simulator, PX4, agent worktree or temporary
  agent directory remained. These results do not close MINCO/corridor,
  transport-gap, repeated SITL, or producer-epoch safety conditions.

### 2026-08-26 - Non-finite optimizer evaluation provenance

- Owner: `navigation_planning_backend` EXP trajectory optimizer and the runtime
  planner trace adapter. Scope: record the first evaluation that becomes
  non-finite, including the validation stage, solver attempt/iteration and
  duration range; reject non-finite objective or gradient before returning to
  L-BFGS. This is diagnostic instrumentation plus fail-closed validation; it
  does not change hard gates or candidate authorization.
- Safety impact: invalid MINCO/constraint/gradient state can no longer be
  silently returned as a usable solver evaluation. The planner remains
  fail-closed when the retry encounters `LBFGSERR_INVALID_FUNCVAL`; no fallback,
  threshold relaxation, duration clamp or bypass was introduced.
- Evidence: Release build of `navigation_planning_backend` and
  `navigation_runtime` passed; planning CTest 2/2, runtime CTest 8/8 and
  Python runtime/planner contracts 147/147 passed. The next replay must use
  the recorded fields to identify the first invalid operand before retry
  parameterization or numerical solver behavior is changed.
- Verification command:
  `source /opt/ros/jazzy/setup.bash && source install/setup.bash && ctest
  --test-dir build/navigation_planning_backend --output-on-failure && ctest
  --test-dir build/navigation_runtime --output-on-failure && python3 -m
  unittest tools.runtime.tests.test_planner_trace tools.runtime.tests.test_runtime_contract`.
- Review condition: preserve the existing fail-closed behavior and keep the
  SITL artifact `external-mode-check-20260825T192608-321637` as `BLOCKED` until
  a replay identifies and fixes the actual non-finite source, then repeat
  dataset/SITL and adversarial review.

### 2026-08-26 - Post-instrumentation validation evidence

- Dataset artifact
  `.artifacts/runtime/dataset-20260825T195426-343584` on `77d7cdd` had full
  IMU/LiDAR coverage, LIO tracking, propagated odometry at `49.9996 Hz`,
  mapping lifecycle `2756/2756` with zero accounting violations, and shadow
  planning `100 READY/0 EMER`. Its contract remains dataset health only;
  `flight_acceptance=false`.
- SITL artifact
  `.artifacts/runtime/external-mode-check-20260825T195939-345561` remained
  `BLOCKED`: the retry objective became non-finite at diagnostic stage `5`,
  attempt `2`, iteration `46`, with duration range `2.601398..4.956079 s`;
  no candidate/PVA was published. The same run observed a common clock and
  transport blackout with gaps up to `783.1 ms` across clock/LiDAR/odometry.
  No freshness, lease or mission gate was relaxed.
- Cleanup: both validation agents returned and were closed; the repository
  retains only the main worktree, no runner/ROS/LIO/Gazebo/PX4 process remains,
  and no agent temporary directory remains. These results identify the retry
  objective as the next product fix but do not close SITL acceptance.

### 2026-08-26 - Feasibility retry duration parameterization

- Owner: `navigation_planning_backend` EXP trajectory optimizer. Scope: treat
  the retry reserve as a dimensionless initialization factor over the last
  finite segment durations; store only the additive per-segment duration
  reserve in seconds and initialize the free duration from the last finite
  solution. Validate the base duration, reserve, resulting offset and mapped
  tau before entering L-BFGS. The reserve is a retry seed and additive floor
  for the free-duration component, not an authorization to skip final dynamic
  or geometric gates.
- Safety impact: removes the previous combination of a full-duration floor
  and a separate 1% free-duration seed, which put the retry in a poorly scaled
  parameterization before the stage-5 non-finite objective. Existing corridor,
  vertical-guide, V/A/J, cancellation and fail-closed candidate gates remain
  unchanged. No fallback, threshold relaxation, QoS change or gate bypass was
  introduced.
- Evidence: focused Release build passed; planning CTest 2/2, runtime CTest
  8/8 and Python runtime/planner contracts 147/147 passed after the change.
  Representative dataset/SITL replay on the previous diagnostic checkpoint
  identified the stage-5 failure; a fresh authoritative build and repeated
  dataset/SITL evidence on this exact change are required before sign-off.
- Review condition: verify finite retry objective/gradient, preserve accepted
  candidate transactionality on retry failure, and run repeated representative
  dataset/SITL distributions. Do not claim this parameterization fixes the
  independent simulator/transport blackout until those runs separate the two
  blockers.

### 2026-08-26 - Feasibility retry candidate transactionality

- Owner: `navigation_planning_backend` EXP trajectory optimizer. Scope: retain
  every finite strict improvement (or a candidate that already satisfies the
  dynamic envelope), and snapshot/restore the complete accepted optimizer
  state when a retry fails, leaves the corridor, or makes no progress. The
  snapshot covers decision variables, additive duration floor, penalty weights,
  penalty diagnostics, objective and iteration count. Route-reference weights
  and deadbands are immutable solve configuration and are never derived from
  or scaled with feasibility penalties;
  `final_duration_s` is emitted only after all physical, vertical-guide and
  flatness gates pass. Planner trace now preserves the producer's explicit
  `commit_decision` separately from `candidate_result`.
- Safety impact: prevents a rejected retry's mutable objective state or stale
  duration from being reported as the selected candidate. The world/corridor,
  V/A/J, flatness, cancellation, execution commit and PX4 freshness gates are
  unchanged; no fallback, threshold relaxation, bypass or candidate
  authorization was introduced.
- Evidence: advisors traced the exact SITL pair
  `1.024820197132971 -> 1.0248201536822739`; the prior `1e-6` deadband wrongly
  discarded this finite improvement and the trace reported duration
  `7.021931 s` with no committed bundle. Focused Release build passed;
  planning CTest 2/2, runtime CTest 8/8 and Python contracts 147/147 passed.
  Fresh authoritative build plus repeated dataset/SITL evidence on this change
  is required before closure.
- Review condition: add deterministic retry rollback/property coverage where
  the optimizer test seam permits it, retain analytic hard extrema as the
  acceptance authority, and do not infer that increasing duration guarantees
  lower V/A/J with non-zero boundary PVAJ or changing geometry.

### 2026-08-26 - Retry transactionality validation evidence

- Dataset artifact `.artifacts/runtime/dataset-20260825T202030-362330` on
  `7cea790` completed with `PASS`, full IMU/LiDAR source coverage, LIO
  tracking, mapping accounting `2756/2756` with zero violations, and planner
  trace `exp_nonfinite_evaluation_count=0`, `exp_first_nonfinite_value_mask=0`.
  This remains recorded-data health evidence only; `flight_acceptance=false`.
- SITL artifact
  `.artifacts/runtime/external-mode-check-20260825T203002-365806` on
  `7cea790` is `BLOCKED` fail-closed: non-finite count/mask were both zero,
  but the retry stopped at `no progress` with velocity violation and
  `initial=1.02482`, no committed candidate/PVA, safety stop at `26.6 s`,
  only waypoint `[0]`, and cross-track p95 `0.722 m`. The result exposed the
  deadband and transactionality defects above; no safety gate was relaxed.
- Validation orchestration first had the SITL agent blocked by a dataset
  runner lock. The dataset session was stopped safely, state became
  `STOPPED/cleanup=PASS`, and the SITL rerun was executed only after process
  and lock cleanup. All agents were closed; only the main worktree remained.

### 2026-08-26 - Feasibility solver stopping semantics

- Owner: `navigation_planning_backend` EXP optimizer and trajectory config.
  Scope: keep the nominal L-BFGS relative-cost stopping rule for the ordinary
  solve, but disable that rule during bounded feasibility retries because cost
  plateau is not a dynamic-feasibility certificate. Retry mode uses the named
  `traj_opt/exp_traj/feasibility_retry_max_iterations` bound and retains the
  existing steady deadline/cancellation monitor. Reaching the iteration bound
  exposes a finite candidate to the same corridor, V/A/J, vertical-guide and
  flatness gates; it does not authorize the candidate.
- Safety impact: prevents `LBFGS_STOP` from ending a retry while the analytic
  velocity/acceleration/jerk extrema still violate mission limits, without
  relaxing any gate. The finite iteration bound limits CPU tail when a solve
  has no deadline; cancellation and fail-closed transaction restore remain
  active. No fallback, threshold relaxation or bypass was introduced.
- Evidence: exact-head `906be2c` SITL showed retries ending with solver return
  `LBFGS_STOP` while velocity ratio remained `1.024216`, with `177790 us` of
  budget remaining. Focused and full Release build/test/check passed after the
  mode/config change; a fresh exact-head SITL and dataset replay is required.
- Review condition: report raw solver return code, retry iteration/cancel
  outcome and latency distribution; accept only a candidate that passes every
  hard gate. Do not increase the iteration bound or infer feasibility from a
  single replay.

### 2026-08-26 - Retry duration lower-bound invariant

- Owner: `navigation_planning_backend` EXP trajectory optimizer. Scope: rename
  the retry duration offset to `duration_lower_bound`; anchor it once to the
  nominal finite segment durations before the retry loop; initialize a separate
  strictly positive `free_duration_seed_s`; use the same additive
  `T = lower_bound + forwardMapTauToT(tau)` semantics in the objective and
  candidate rebuild. A lower-bound size mismatch is now rejected fail-closed.
- Safety impact: the retry can no longer optimize a segment below the nominal
  duration merely because the reserve was stored as the lower bound. This fixes
  an invalid parameterization invariant; it does not claim that longer duration
  guarantees lower V/A/J with fixed boundary PVAJ, and it does not authorize a
  candidate. Analytic dynamic extrema, corridor, vertical-guide, flatness,
  cancellation and transactional commit gates remain authoritative. No
  threshold, fallback or bypass changed.
- Evidence: duration-map property and round-trip tests pass; planning CTest
  2/2, runtime CTest 8/8 and Python runtime/planner contracts 147/147 pass;
  focused Release targets build. The exact `363cad4` SITL artifact recorded
  raw `LBFGSERR_MAXIMUMLINESEARCH (-1009)`, velocity ratio `1.02482`, no
  candidate/PVA and no commit, while the exact dataset artifact remained
  healthy at sensor/LIO/mapping level but failed its shadow EMER-before-READY
  contract. Fresh authoritative build, dataset replay and repeated SITL are
  required before closure.
- Review condition: retain the fail-closed path and inspect the new lower-bound
  and free-seed trace fields on replay; do not treat finite seed, iteration
  limit, raw solver success or one replay as a feasibility certificate. If
  `-1009` persists, investigate line-search/objective discretization and
  conditioning separately from the duration invariant.

### 2026-08-26 - Lower-bound checkpoint validation evidence

- Authoritative Release checkpoint: commit `c41f3eb` with manifest source HEAD
  matching the commit; tracked worktree clean except the pre-existing
  untracked `test-results-asan/` directory. The duration property tests,
  planning/runtime CTest and Release selector were green before replay.
- Dataset artifact
  `.artifacts/runtime/dataset-20260825T214950-436370/report.json` is `FAIL`:
  the bounded run replayed only `19,435/55,435` IMU and `972/2,772` LiDAR
  samples (35.06%), propagated odometry remained `49.998 Hz` with a
  `26.026 ms` maximum gap, but mapping recorded one pending-cloud replacement
  and one accounting violation. Shadow planning produced no READY/commit and
  rejected a flatness report with non-finite thrust. This is not a product
  acceptance result; the incomplete source window and planner finite-output
  failure must be separated in a longer, complete replay.
- SITL artifact
  `.artifacts/runtime/external-mode-check-20260825T214922-434888/report.json`
  is `FAIL` fail-closed: PX4 entered External Mode but arm was rejected by
  `pre_flight_checks=false`, `ekf2 missing data` and unstable heading. No
  planner cycle, candidate, PVA, commit or duration trace was observed;
  IMU/external-odometry gaps reached `391.78/397.91 ms`. This run cannot
  validate or refute the retry parameterization and does not authorize any
  freshness/preflight relaxation.
- Cleanup: both sessions ended `STOPPED/cleanup=PASS`; all validation agents
  were closed and no ROS, PX4, Gazebo, XRCE, runner or detached agent
  worktree remained. Repeat a complete dataset replay and a preflight-clean
  SITL before closing the retry-duration item.

### 2026-08-26 - Shared time conversion and local clock boundary

- Owner: `navigation_common`, runtime input pairing, and PX4 odometry bridge.
  Scope: make ROS timestamp-to-nanosecond, PX4 microsecond conversion, and
  steady receive/latency sampling use one product-owned utility implementation.
  `steadyClockNowNanoseconds()` is explicitly local steady time and is not
  comparable with ROS, simulation, or sensor source timestamps.
- Safety impact: behavior-preserving refactor only. Malformed/zero/negative
  timestamps still become the existing fail-closed sentinel at each consumer;
  ROS freshness remains ROS-clock based and receive/deadline measurements
  remain steady-clock based. No clock-domain assumption, freshness threshold,
  frame contract, fallback, or bypass changed.
- Evidence: added common nanosecond-to-microsecond and steady-clock tests,
  removed the duplicate runtime timestamp and steady-clock helpers, and made
  PX4 `TimestampConverter` call the shared microsecond conversion. Focused
  build/test and full Release validation are required before checkpoint.
- Removal condition: none; this is the canonical utility boundary. Later
  migration of FAST-LIO clock-domain types, manual ROS time arithmetic, and
  frame/quaternion/covariance conversion is a separate phase with its own
  tests and ledger evidence.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build && python3 tools/runtime/build.py --mode release test && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Runtime planner status boundary

- Owner: `navigation_planning::PlannerStatus` and the runtime planner FSM.
  Scope: move planner-policy classification onto a product-owned enum and keep
  backend integer return-code mapping in the runtime implementation source.
  `planner_fsm.hpp` no longer includes the backend planner header.
- Safety impact: behavior-preserving vocabulary refactor. Success, finish,
  retained-command, restart, retry, emergency and fail-closed dispositions map
  one-to-one with the previous policy. No optimizer, deadline, UNKNOWN policy,
  continuity rule, fallback or safety gate changed.
- Evidence required: product FSM tests, full Release build/test/check and a
  clean installed consumer that includes the product status header without a
  backend header. The planner class itself and runtime node ownership remain a
  later PImpl phase.
- Removal condition: none; new backend codes must be translated at this one
  implementation boundary before entering runtime policy code.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build && python3 tools/runtime/build.py --mode release test && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Package export contract correction

- Owner: `navigation_mapping` and `navigation_runtime` CMake package boundaries.
  Scope: export the targets that are already installed, declare the installed
  include directory, and make `navigation_mapping` propagate its direct world
  model and ROS message interface dependencies.
- Safety impact: packaging/build contract only. No planner, mapper, estimator,
  timestamp, frame, unknown-cell, deadline, or runtime safety behavior changed;
  no fallback or bypass was added.
- Evidence: clean external tree `/tmp/uav-navigation-arch-kmPKjc` configured
  and built the dependency chain, focused CTest completed `12` tests with
  `0` errors/failures/skips, and an installed consumer configured and linked
  through `navigation_mapping::navigation_mapping`,
  `navigation_planning::navigation_planning`, and
  `navigation_runtime::navigation_runtime_core`. This does not close the
  planner facade or vendor quarantine; public headers still require a later
  boundary phase.
- Removal condition: none; every installed target must have a corresponding
  export file and every public header dependency must be represented by a
  direct target/interface dependency.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build && python3 tools/runtime/build.py --mode release test && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Contract checkpoint validation evidence

- Checkpoint `1a4b3f8` had an authoritative Release manifest matching HEAD,
  with only the pre-existing untracked `test-results-asan/` directory. The
  vertical contract tests passed; the subsequent parallel validation obeyed
  the canonical lock and made no bypass or gate change.
- Dataset validation was `BLOCKED_ENVIRONMENT` before session creation because
  the SITL runner held `.build-runtime.lock`; no dataset artifact or product
  claim was made.
- SITL artifact
  `.artifacts/runtime/external-mode-check-20260825T225925-499882/report.json`
  is `BLOCKED` fail-closed. PX4 preflight, arm, takeoff and External Mode
  entry passed, but the planner committed no PVA candidate: the final EXP
  dynamic violation was `1.008735`, the last retry returned `-1008`, and no
  valid trajectory/setpoint with finite yaw/rate was observed. Safety stopped
  at `28.52 s`, PX4 Hold handover followed in `148 ms`, and only waypoint `0`
  of `0..4` was accepted. Independent timing evidence shows corrected-odom
  arrival gaps up to `1624.7 ms` and external-odom callback/arrival stalls;
  this is a simulator/transport blocker distinct from planner feasibility.
- Cleanup: SITL ended `STOPPED/cleanup=PASS`; both validation agents were
  closed, stale lock metadata was removed only after verifying its PIDs were
  dead, and the main checkout has one worktree with no runner/PX4/Gazebo/XRCE
  process remaining. This evidence does not close the planner or SITL gates.

### 2026-08-26 - Canonical dynamic-limit product contract

- Owner: `navigation_planning::DynamicLimits` and the runtime mission-file
  adapter. Scope: remove the duplicate `DynamicsLimits` contract from
  `planning_request.hpp`, make `planning_limits.hpp` the single product-owned
  definition, and keep `mission_dynamics.hpp` independent of the planner
  implementation header.
- Safety impact: type/ownership refactor only. Field meanings and positive,
  finite validation are unchanged; planner optimizer limits, unknown-cell
  policy, continuity, deadlines, and all safety gates are unchanged. No
  fallback or bypass was added.
- Evidence required: planning contract test for valid/invalid limits, focused
  runtime mission-adapter test, full Release build/test/check, and a clean
  installed-header compile audit. This phase does not claim the planner PImpl
  boundary is closed.
- Removal condition: none; `navigation_planning::DynamicLimits` is the sole
  product type. Future limit changes must update this contract and its tests,
  not create another spelling or backend alias.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build && python3 tools/runtime/build.py --mode release test && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Deferred activation of a committed trajectory

- Owner: `navigation_execution::CommandSampler` and the runtime command
  publication boundary. Scope: distinguish an immutable candidate that has
  committed successfully from one whose declared `valid_from_ns` has arrived.
  A command publication tick before the trajectory start now retains the
  committed bundle and defers sampling; it does not expose a zero command and
  does not latch planner failure.
- Safety impact: closes a false emergency path exposed by the exact dataset
  artifact: the candidate committed at generation `1`, but its trajectory start
  was about `10.617 ms` after the publication/source timestamp, so the sampler
  correctly rejected a pre-start evaluation and runtime incorrectly converted
  that expected activation lead into terminal `REJECTED/EMER`. The planner's
  negative pre-start trajectory-time semantics and fail-closed rejection for
  evaluator failure, expiry, non-finite output, stale execution, and continuity
  violations remain unchanged. No freshness threshold, activation window,
  fallback, or bypass was added.
- Evidence: the exact artifact
  `.artifacts/runtime/dataset-20260825T222025-461002/report.json` records full
  IMU/LiDAR coverage, LIO TRACKING, mapping `2756/2756` with zero accounting
  violations, commit generation `1`, then `READY=0/EMER=1`; the new sampler
  regression covers before-boundary retention, boundary evaluation, and expiry.
  Rebuild, full tests, and repeated dataset plus preflight-clean SITL are
  required before closure.
- Removal condition: none; this is an explicit execution state distinction.
  Any future change that holds the trajectory at `t=0` before activation must
  be a separate motion-contract change with P/V/A/J/yaw continuity evidence.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build && python3 tools/runtime/build.py --mode release test && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Activation-boundary checkpoint validation

- Authoritative Release checkpoint: commit `a791576` with manifest source HEAD
  matching the commit; the only remaining worktree item is the pre-existing
  untracked `test-results-asan/` directory. Release build completed `21/21`
  packages, the product selector completed `13/13` packages, and
  `build.py check` reported `68` tests with `0` errors, failures, or skips.
- Dataset artifact
  `.artifacts/runtime/dataset-20260825T223901-478552/report.json` is `PASS`
  as recorded-data health evidence: LIO tracking coverage `1.0`, accepted
  corrections `2754/2754`, mapping received/accepted/started/updated/published
  `2756/2756/2756/2756/2756` with zero failures/drops/accounting violations,
  propagated odometry `13772` samples at approximately `50 Hz`, and shadow
  planning `50 READY/0 EMER` on generation `1`. `flight_acceptance=false` is
  intentional for this dataset workflow; PX4/mission acceptance is not claimed.
  The report also records synthetic operator-takeover teardown and diagnostic
  duplicate/regression observations; these are preserved for follow-up and do
  not justify relaxing product gates.
- The parallel SITL agent was `BLOCKED_ENVIRONMENT` before session creation
  because the dataset runner held the canonical runtime lock. No SITL artifact,
  PX4, planner, odometry, or continuity claim was made. The dataset completed
  `STOPPED/cleanup=PASS`; both agents were closed, the stale lock metadata was
  removed only after verifying its PIDs were dead, and no validation process or
  detached worktree remains.
- Closure status: the recorded-data activation-boundary symptom is reproduced
  as fixed on this checkpoint, but repeated dataset evidence and a preflight-
  clean SITL run remain required for end-to-end closure.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build && python3 tools/runtime/build.py --mode release test && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Planner implementation isolation from runtime public header

- Owner: `navigation_runtime::NavigationRuntimeNode` and the
  `navigation_planning_backend` implementation boundary. Scope: remove planner
  implementation headers from the installed runtime node header, forward
  declare the private planner/context types, and retain ownership in the
  out-of-line runtime implementation. The planner remains behaviorally
  unchanged; this is the first compile/include fence before a complete product
  planner facade.
- Safety impact: no planner result, candidate validity, world identity,
  timestamp, frame, dynamic-limit, deadline, unknown-cell, fallback, or PX4
  gate changed. No bypass or relaxed validation was added. The change reduces
  accidental coupling but does not yet close direct backend use in the runtime
  implementation or vendor header installation.
- Evidence: `python3 tools/runtime/build.py --mode release build
  --packages navigation_runtime` completed; the package test selector passed
  `8/8`; an installed external consumer including product mapping/planning
  headers and linking `navigation_runtime::navigation_runtime_core` configured
  and linked successfully from a clean CMake build directory
  `/tmp/uav-navigation-public-consumer-hS52j8`.
- Removal condition: none. The temporary compatibility boundary is complete
  only when runtime implementation code consumes a product planner facade and
  the backend install has an explicit private/public allowlist; until then the
  remaining direct backend calls are an open refactor item.
- Verification command: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/runtime/build.py --mode release build --packages navigation_runtime && python3 tools/runtime/build.py --mode release test --packages navigation_runtime && source /home/letandat/Dev/uav-navigation/install/setup.bash && cmake -S /tmp/uav-navigation-arch-kmPKjc/consumer -B /tmp/uav-navigation-public-consumer-hS52j8 -DCMAKE_BUILD_TYPE=Release && cmake --build /tmp/uav-navigation-public-consumer-hS52j8 --parallel 1`.

### 2026-08-26 - Planner-status checkpoint validation

- Checkpoint `b3c94b6edd1ae462d01c9611376c1773e0006d00` had an authoritative
  Release manifest before validation; the only source-tree item was the
  pre-existing untracked `test-results-asan/` directory. The dataset artifact
  `.artifacts/runtime/dataset-20260825T231828-526332/report.json` reported
  `PASS` for recorded-data health: full IMU/LiDAR source coverage, LIO
  tracking coverage `1.0`, mapping `2756/2756` with zero accounting
  violations, propagated odometry near `50 Hz`, and shadow planning
  `51 READY/0 EMER` on generation `1`. Its `flight_acceptance=false` and
  synthetic operator-takeover teardown remain explicit; no end-to-end flight
  claim is made.
- The parallel SITL artifact was `BLOCKED_ENVIRONMENT` before launch because
  the dataset runner held the canonical `.build-runtime.lock`; PX4, planner
  trajectory, odometry continuity, and mission evidence were not collected.
  The dataset agent was closed after its report, the recorded owner PID was
  verified dead, the stale lock was removed, and the main checkout was left
  with one worktree and no runtime runner process.
- Closure status: this confirms product data-path health only. SITL and planner
  feasibility remain open and must be rerun after the public-boundary phase
  reaches a stable checkpoint.

### 2026-08-26 - Product planner facade and installed-header fence

- Owner: `navigation_planning_backend::PlannerFacade` and the
  `navigation_runtime::NavigationRuntimeNode` planning boundary. Scope: keep
  planner implementation ownership behind a PImpl facade, expose only product
  status/candidate/trajectory/diagnostic types to runtime, preserve role and
  completion semantics while sampling, and carry both the pinned and validated
  WorldModel identities through the committed snapshot. The watchdog now reads
  stage/point counters directly and cannot reset module timing through a
  diagnostic read.
- Safety impact: ownership and observability refactor with explicit provenance;
  no optimizer objective, dynamic limit, UNKNOWN policy, deadline, continuity
  rule, candidate authorization, emergency gate, PX4 gate, or fallback behavior
  was relaxed. Failed/empty snapshots remain fail-closed. No bypass or
  test-only product behavior was added.
- Evidence: Release build completed for
  `navigation_planning`, `navigation_planning_backend`, and
  `navigation_runtime`; focused CTest passed planning `1/1`, backend `3/3`
  (including the facade boundary test), and runtime `8/8`. A fresh external
  install `/tmp/uav-navigation-facade-install-Zogb2R` installed only
  `navigation_planning_backend/planner_facade.hpp`; a C++20 `-Wall -Wextra
  -Werror` public-header compile and clean consumer link both passed. The
  runtime WorldSnapshotStore test no longer includes planner implementation
  headers.
- Removal condition: none for the facade contract. The remaining internal
  planner vocabulary and vendor implementation headers are intentionally
  quarantined for the next naming/ownership phase; do not expose them again
  through an install rule or runtime public header. Dataset/SITL evidence is
  still required and has not been run on this uncommitted boundary.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && python3 tools/runtime/build.py --mode release build
  --packages navigation_planning navigation_planning_backend
  navigation_runtime && python3 tools/runtime/build.py --mode release test
  --packages navigation_planning navigation_planning_backend navigation_runtime`.

### 2026-08-26 - Planner facade checkpoint parallel validation

- Owner: validation harness only; no product source change. Scope: run the
  recorded-data and PX4/SITL sidecars against exact commit `937c8ef` after the
  facade checkpoint, and keep dataset evidence separate from flight acceptance.
- Safety impact: evidence collection only. No gate, threshold, timeout,
  fallback, UNKNOWN policy, preflight rule, or bypass was changed. The SITL
  sidecar was fail-closed before launch because the canonical runtime lock was
  held by the dataset runner.
- Evidence: dataset artifact
  `.artifacts/runtime/dataset-20260826T001126-569319/report.json` plus
  `dataset_shadow_planning.json` recorded complete IMU/LiDAR source coverage,
  LIO tracking coverage `1.0`, propagated odometry near `50 Hz` with maximum
  observed gap about `25.98 ms`, mapping `2756/2756` received/published with
  `0` accounting violations, and shadow planning `98 READY/0 EMER` across
  generations `1..9`. `experimental_bypasses={}` and
  `flight_acceptance=false` remain explicit. The SITL sidecar reported
  `BLOCKED_ENVIRONMENT` with no session/report and therefore no PX4, planner,
  odometry, or mission evidence.
- Cleanup: both agents were closed; the dataset runner initially remained in
  `STARTING` after replay data had completed, so it was shut down only after
  the report existed and its recorded PIDs were checked. Final state was
  `STOPPED/cleanup=PASS`, no validation process remained, the stale lock was
  removed only after its owner PID was dead, and one worktree remained.
- Removal condition: repeat a preflight-clean SITL run and representative
  dataset distribution before making any end-to-end or flight-acceptance claim.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && python3 tools/runtime/build.py --mode release check`.

### 2026-08-26 - Parallel dataset/SITL runtime lock boundary

- Owner: `tools/runtime/runner.py` and
  `tools/runtime/runtime_environment.py`. Scope: allow read-only dataset and
  SITL sessions to use the same verified Release install concurrently, while
  keeping every build/test/check exclusive. Dataset and SITL have separate
  session locks and separate ROS domains so their ROS graphs cannot exchange
  sensor, clock, planner, or PX4 messages.
- Safety impact: orchestration and observability only. No planner objective,
  dynamic limit, freshness gate, PX4 preflight rule, mission timeout, UNKNOWN
  policy, fallback, or acceptance threshold changed. A build is still blocked
  while any runtime reader holds the canonical install lock.
- Evidence: the previous facade checkpoint artifact
  `.artifacts/runtime/dataset-20260826T001126-569319` reached recorded-data
  health but held `.build-runtime.lock` while the SITL sidecar was rejected
  before session creation. Runtime contract tests passed `144/144` after the
  lock/isolation change, including shared-reader/exclusive-build and
  workflow-aware lock checks. A new parallel run is the removal evidence for
  the starvation incident; it must record session creation for both agents
  before any product verdict is considered.
- Removal condition: replace this compatibility boundary only when runtime
  sessions no longer share the canonical install or when a stronger explicit
  multi-install isolation contract is adopted. Until then, do not revert
  runtime readers to an exclusive build lock.
- Verification command: `env -u VIRTUAL_ENV -u PYTHONHOME /usr/bin/python3
  -m unittest tools/runtime/tests/test_runtime_contract.py`.

### 2026-08-26 - Preserve the guide-time seed before hard-gated optimization

- Owner: `ExpTrajOpt` guide-time initialization in
  `navigation_planning_backend`. Scope: remove the unexplained `0.8`
  contraction from both the implementation and the inline compatibility path;
  preserve the segment durations produced by the guide allocation.
- Safety impact: the former contraction could make the initial candidate
  violate the configured velocity/acceleration envelope before optimization.
  The correction removes an unsafe seed transformation. It does not change
  dynamic limits, retry budgets, freshness gates, fallback authorization,
  UNKNOWN handling, or acceptance thresholds. The hard feasibility gates
  remain the authority.
- Evidence: the pre-fix SITL artifact
  `.artifacts/runtime/external-mode-check-20260826T004800-594297` recorded
  normalized velocity violation `1.0247871740370653` and MINCO line-search
  exhaustion. After the correction, Release build, selected CTest, and
  runtime contracts `173/173` passed; the exact full dataset artifact
  `.artifacts/runtime/dataset-20260826T034428-4` passed with complete
  `55435/55435` IMU and `2772/2772` LiDAR coverage and zero accounting
  violations. A fresh SITL mission result is still open.
- Performance note: the same clean dataset replay measured planner
  `planning_latency` p95 `2.343 ms` and `exp_opt` mean `641 us`, above the
  two earlier clean baselines (`1.389/1.410 ms` and `472/380 us`), while
  mapping and whole-replay throughput did not show a material regression.
  This is an optimization follow-up, not permission to restore the magic
  contraction or relax a safety gate.
- Removal condition: none for the safe seed correction. Close the performance
  follow-up only after repeated same-scenario A/B distributions and a fresh
  SITL run demonstrate that any chosen optimization preserves feasibility,
  continuity, clearance, PX4 state, and fail-closed behavior.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && make build && make test && make dataset-check
  DATASET=aist-mid360-drive RATE=1.0`.

### 2026-08-26 - Do not count carried planner timings as new execution

- Owner: `tools/runtime/planner_trace.py`, `tools/runtime/report.py`, and
  `tools/runtime/html_report.py`. Scope: execution-specific planner timing
  fields (`exp_frontend_us`, `exp_opt_us`, `backup_frontend_us`,
  `backup_opt_us`) are valid for a cycle only when the producer's explicit
  `exp_diagnostics_valid` flag is true. Legacy records without that flag stay
  compatible; planner total and scheduling fields are not filtered by this
  rule.
- Safety impact: observability/reporting only. The change prevents stale
  telemetry from distorting performance decisions; it does not authorize or
  reject commands and does not change any planner objective, dynamic limit,
  deadline, freshness gate, fallback, UNKNOWN policy, or acceptance threshold.
- Evidence: current replay samples contained 9 valid optimizer solves followed
  by setup-only cycles carrying the previous `exp_opt_us` value. Added a
  regression test proving false validity excludes carried execution timings.
  Corrected reports for `.artifacts/runtime/dataset-20260826T041003-4` and
  `.artifacts/runtime/dataset-20260826T041557-4` each contain 9 valid solves;
  both replay verdicts are `PASS` with full source coverage and no experimental
  bypasses.
- Performance characterization: the corrected replay pair achieved
  `0.985925x/0.986787x` source-to-wall playback, above the earlier
  `0.983289x/0.985057x` baselines. Valid `exp_opt` means were `742/747 us`
  versus `603/638 us` in the two valid baseline samples. Keep this as an open
  optimizer-phase investigation; it is not a whole-replay regression or a
  safety acceptance failure.
- Removal condition: none while the runtime diagnostic contract can carry
  forward values. Revisit only when the producer emits an explicit per-field
  execution identity and the report can consume that stronger contract.
- Verification command: `env -u VIRTUAL_ENV -u PYTHONHOME /usr/bin/python3
  -m unittest tools/runtime/tests/test_runtime_contract.py`.

### 2026-08-26 - Remove inactive duplicate EXP implementations

- Owner: `navigation_planning_backend::ExpTrajOpt`. Scope: remove the unused
  inline `processCorridorWithGuideTraj2()` and `setupProblemAndCheck2()` clones,
  plus the unreachable default-initialization helpers behind the active
  fail-closed path. The production call graph retains only
  `processCorridorWithGuideTraj()` and `setupProblemAndCheck()`.
- Safety impact: source/ownership refactor only. No optimizer objective,
  guide-time allocation, dynamic limit, hard gate, retry budget, cancellation,
  fallback, UNKNOWN policy, candidate authorization, or PX4 gate changed.
  The active guide allocation remains the sole duration seed; no compatibility
  implementation is kept as a second product path.
- Evidence: call-graph review found no production consumer of either `*2`
  function; the duplicate path also lacked active overlap/guide matching and
  feasibility-reference behavior. The port manifest hashes were updated for
  the reviewed deletion. Component tests and a direct EXP seed-duration
  regression are required before this slice is considered stable; SITL remains
  a separate acceptance gate.
- Removal condition: none. Reintroducing a second implementation is prohibited;
  any future algorithmic change must modify the single active path and update
  the parity manifest/provenance review.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && make build && make test`; additionally run the planner
  parity checker when the pinned upstream checkout is available.

### 2026-08-26 - Make planner and execution exports describe their real boundary

- Owner: `navigation_planning_backend` and `navigation_execution` CMake/export
  contracts. Scope: mark backend-only fmt/ROG/YAML/std_msgs dependencies
  private, keep only Eigen/planning/world-model dependencies in the backend
  public link interface, and make the execution interface explicitly link the
  product planning/world-model targets. `navigation_contracts` is test-only for
  execution and is no longer exported by that package.
- Safety impact: packaging/ABI boundary only. No runtime source, command gate,
  planner objective, dynamic limit, deadline, fallback, UNKNOWN policy, or
  acceptance threshold changed. The change is intended to prevent consumers
  from accidentally compiling against implementation/vendor headers.
- Evidence: current source inspection showed the facade header includes only
  Eigen, `navigation_planning`, and `navigation_world_model`; execution
  production headers use the latter two but not ROS contract headers. A fresh
  install inventory and external C++20 consumer compile are required before
  this boundary is stable; the existing canonical install contains stale
  generated headers and is not evidence.
- Removal condition: none. Re-export a dependency only if a public installed
  header requires it and add a compile-fence test proving that requirement.
- Verification command: configure/build into a fresh temporary prefix, assert
  the installed header allowlist, compile one translation unit per public
  header with `-Wall -Wextra -Werror`, and run the backend/execution/runtime
  focused CTest targets.

### 2026-08-26 - Preserve direct evidence for the single EXP seed path

- Owner: `navigation_planning_backend::ExpTrajOpt` regression coverage. Scope:
  exercise the active guide overload through a convex corridor and assert that
  the diagnostic initial duration equals the guide's terminal timestamp.
- Safety impact: test-only observability. The test does not alter optimizer
  configuration, feasibility gates, retry behavior, dynamic limits, fallback,
  or runtime acceptance.
- Evidence: `test_exp_optimizer_seed` passed in the sourced ROS/workspace
  environment; the full selected test command also passed with 69 C++ tests
  and 173 Python tests. The direct run measured a successful finite trajectory
  and `initial_duration_s == guide_t.back()`.
- Removal condition: none while the guide allocation remains the sole active
  duration seed. Any future seed change must update this invariant and its
  safety rationale.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && build/navigation_planning_backend/
  test_exp_optimizer_seed --gtest_color=no`.

### 2026-08-26 - Fresh backend install has a narrow public header surface

- Owner: `navigation_planning_backend` install/export contract. Scope: verify
  the package in an empty temporary install prefix, not the accumulated
  workspace install.
- Safety impact: packaging/compile-boundary only. No runtime behavior,
  planner objective, deadline, gate, fallback, or acceptance threshold changed.
- Evidence: a fresh backend prefix installed exactly
  `include/navigation_planning_backend/planner_facade.hpp`; no `super*`,
  `mars*`, optimizer implementation, or private config headers were installed.
  A C++20 external translation unit including only the facade compiled with
  `-fsyntax-only` successfully. A fresh execution-only attempt was not
  accepted because its dependency package hooks were absent from that prefix;
  execution still requires a fresh multi-package consumer check.
- Removal condition: none. Keep the explicit public-header allowlist and close
  the execution check only after its dependencies are installed into the same
  clean prefix.
- Verification command: `colcon build --packages-select
  navigation_planning_backend --build-base <fresh>/build --install-base
  <fresh>/install --merge-install`, then inventory the installed include tree
  and compile the facade with C++20.

### 2026-08-26 - Independent corridor certificate and route-reference objective

- Owner: `navigation_planning_backend::ExpTrajOpt` objective and acceptance
  boundary. Scope: replace the derived `feasibility_point_weight` with the
  explicit `route_reference/{lateral,vertical}_weight` and deadband terms.
  Route-reference quality cost is evaluated independently from corridor and
  dynamic feasibility penalties; retries may scale only the configured
  feasibility penalties. The hard corridor certificate now recomputes the
  maximum sampled half-space violation from the trajectory and corridor
  planes, so `objective.position_penalty_weight` cannot disable or authorize that certificate.
- Safety impact: removes a conditioning and ownership coupling that could make
  a quality term inherit `5e8` from the position penalty, and closes the prior
  non-positive position-penalty certificate bypass. A malformed or non-finite route
  reference is rejected rather than replaced with the candidate itself. The
  route weights/deadbands are quality parameters only; they do not certify
  clearance or flight safety. The sampled corridor certificate remains
  provisional until continuous-extrema evidence is added.
- Evidence: product YAML declares all four route-reference fields; nominal
  config requires them, validates finite positive optimizer accuracy and
  integral resolution, and tests assert the loaded values. The post-correction
  focused build passed `navigation_planning_backend`, `test_exp_optimizer_seed`,
  `test_trajectory`, `test_planner_config`, `test_planner_runtime_context`,
  and `test_planner_facade`; the focused tests passed 1/1, 34/34, 11/11,
  3/3, and 1/1 respectively. The runtime contract suite passed 145/145.
  Objective conditioning A/B and representative dataset/SITL evidence remain
  open.
- Removal condition: none for independent ownership and fail-closed
  validation. Revisit the provisional YAML quality values only after repeated
  same-scenario objective decomposition, gradient/iteration, latency and
  trajectory-quality A/B distributions; never tune a hard gate from one run.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/navigation_planning_backend --target navigation_planning_backend
  test_exp_optimizer_seed test_trajectory test_planner_config
  test_planner_runtime_context -j1`, followed by the sourced focused tests and
  representative dataset/SITL validation.

### 2026-08-26 - Role-preserving known-free backup certificate

- Owner: planner authorization and committed-trajectory revalidation. Scope:
  apply the mission UNKNOWN policy to MAIN only; BACKUP always requires
  `kRequireKnownFree` for endpoint samples and swept segments. A candidate
  using UNKNOWN for MAIN must contain a complete finite role partition with a
  BACKUP suffix reaching the trajectory end. Revalidation copies the committed
  role intervals before sweeping the latest world, so the runtime cannot turn
  a BACKUP suffix into MAIN by reconstructing only position/yaw.
- Safety impact: closes the predicate weakness where an atomic transaction
  could commit or later revalidate a candidate through UNKNOWN without a real
  backup certificate. Malformed, gapped or incomplete role schedules fail
  closed. This can increase false rejects in partially observed maps; that is
  intentional and must be characterized rather than bypassed. The certificate
  is still sampled swept evidence, not a continuous polynomial proof, and no
  hardware-flight claim is made.
- Evidence: backup visibility uses known-free segment traversal; live and
  immutable snapshot traversal explicitly check start, intermediate cells and
  endpoint. Added role-policy tests cover MAIN UNKNOWN plus BACKUP UNKNOWN,
  backup-suffix coverage and malformed-role rejection. The post-change
  `test_trajectory` suite passed 34/34 and `test_planner_facade` passed 1/1;
  runtime revalidation, repeated replay and SITL evidence remain open.
- Removal condition: none. Keep the role partition and known-free predicate
  as the sole authorization contract. Strengthen only by adding evidence or a
  formally stronger certificate; never restore UNKNOWN acceptance for BACKUP
  to improve planning rate.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/navigation_planning_backend --target navigation_planning_backend
  test_trajectory -j1 && build/navigation_planning_backend/test_trajectory
  --gtest_color=no`; then run the runtime revalidation, dataset and SITL
  scenarios with full artifact correlation.

### 2026-08-26 - Planner solve failure classification

- Owner: `navigation_planning_backend::Planner` result contract. Scope:
  preserve cancellation and deadline precedence while allowing each caller to
  supply the non-budget fallback: main trajectory generation maps to
  `PLANNER_EXP_FAILED`, candidate construction/world authorization maps to
  `PLANNER_CANDIDATE_REJECTED`, and actual backup-stage failure remains
  `PLANNER_BACKUP_FAILED`.
- Safety impact: diagnostics and error ownership only. No failure is converted
  into success, no fallback is weakened, and the previously committed command
  remains unchanged on rejection. More precise codes are required to prevent
  a corridor, optimizer, certificate or candidate error being misdiagnosed as
  a backup-generation failure.
- Evidence: enum/string regression coverage was extended; the post-change
  `test_trajectory` suite passed 34/34 and the planner backend target rebuilt
  successfully. Runtime replay and SITL evidence remain separate acceptance
  gates.
- Removal condition: retain the distinct codes until the product diagnostic
  schema carries structured source/reason fields and all runtime consumers use
  that schema.
- Verification command: `source /opt/ros/jazzy/setup.bash && cmake --build
  build/navigation_planning_backend --target navigation_planning_backend
  test_trajectory -j1 && build/navigation_planning_backend/test_trajectory
  --gtest_color=no`.

### 2026-08-26 - Candidate mission identity is immutable across export and sampling

- Owner: `navigation_planning_backend::Planner` and
  `navigation_execution::CommandSampler`. Scope: persist localization epoch,
  goal epoch and request id on the backend committed trajectory, require the
  runtime to set that identity before solving, reject export under a different
  identity, and reject a retained bundle when the active goal epoch has
  advanced.
- Safety impact: closes a provenance flaw where a trajectory planned for an
  older hot-retarget goal could be exported with newer goal metadata. A
  retained bundle now becomes unavailable until a candidate for the new goal
  is committed; this may create a command gap and is intentionally fail-closed.
  No visibility predicate, timeout, fallback or acceptance threshold was
  relaxed.
- Evidence: backend/facade compilation passed after the identity fields were
  added; `test_planner_facade` passed 1/1; `test_committed_bundle_store` passed
  5/5; `navigation_runtime_node` relinked successfully against the refreshed
  backend install. Full runtime, dataset and SITL evidence remains open.
- Removal condition: none. Keep identity attached to the candidate until the
  planner no longer owns any internal trajectory representation and the
  execution coordinator receives a native immutable candidate directly.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && cmake --build build/navigation_planning_backend
  --target install -j2 && cmake --build build/navigation_runtime --target
  navigation_runtime_node -j2 && build/navigation_execution/
  test_committed_bundle_store --gtest_color=no`.

### 2026-08-26 - Execution store is the sole exposed candidate commit

- Owner: planner/execution boundary. Scope: the planner now authorizes and
  stages a candidate with its world and mission certificate; runtime exports
  that pending candidate and commits it through `CommittedBundleStore`. The
  planner receives an ACK only after the execution-store swap and updates
  `CmdTraj` as replanning history. The sampler never reads `CmdTraj`.
- Safety impact: removes the second command commit path and prevents runtime
  readiness from being inferred from a backend generation increment. A failed
  execution commit discards the pending candidate; an ACK mismatch clears the
  execution store and keeps exposure fail-closed. This is an ownership change,
  not a relaxation of candidate/world/backup predicates.
- Evidence: backend, facade and runtime targets rebuilt after the cutover;
  `test_planner_config` passed 11/11, `test_trajectory` passed 34/34,
  `test_planner_facade` passed 1/1, `test_committed_bundle_store` passed 5/5,
  and PX4 odometry bridge CTest passed 6/6 after the related parameter cleanup.
  Full runtime integration, dataset and SITL evidence remain open.
- Removal condition: none. Remove the transitional backend trajectory history
  only after the planner consumes an explicit immutable previous-candidate
  input and all retained-command validation reads execution-owned state.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && cmake --build build/navigation_planning_backend
  --target install -j2 && cmake --build build/navigation_runtime --target
  navigation_runtime_node -j2 && build/navigation_planning_backend/
  test_trajectory --gtest_color=no && build/navigation_execution/
  test_committed_bundle_store --gtest_color=no`.

### 2026-08-26 - Unified command anchor safety contract

- Owner: `navigation_contracts::kCommandAnchorErrorLimitM`. Scope: runtime
  retained-command validation and PX4 command acceptance use the same named
  geometric limit; the PX4-local `command_anchor_max_error_m` copy and the
  speed-dependent `command_tracking_lag_s` behavior parameter are removed.
- Safety impact: removes a second authority and prevents forward tracking delay
  from enlarging the command acceptance envelope. A command must remain within
  the finite geometric limit in forward, reverse and lateral directions.
  This can increase command rejection during genuine tracking lag; it is
  intentionally fail-closed and is not a SITL tuning result.
- Evidence: focused tracking-envelope tests were updated to cover strict
  forward/reverse/lateral limits; runtime and PX4 builds are pending after the
  contract change. Repeated SITL and recorded-data distributions are required
  before changing this product constant.
- Removal condition: none. Change the constant only with repeated
  representative tracking evidence and a reviewed safety decision; do not
  reintroduce node-local copies or a lag allowance parameter.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && cmake --build build/px4_navigation_external_mode
  --target px4_navigation_external_mode_core test_tracking_envelope -j2 &&
  build/px4_navigation_external_mode/test_tracking_envelope
  --gtest_color=no && cmake --build build/navigation_runtime --target
  navigation_runtime_node -j2`.

### 2026-08-26 - User-facing duration units are seconds at ROS boundaries

- Owner: `navigation_common::secondsToNanoseconds()` and estimator/PX4
  parameter loaders. Scope: user-facing YAML/ROS duration keys now use `_s`;
  conversion to integer nanoseconds occurs once before entering estimator,
  odometry and point-time core contracts.
- Safety impact: removes ambiguous raw integer duration parameters and prevents
  accidental ns/ms unit mistakes. Existing numeric durations and all freshness,
  continuity, timestamp and propagation gates are preserved exactly after
  conversion; no timeout or grace period was relaxed.
- Evidence: FAST-LIO CTest passed 12/12, PX4 odometry bridge CTest passed 6/6,
  and focused parameter-loader tests passed 20/20 with ROS logs redirected to a
  writable test directory. SITL and representative replay evidence remain
  required for runtime timing claims.
- Removal condition: none. New duration parameters must use seconds in the
  profile and a named unit-suffixed internal field when integer timestamps are
  required.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ROS_LOG_DIR=/tmp/uav-navigation-test-ros-logs ctest
  --test-dir build/fast_lio_ros --output-on-failure && ctest --test-dir
  build/px4_odometry_bridge --output-on-failure`.

### 2026-08-26 - Planner collision radius is derived from owned budgets

- Owner: planner configuration safety-envelope composition. Scope: remove the
  independent `planner/robot_r` YAML value and derive the collision radius from
  vehicle, tracking, localization, mapping and planning-margin budgets before
  computing corridor/braking geometry.
- Safety impact: prevents a stale radius parameter from silently disagreeing
  with its component budgets. The derived radius is still validated and all
  downstream collision/corridor gates keep the same value for the current
  profile; no margin was reduced.
- Evidence: configuration/trajectory focused tests must be rerun after the
  exact YAML/config change; SITL and recorded-data evidence remain open.
- Removal condition: none. Keep the component budgets as the only owners; do
  not reintroduce an independently tunable aggregate radius.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && cmake --build build/navigation_planning_backend
  --target test_planner_config -j2 && build/navigation_planning_backend/
  test_planner_config --gtest_color=no`.

### 2026-08-26 - Mission configuration contains only consumed behavior

- Owner: mission schema and runtime scheduling boundary. Scope: remove
  `planning.replan_rate_hz` and `control.pass_through_lookahead_m`, which were
  parsed and serialized in profiles but had no behavior consumer.
- Safety impact: no gate, planner cadence, waypoint acceptance rule, or grace
  interval is weakened. Runtime remains the owner of planner cadence and the
  mission controller remains the owner of waypoint acceptance.
- Evidence: strict parser, mission struct, tests and product mission profiles
  were updated; fresh mission-profile validation is still required.
- Removal condition: none. A new mission behavior field requires a consumer,
  unit contract and negative test before it may enter the schema.
- Verification command: `rg -n "replan_rate_hz|pass_through_lookahead_m"
  src config/runtime/missions` must return no product matches, followed by the
  PX4 mission CTest suite.

### 2026-08-26 - Clock-domain selection remains explicit and fail-closed

- Owner: launch ROS clock plus estimator measurement-domain contract. Scope:
  `use_sim_time` selects the ROS node clock (`/clock` for SITL/replay, system
  clock for realtime); `fast_lio.timing.clock_domain` describes incoming
  measurement stamps (`simulation_time`, `sensor_time`, or `ros_time`). The
  removed bridge flags did not control either clock and were not replaced by a
  hidden boolean.
- Safety impact: no implicit fallback from realtime to simulation timestamps
  is allowed. The external odometry converter authorizes `ros_time` or
  `system_time` only when `use_sim_time=false`; PX4's uXRCE-DDS transport owns
  the ROS-to-PX4 boot-time offset after its own convergence gate. Sensor-time
  input remains fail-closed until a sensor-to-ROS adapter exists.
- Evidence: launch files pass `use_sim_time`; SITL uses
  `timing.clock_domain: simulation_time`, dataset uses `sensor_time`, and the
  converter rejects mismatched domains. The official uXRCE-DDS implementation
  passes its session offset into timestamp serialization/deserialization and
  waits for convergence before exposing topics; `TimesyncStatus` remains
  diagnostic evidence, not a duplicate product mapping authority. Repeated
  realtime and SITL validation remain open.
- Removal condition: verify the source stamp domain on target hardware,
  confirm the uXRCE-DDS session convergence and timestamp shift in captured
  messages, then repeat realtime recorded-data plus SITL validation without
  relaxing freshness gates. If a transport variant does not provide this
  contract, retain fail-closed behavior rather than adding a local offset.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ctest --test-dir build/px4_odometry_bridge
  --output-on-failure` plus dedicated mapping-convergence tests.

### 2026-08-26 - A* search workspace follows the pinned world geometry

- Owner: planner frontend search workspace. Scope: remove the independent
  `astar/map_voxel_num` geometry parameter, derive the local index extents from
  the pinned `WorldGeometry`, and store only touched nodes for the current
  search.
- Safety impact: the search remains bounded by the actual immutable map window;
  no unknown-space policy, collision query, diagonal-edge check, search
  deadline, or fallback order is relaxed. Non-finite, non-positive, or
  non-integral geometry is rejected before a path can be returned.
- Evidence: focused build and CTest are required after the change. The
  previous implementation reserved approximately 25 million pointer slots
  from a second YAML geometry, while the new workspace has no eager map-sized
  allocation. Repeated planner latency/RSS and dataset/SITL distributions
  remain open; this entry is not a performance acceptance claim.
- Removal condition: none. Future search-window changes must come from the
  world-model geometry or a named planner horizon contract, never a duplicate
  voxel-count parameter.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest
  --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - C++ mission schema had multiple parsing owners

- Owner: product mission contract. Scope: moved the validated mission schema
  and YAML loader to `navigation_mission`; PX4 External Mode and runtime now
  consume that one implementation. Runtime also checks the configured planning
  frame through the common loader before constructing the planner, and the
  loader returns the world-model UNKNOWN enum rather than a raw policy string.
- Safety impact: removes drift between PX4 mission behavior and runtime
  planner limits, including waypoint validation, frame validation, dynamic
  limits and UNKNOWN policy. No limit, policy or acceptance threshold was
  relaxed.
- Evidence: added the shared package and contract tests; the new package, PX4
  External Mode (3/3) and runtime (7/7) tests pass after the migration.
- Removal condition: none. Python runner/report remain non-flight tooling
  readers and must not become a second product authority.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mission
  px4_navigation_external_mode navigation_runtime
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_mission --output-on-failure`.

### 2026-08-27 - Revalidation bypassed the main-only certificate policy

- Owner: planner facade trajectory revalidation. Scope: revalidation now uses
  the same role-aware `candidateCertificatePolicy()` as initial candidate
  authorization. A main-only candidate is known-free even when the mission
  permits UNKNOWN; only a candidate with a complete BACKUP suffix may apply the
  mission policy to MAIN, while BACKUP remains known-free.
- Safety impact: closes a P0 certificate gap where a newer-world revalidation
  could have re-authorized an old main-only trajectory through UNKNOWN. No
  mission policy was broadened, and OCCUPIED/OUT_OF_MAP plus swept-volume
  checks remain fail-closed.
- Evidence: adversarial review traced the mismatch from facade revalidation to
  runtime bundle retention. The fix reuses the existing resolver; focused
  planning/facade tests and runtime revalidation coverage are required before
  this entry can be closed.
- Removal condition: none. Any future candidate role must select its
  certificate policy through the shared resolver at both initial authorization
  and revalidation boundaries.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && cmake --build build/navigation_planning_backend
  --target navigation_planning_backend test_trajectory test_planner_facade -j2
  && ctest --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Continuous certificate rejected off-tube cells

- Owner: executable trajectory swept-volume validator. Scope: the continuous
  certificate now enumerates the conservative voxel neighborhood around the
  curve/chord tube and filters each cell by its distance to the chord. It no
  longer treats the whole axis-aligned bounding box as occupied by the
  trajectory.
- Safety impact: removes a false-reject source for diagonal and curved paths;
  every voxel that can intersect the bounded curve tube is still checked, and
  OCCUPIED/OUT_OF_MAP plus BACKUP known-free semantics remain fail-closed.
- Evidence: added an off-tube occupied-cell regression alongside the existing
  curve-through-occupied-cell regression; navigation planning CTest passed 5/5.
  Full body/tracking/localization/mapping certificate and runtime distributions
  remain open.
- Removal condition: none. Any faster rasterization must cover the same
  bounded tube or provide a stronger independent certificate.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest
  --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - SITL external odometry bridge declares its simulation clock domain

- Owner: PX4 external odometry bridge time boundary. Scope: the SITL profile now
  declares `timing.clock_domain: simulation_time` for the external odometry
  bridge, matching the launcher's `use_sim_time:=true`. The bridge remains
  responsible only for converting the ROS/simulation timestamp at its declared
  boundary; PX4 uXRCE-DDS session synchronization is not duplicated locally.
- Safety impact: prevents a startup configuration mismatch from leaving the
  bridge unresolved or encouraging an unsafe fallback to wall time. This does
  not relax freshness, continuity, timestamp, or PX4 preflight gates.
- Evidence: the bridge mapping implementation rejects mismatched clock-domain
  and `use_sim_time` combinations; the runtime profile contract test now checks
  the explicit simulation setting. Full SITL startup, timestamp continuity,
  estimator freshness, and repeated scenario evidence remain open.
- Removal condition: none. A different simulator clock contract must provide a
  typed profile and matching launch/test evidence; it must not rely on the
  bridge default.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && python3 -m unittest
  tools.runtime.tests.test_runtime_contract && colcon build --packages-select
  px4_odometry_bridge navigation_runtime --cmake-args -DBUILD_TESTING=ON`.

### 2026-08-27 - Visibility horizon configuration has explicit ownership

- Owner: planner visibility-horizon configuration. Scope: replace the
  overloaded `safe_corridor_line_*` fields with a configured cap, configured
  floor, and derived effective horizon. Replace the negative `sensing_horizon`
  sentinel with `sensing_horizon_m: 0.0` for an explicitly disabled optional FOV
  cut. The effective horizon is never written back into the configured cap.
- Safety impact: preserves the existing 23 m cap, 14 m floor, braking/replan
  validation, and FOV-disabled behavior while preventing an invalid negative
  physical distance or a hidden cap mutation. No visibility, unknown-space, or
  backup gate was relaxed.
- Evidence: planner config tests now assert the derived 14 m effective horizon;
  runtime report fixtures and profile keys use the new unit-explicit names.
  Build/CTest plus profile/report tests are required below.
- Removal condition: none. Any future horizon source must declare whether it is
  a cap, a floor, or a derived value and must retain the fail-closed envelope
  check.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && python3 -m unittest
  tools.runtime.tests.test_runtime_contract tools.runtime.tests.test_html_report
  && colcon build --packages-select navigation_planning_backend navigation_runtime
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Executable trajectory validation bounds polynomial curve deviation

- Owner: mapping-to-execution trajectory certificate. Scope: each validated
  polynomial segment now derives a conservative continuous acceleration bound
  from its coefficients, adaptively subdivides until the curve-to-chord bound
  is below a quarter of the inflated voxel resolution, and checks the
  resulting axis-aligned tube cells before accepting the chord. Tube size and
  cell-query work are bounded; an invalid or oversized certificate fails
  closed.
- Safety impact: closes the previous chord/sample-only blind spot for curve
  bowing between samples. OCCUPIED and OUT_OF_MAP remain failures for MAIN and
  BACKUP; BACKUP still additionally requires KNOWN_FREE for every tube cell.
  This is conservative and may reduce feasibility near unknown boundaries; no
  unknown-space permission or numerical gate was relaxed.
- Evidence: added a regression with a curved polynomial crossing an occupied
  voxel while sampled points remain away from its center. Planning CTest passed
  5/5 after the change. Full optimizer integration, mapping implementation
  parity, runtime latency distributions, dataset, and repeated SITL evidence
  remain open.
- Removal condition: none. Any optimization must preserve a proved geometric
  bound and an equivalent or stronger tube query; replacing it with fewer
  samples requires a new certificate and adversarial test.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend navigation_runtime --cmake-args -DBUILD_TESTING=ON
  && ctest --test-dir build/navigation_planning_backend --output-on-failure
  && ctest --test-dir build/navigation_runtime --output-on-failure`.

### 2026-08-27 - Planner unknown-space policy has one typed mission boundary

- Owner: planner mission-policy boundary. Scope: remove the product YAML
  `frontend_in_known_free` boolean and the unused planner-level
  `visual_process` setting, including the runner's generated ghost key. The
  planner stores a typed world-model unknown-space policy; when a mission
  contract is supplied, that contract is the sole authority for exploratory A*
  and corridor policy. A* visualization remains under its own search owner.
- Safety impact: removes a second policy knob that could drift from mission
  `unknown_policy`. MAIN continues to use the mission policy; BACKUP continues
  to require `KNOWN_FREE` independently. No unknown-space permission is added
  and no safety gate is relaxed.
- Evidence: added a profile contract assertion and compile-time use of the
  typed policy in all A* map flags. Planner and runtime tests/build are the
  verification gate; dataset/SITL policy propagation remains required.
- Removal condition: none. Any future policy must be represented by one typed
  contract and explicitly mapped at the planner boundary.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && python3 -m unittest
  tools.runtime.tests.test_runtime_contract && colcon build --packages-select
  navigation_planning_backend navigation_runtime --cmake-args -DBUILD_TESTING=ON
  && ctest --test-dir build/navigation_planning_backend --output-on-failure
  && ctest --test-dir build/navigation_runtime --output-on-failure`.

### 2026-08-27 - A* uses exact per-layer bounds and stable frontier entries

- Owner: planner frontend search workspace. Scope: A* now derives its index
  domain from the pinned evidence or inflated `GridBounds`, including the
  signed global minimum and asymmetric maximum index, instead of reconstructing
  a centered window around the search point. Frontier queue entries capture
  their score and sequence at enqueue time; stale entries are discarded. An
  exhausted search that reaches its steady deadline reports `TIME_OUT` rather
  than `NO_PATH`.
- Safety impact: prevents search nodes, endpoint projections, and returned
  frontier paths from escaping the immutable layer storage. It also preserves
  deadline truth for the caller, so a later fallback cannot mistake an
  incomplete search for a proved geometric no-path. Unknown-space policy,
  diagonal-edge traversal, and backup known-free requirements are unchanged.
- Evidence: exact-bound and frontier-order source/tests build successfully;
  the planning backend CTest passes 5/5 and the runtime CTest passes 7/7 for
  ten consecutive repetitions. Dataset distributions, sparse-workspace RSS,
  and repeated SITL evidence remain open; this entry is not a performance or
  flight-acceptance claim.
- Removal condition: none. Any future local horizon must be an explicit
  planner contract intersected with these world bounds, never a second map
  geometry definition.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_world_model navigation_mapping navigation_planning_backend
  navigation_runtime --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure && ctest --test-dir
  build/navigation_runtime --repeat until-fail:10 --output-on-failure`.

### 2026-08-27 - Runtime parameter materialization preserves map evidence

- Owner: runtime harness and canonical planner profile. Scope:
  `_mapping_params()` no longer overwrites `rog_map.raycasting.enable` with
  `false`; the generated session planner file preserves the product profile's
  sensor-origin raycasting setting. Mission unknown policy remains a separate
  planner input and does not disable evidence production.
- Safety impact: removes a hidden configuration contradiction that made BACKUP
  known-free certification impossible in generated runtime sessions. No
  unknown-space gate is relaxed; MAIN may still follow its explicit mission
  policy and BACKUP still requires known-free evidence and inflated occupancy.
  Raycasting can increase map callback cost, so this change is not a
  performance acceptance claim.
- Evidence: runtime contract tests now assert that both blocked and
  allow-unknown mission materializations retain raycasting enabled. Dense
  replay callback p50/p95/p99, RSS, and repeated SITL evidence remain required.
- Removal condition: only after an independently verified replacement for
  sensor-origin known-free evidence is installed and recorded here.
- Verification command: `python3 -m unittest tools.runtime.tests.test_runtime_contract`
  plus the dataset and SITL validation commands after the next stable build.

### 2026-08-27 - Mapping probability configuration has one read per key

- Owner: mapping configuration loader. Scope: remove the duplicate
  `raycasting/p_free` load so one YAML key has one load site and one default;
  the parsed value and mapping algorithm are otherwise unchanged.
- Safety impact: none to the selected value or occupancy semantics. The change
  removes ambiguity that could hide a future conflicting duplicate; all
  probability ranges and known-free/occupied policies remain fail-closed.
- Evidence: source audit identifies the duplicate load and the mapping vendor
  build/tests are required below. Runtime/dataset/SITL evidence is unchanged.
- Removal condition: none.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select rog_map_vendor
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/rog_map_vendor
  --output-on-failure`.

### 2026-08-27 - Inflated map queries honor their own bounds

- Owner: immutable mapping snapshot query boundary. Scope: classification and
  segment traversal now validate containment against the selected evidence or
  inflated grid layout; a point covered by the base layer but absent from the
  inflated layer is out of map and cannot be treated as free.
- Safety impact: closes a fail-open boundary mismatch that could allow an
  inflated ray query outside its storage to pass under `kAllowUnknown`.
  Unknown-space and known-free requirements remain unchanged.
- Evidence: added a regression test with a shifted inflated layer; the mapping
  world-model build and test command below must pass before checkpointing.
- Removal condition: none. If layers are intentionally resized, their own
  physical bounds must remain authoritative for all layer-specific queries.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_runtime
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_runtime --output-on-failure`.

### 2026-08-27 - Present configuration type errors fail closed

- Owner: shared YAML configuration loading boundary. Scope: a missing optional
  key may still use its declared default, but a present value with an invalid
  scalar/sequence type or an explicit null now raises an error for both
  required and optional parameters.
- Safety impact: prevents malformed deployment parameters from silently
  selecting a different planner, mapping, timing, or vehicle-safety behavior.
  No default value, gate, timeout, or unknown-space policy was relaxed.
- Evidence: added regression coverage for optional type mismatch and present
  null values; package build and YAML-loader tests are required below.
- Removal condition: none. Any future compatibility exception must be isolated
  at an explicit migration boundary and recorded here with its expiry.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select rog_map_vendor
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/rog_map_vendor --output-on-failure`.

### 2026-08-27 - Product mapping emits evidence for backup certification

- Owner: product mapping profile and exploration/backup safety contract. Scope:
  the product planner profile enables sensor-origin raycasting so the mapper
  can produce `KNOWN_FREE` evidence; exploratory MAIN may still use UNKNOWN,
  while every committed BACKUP segment remains fail-closed on known-free
  evidence and inflated occupancy.
- Safety impact: removes the configuration contradiction where backup required
  `KNOWN_FREE` but mapping only inserted occupied endpoints. This increases map
  update work and therefore cannot be treated as a performance improvement or
  flight certification; no unknown-space gate was relaxed.
- Evidence: source/test review confirms endpoint-only mode leaves the ray
  between origin and hit UNKNOWN; product profile now selects the raycasting
  path. Dense-cloud, 1x/2x replay, RSS and p50/p95/p99 mapping/export measures
  remain mandatory before accepting the profile for hardware.
- Removal condition: only if an equivalent, independently verified observed-free
  certificate is implemented. Never disable raycasting while retaining a
  `KNOWN_FREE` backup requirement without an explicit replacement certificate.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_runtime
  rog_map_vendor --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_runtime --output-on-failure && ctest --test-dir
  build/rog_map_vendor --output-on-failure`.

### 2026-08-27 - Command exposure revalidates the execution certificate

- Owner: execution command publication boundary. Scope: a sampled immutable
  bundle is rechecked under the committed-store transaction lock immediately
  before publication against pointer identity, goal epoch, and world identity;
  a missing bundle during world recertification is treated as pending rather
  than a planner terminal failure.
- Safety impact: closes the load-then-publish race that could expose a bundle
  after a newer world invalidated it, and avoids a false terminal latch during
  the intentional recertification gap. No stale command is retained and no
  safety gate is relaxed.
- Evidence: added deterministic store tests for world and retained-goal
  invalidation. Full runtime build/tests and a repeated multi-thread runtime
  exercise remain required; this change is not SITL acceptance evidence.
- Removal condition: none. Any future zero-copy publication path must preserve
  the same lock/identity transaction at the transport exposure boundary.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_execution navigation_runtime --cmake-args -DBUILD_TESTING=ON
  && ctest --test-dir build/navigation_execution --output-on-failure &&
  ctest --test-dir build/navigation_runtime --output-on-failure`.

### 2026-08-27 - World revision uses validated certificate transfer

- Owner: mapping-to-execution publication boundary. Scope: a newer immutable
  world snapshot no longer unconditionally removes a still-safe command. The
  runtime validates the exact currently committed backend trajectory on the
  new snapshot, and the world publication plus certificate transfer are
  serialized under one publication gate. Pointer/generation mismatch,
  validation failure, or an invalid certificate clears exposure.
- Safety impact: restores liveness for map changes outside the remaining
  swept trajectory without retaining an uncertified command. MAIN UNKNOWN
  policy, BACKUP KNOWN_FREE policy, inflated occupancy, and localization/goal
  identity checks are unchanged. This is not permission to assume that a map
  delta is irrelevant; the full remaining executable trajectory is validated.
- Evidence: added committed-bundle recertification and dependent-publication
  ordering tests. The required execution and runtime CTest commands passed
  2/2 and 7/7. End-to-end race, dataset distribution, and repeated SITL
  evidence remain open.
- Removal condition: none. Any future delta-aware optimization must prove an
  equivalent or stronger swept-volume certificate and retain the exact-bundle
  generation check.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_execution navigation_runtime --cmake-args -DBUILD_TESTING=ON
  && ctest --test-dir build/navigation_execution --output-on-failure &&
  ctest --test-dir build/navigation_runtime --output-on-failure`.

### 2026-08-27 - CIRI and corridor generation obey one steady deadline

- Owner: planner corridor frontend. Scope: CIRI geometry and corridor
  diagnostic collection now reject non-finite or degenerate inputs, check the
  shared steady deadline at bounded loop points, and cap retained diagnostic
  point-cloud memory. The planner passes the same absolute deadline to main
  and backup corridor generation.
- Safety impact: prevents NaN/Inf geometry from becoming a candidate and
  prevents diagnostic accumulation or repeated corridor work from escaping
  the solve budget. Timeout returns remain fail-closed; no feasibility gate,
  unknown policy, or backup certificate is relaxed.
- Evidence: `test_trajectory` covers invalid CIRI configuration and the
  navigation planning backend build plus CTest passes 5/5. Deadline coverage
  in a forced long-running corridor, allocator/RSS measurements, dataset, and
  repeated SITL evidence remain open.
- Removal condition: none. Any larger diagnostic limit or additional solver
  work must remain bounded by the same deadline and be justified by measured
  memory/timing distributions.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest
  --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - CIRI rejects a zero-length seed before geometric construction

- Owner: planner corridor seed validation. Scope: CIRI now rejects a non-finite
  or numerically degenerate seed segment before constructing the oriented
  ellipsoid. A zero-length seed cannot define a stable tangent frame or a
  meaningful corridor axis.
- Safety impact: removes an undefined-orientation numerical path. It does not
  relax obstacle, unknown-space, corridor, dynamic, deadline, or backup gates;
  malformed input remains fail-closed as `INIT_ERROR`.
- Evidence: added a direct regression using a finite bounded polytope and a
  zero-length seed; planning backend CTest is required below. Full integration,
  dataset, and repeated SITL evidence remain open.
- Removal condition: none. Any future degenerate-seed policy must preserve a
  deterministic fail-closed result or supply a separately certified axis.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest
  --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Remove the ignored dual-planning runtime switch

- Owner: runtime harness and planner role contract. Scope: remove
  `DUAL_PLANNING`/`--dual-planning`, which was accepted and serialized by the
  runner but discarded before parameter materialization. MAIN plus BACKUP is
  now the single planner contract; the flag cannot create a second behavior
  mode.
- Safety impact: removes an inert configuration surface and prevents operators
  from believing a command-line switch changes backup safety. No MAIN policy,
  BACKUP known-free requirement, fallback, gate, or acceptance threshold
  changed.
- Evidence: runner, Makefile, docs, and contract tests no longer expose or pass
  the ignored flag. Runtime Python contracts are required below; SITL and
  recorded-data acceptance remain independent evidence.
- Removal condition: none. A future role-policy experiment must be a typed,
  explicitly scoped test fixture and must not be presented as a product mode.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && python3 -m unittest
  tools.runtime.tests.test_runtime_contract tools.runtime.tests.test_html_report`.

### 2026-08-27 - Propagated odometry enable switch duplicated a mandatory contract

- Owner: FAST-LIO ROS estimator output boundary. Scope: propagated odometry is
  the only estimator stream accepted by the navigation runtime, so the
  `propagated_odometry.enabled` parameter and all branches that attempted to
  disable that stream were removed. Publish rate, ingress capacity, history
  duration, and correction-age limits remain explicit typed parameters.
- Safety impact: removes a dead-looking configuration path that could make the
  estimator report a superficially valid corrected stream while the runtime
  still required propagated odometry. The node now always constructs and starts
  the propagated worker, always publishes its health as required, and keeps the
  dynamic TF owner unambiguously propagated. No gate is relaxed; propagation
  health starts invalid and remains fail-closed until the worker is ready.
- Evidence: removed the YAML key, loader field, disabled branches, and
  misleading diagnostics flag; parameter-loader and FAST-LIO regression tests
  plus the package build are required below. Dataset/SITL evidence remains
  open.
- Removal condition: none. Reintroducing an enable switch requires a different
  end-to-end runtime contract in which navigation can safely consume an
  explicitly selected estimator source.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select fast_lio_ros
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/fast_lio_ros
  --output-on-failure`.

### 2026-08-27 - Backup failure classification preserved actionable causes

- Owner: planner result boundary. Scope: backup return codes are mapped to
  distinct planner diagnostics for timeout, optimizer failure, initialization
  failure and explicit no-path. The raw backend return value remains in the
  planner log; only the previously lossy public result classification changes.
- Safety impact: no candidate authorization or fallback gate is relaxed. A
  timeout still fails closed, and every non-authorized backup result still
  leaves the previous committed bundle untouched. The change improves the
  runtime error history and prevents a genuine deadline/initialization issue
  from being mislabeled as a generic backup failure.
- Evidence: added a focused classifier test and updated both PlanFromRest and
  ReplanOnce failure paths to apply cancellation/deadline precedence before
  the specific backup cause. Dataset and SITL evidence remain open.
- Removal condition: none. New backend return codes must be assigned an
  explicit planner result before they can enter the product boundary.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest
  --test-dir build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Workspace and installed runtime profiles are parity-checked

- Owner: navigation bringup configuration boundary. Scope: the workspace
  runtime profile and the package-installed default now contain the same
  navigation parameter map, including the status topic. A contract test
  compares their parsed values so a launch-default drift cannot silently create
  a second topic/rate/frame owner.
- Safety impact: configuration parity only; no gate, threshold, fallback,
  unknown policy, timestamp rule, or planner behavior changed.
- Evidence: added the missing status-topic entry and a parsed-profile equality
  regression in the Python runtime contract suite. End-to-end launch evidence
  remains required.
- Removal condition: none. If the profiles must differ for deployment, split
  them into explicitly named deployment contracts and add a documented mapping
  test rather than allowing accidental drift.
- Verification command: `python3 -m unittest
  tools.runtime.tests.test_runtime_contract`.

### 2026-08-27 - Main-only known-free completion was rejected too early

- Owner: planner candidate authorization. Scope: an `allow_unknown` mission may
  commit a candidate without a BACKUP suffix only when the complete executable
  trajectory is independently certified `KNOWN_FREE` (the `FINISH`/`NO_NEED`
  result). Candidates with a BACKUP suffix continue to use the mission policy
  for MAIN and the mandatory known-free policy for BACKUP.
- Safety impact: removes an over-restrictive early rejection that treated every
  main-only candidate as unsafe, while preserving fail-closed validation. A
  main-only trajectory that contains UNKNOWN, OCCUPIED, OUT_OF_MAP or an invalid
  swept segment is still rejected by the independent certificate.
- Evidence: added `candidateCertificatePolicy()` and a regression covering both
  main-only and MAIN-plus-BACKUP schedules; navigation planning CTest passed
  5/5. End-to-end dataset and repeated SITL evidence remain open.
- Removal condition: none. Any new candidate role must select an explicit
  certificate policy before it can reach the commit boundary.
- Verification command: `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest
  --test-dir build/navigation_planning_backend --output-on-failure`.
### 2026-08-27 - Route reference was incorrectly promoted to a sampled hard gate

- Owner: nominal trajectory quality objective and corridor certificate. Scope:
  removed `vertical_guide_tolerance_m` and the sampled vertical-guide envelope
  rejection from the nominal optimizer.
- Safety impact: the route reference no longer rejects a trajectory solely for
  leaving the guide's vertical envelope. Safety remains enforced by sampled
  corridor containment, dynamic/flatness gates, and the final world-model
  swept certificate for the executable candidate; no UNKNOWN permission was
  added and no existing safety certificate was relaxed.
- Evidence: the route-reference terms remain objective-only; focused planner
  configuration and trajectory tests must pass after this change.
- Removal condition: none. A future vertical terrain/altitude constraint must
  be introduced as a separately owned geometric contract with its own evidence,
  not as a route-reference tolerance.
- Verification: `source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
  colcon build --packages-select navigation_planning_backend
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Missing mission no longer inherited an exploratory UNKNOWN default

- Owner: planner mission-policy boundary. Scope: changed the planner's
  no-mission default to `RequireKnownFree`; `allow_unknown` is obtained only
  from a successfully validated mission contract.
- Safety impact: closes the gap where an omitted mission could implicitly make
  UNKNOWN traversable. This is fail-closed and does not alter the explicit
  mission policy or the independent BACKUP known-free rule.
- Evidence: product planner configuration test asserts the no-mission default;
  navigation mission, planner and runtime contract tests remain required.
- Removal condition: none. A future permissive mode must be an explicit,
  validated mission policy and retain the role-aware candidate certificate.
- Verification: `source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
  colcon build --packages-select navigation_planning_backend
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Removed the EXP jerk objective bypass and negative objective sentinel

- Owner: planner objective configuration and runtime validation harness. Scope:
  objective weights now use `0` to disable a term; negative and non-finite
  objective weights are rejected. The historical TB-001 runner option, marker,
  report provenance path and A/B test fixtures were removed.
- Safety impact: removes an uncertified behavior switch and prevents a negative
  sentinel from silently changing optimizer semantics. Analytic dynamic and
  geometric certificates remain independent of objective weights; no safety
  threshold was relaxed and no alternate planner mode was introduced.
- Evidence: planner config rejects invalid weights; generated planner config
  keeps EXP jerk weight at zero; runtime tests cover the zero-disabled contract;
  focused planner build/tests and full runtime Python tests are required after
  this change. Existing historical ledger entries describing the experiment are
  retained as provenance only.
- Removal condition: none. Any future objective experiment must be a separate,
  explicitly non-product tool and must not be exposed as a flight/runtime
  switch.
- Verification: `python3 -m unittest discover -s tools/runtime/tests -p
  'test_*.py'` and `source /opt/ros/jazzy/setup.bash && source install/setup.bash
  && colcon build --packages-select navigation_planning_backend
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 - Localization reset no longer publishes an unfinalized empty snapshot

- Owner: mapping actor and navigation runtime publication boundary. Scope: a
  localization-epoch change still resets the mutable map and advances the world
  generation, but no longer exports/publishes an empty intermediate snapshot.
  The first post-reset observation is exported once and published through the
  existing `publishAndFinalize` transaction.
- Safety impact: removes a transient state where a new world was visible before
  the dependent execution certificate was invalidated. It also removes one
  full-map export on reset; no occupancy policy, UNKNOWN policy, certificate
  predicate or threshold was relaxed.
- Evidence: source has one snapshot export per accepted map update and the
  runtime publication path has one atomic finalization boundary. Mapping and
  runtime tests/builds are required after this change.
- Removal condition: none. Any future reset marker must be a diagnostic event,
  not a second world publication without dependent-state finalization.
- Verification: `source /opt/ros/jazzy/setup.bash && source install/setup.bash
  && colcon build --packages-select navigation_mapping navigation_runtime
  --cmake-args -DBUILD_TESTING=ON` followed by the selected package CTests.

### 2026-08-27 — Removed boolean hardware visibility enable gate

- **Owner:** navigation runtime.
- **Scope:** `hardware_visibility_certified` was removed from runtime parameters.
- **Safety impact:** hardware startup now fails closed unconditionally until an
  immutable sensor FOV/mounting certificate and a runtime verifier exist. A
  parameter cannot turn an unverified visibility assumption into evidence.
- **Evidence:** runtime source inspection and SITL/unit builds; no hardware
  acceptance is claimed.
- **Removal condition:** replace the unconditional block only after the
  certificate schema, verifier, and repeated hardware/recorded-data evidence
  are implemented.
- **Verification command:** `rg -n "hardware_visibility_certified" src config docs`
  returns no active parameter usage; runtime unit tests remain green.

### 2026-08-27 — Mapping probability defaults now satisfy their own contract

- **Owner:** mapping configuration loader.
- **Scope:** the optional `raycasting/p_miss` default is now `0.35`, matching
  the documented informative-miss invariant (`p_miss < 0.5`) and the product
  fixture baseline. The explicit YAML value remains authoritative.
- **Safety impact:** prevents an omitted optional key from selecting the old
  contradictory `0.70` default and failing only after the rest of the map
  configuration has loaded. No occupied/free threshold or UNKNOWN policy was
  relaxed; invalid explicit values still fail closed.
- **Evidence:** strict mapping configuration validation and
  `test_rog_map_vendor` pass after the change.
- **Removal condition:** none. Any future default must satisfy the probability
  ordering invariant and be covered by a configuration regression.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select rog_map_vendor
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/rog_map_vendor
  --output-on-failure`.

### 2026-08-27 — Planner and mapping configuration parse once per boundary

- **Owner:** planner and mapping configuration boundaries.
- **Scope:** planner trajectory, backup and A* configuration now share one
  parsed YAML document; mapping configuration validates all loaded geometry,
  probability and map-bound parameters before allocation/reset.
- **Safety impact:** removes repeated parse overhead and rejects inconsistent
  geometry before a planner query or map allocation can use it. No timeout,
  unknown-space policy, certificate predicate or safety threshold was relaxed.
- **Evidence:** focused planner, mapping and runtime package builds/CTest pass;
  invalid YAML type, probability and geometry regressions are present.
- **Removal condition:** none. New configuration consumers must either reuse
  the typed boundary or document a separate owner and contract test.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select rog_map_vendor
  navigation_planning_backend navigation_mapping navigation_runtime
  --cmake-args -DBUILD_TESTING=ON` followed by the selected package CTests.

### 2026-08-27 — Snapshot ray certificates now match inclusive virtual planes

- **Owner:** immutable mapping snapshot traversal certificate.
- **Scope:** evidence and inflated ray checks now use the same inclusive
  virtual ground/ceiling boundaries as point classification; inflated checks
  use the inflated plane bounds, and unknown inflation no longer masks an
  unknown cell as free near a virtual plane.
- **Safety impact:** closes a boundary inconsistency where a segment exactly on
  a virtual plane could pass the ray certificate under `allow_unknown` despite
  being classified `OCCUPIED`. The change tightens consistency and does not
  broaden UNKNOWN access.
- **Evidence:** added a regression covering both grid layers and both unknown
  policies; mapping/runtime package tests are the verification gate.
- **Removal condition:** none. Any future layer certificate must share the
  point-classification boundary semantics and test the exact boundary values.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mapping
  navigation_runtime --cmake-args -DBUILD_TESTING=ON` followed by their CTests.

### 2026-08-27 — Propagated odometry receives an explicit localization barrier

- **Owner:** typed estimator-health contract, FAST-LIO output publisher,
  navigation runtime and PX4 External Mode.
- **Scope:** `EstimatorHealth` now carries the latest propagated-state source
  timestamp. Runtime and PX4 reject propagated `nav_msgs/Odometry` at or before
  that timestamp after an epoch transition, reject source-time regressions, and
  hold the state lease until a valid post-transition boundary exists.
- **Safety impact:** closes the untagged-message gap where a delayed propagated
  packet could be assigned the new active localization epoch and feed planning
  or control after a reset. This is fail-closed and may discard boundary
  samples; it does not broaden any command or UNKNOWN-space gate.
- **Evidence:** `navigation_contracts`, `fast_lio_ros`, `navigation_runtime` and
  `px4_navigation_external_mode` rebuild successfully; typed contract coverage
  and source inspection verify the field and both consumers. Reset/replay
  evidence on representative recorded data and SITL remains open.
- **Removal condition:** none. The barrier may only be replaced by a typed
  epoch-carrying state message after equivalent or stronger end-to-end
  provenance evidence is available.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_contracts
  fast_lio_ros navigation_runtime px4_navigation_external_mode
  --cmake-args -DBUILD_TESTING=ON` followed by the selected package CTests.

### 2026-08-27 — Typed estimator health no longer grants an unhealthy grace

- **Owner:** PX4 External Mode health boundary.
- **Scope:** removed `navigation.lio_health_grace_s`, the associated mission
  configuration, and both execution/arming grace branches. Health source stamps
  must be positive and strictly increasing; malformed, stale, future or
  non-monotonic samples make the health gate fail closed.
- **Safety impact:** an explicit unhealthy typed health sample can no longer
  leave External Mode active for a configurable interval. The bounded
  pre-health stationary acquisition window remains only before any typed
  health sample has been received; it cannot mask an invalid sample.
- **Evidence:** PX4 External Mode rebuilt with the parameter removed; source
  scan shows no `lio_health_grace_s` consumer. Negative health tests and loaded
  SITL/recorded-data timing evidence remain required.
- **Removal condition:** none. Any recovery behavior must be represented by an
  explicit state with its own safety certificate, not a time grace parameter.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  px4_navigation_external_mode --cmake-args -DBUILD_TESTING=ON` followed by
  the External Mode CTest.

### 2026-08-27 — Invalid mapping counts are rejected instead of clamped

- **Owner:** mapping configuration boundary.
- **Scope:** removed the implicit conversion of non-positive
  `point_filt_num` and `raycasting/batch_update_size` to `1`; the strict
  validator now sees and rejects the original invalid value.
- **Safety impact:** prevents malformed configuration from silently changing
  map update density and timing. No valid configuration or safety threshold
  changed.
- **Evidence:** the mapping invalid-configuration regression now exercises the
  constructor failure directly; `test_rog_map_vendor` passes.
- **Removal condition:** none.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select rog_map_vendor
  --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/rog_map_vendor
  --output-on-failure`.

### 2026-08-27 — CIRI iteration exhaustion telemetry is accurate

- **Owner:** planner corridor-generation diagnostics.
- **Scope:** both `while (max_iter--)` exhaustion checks now recognize the
  post-decrement sentinel `-1`.
- **Safety impact:** diagnostics no longer under-report a fully exhausted
  ellipsoid iteration. Geometry, iteration limits and acceptance predicates
  are unchanged.
- **Evidence:** planner source review and focused planner build/test; no gate
  was relaxed.
- **Removal condition:** none.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 — Stale command certificates require disjoint map provenance

- **Owner:** immutable world publication and planner command certificate.
- **Scope:** a swept candidate now exports a conservative protected region.
  Immutable mapping snapshots retain persistent per-revision change records;
  the publication store may transfer a certificate across a newer revision
  only when every intervening change is proven outside that region. Missing
  history, invalid geometry, map sliding, epoch changes and generation changes
  remain fail-closed.
- **Safety impact:** removes unconditional revision-based liveness rejection
  for updates outside the executable trajectory without accepting a stale map
  when a change can affect the trajectory. The region includes the certificate
  tube and the mapping inflation/unknown-inflation footprint; it does not relax
  UNKNOWN, occupancy or OUT_OF_MAP predicates.
- **Evidence:** added world-authorizer disjoint-provenance coverage and rebuilt
  the mapping, planning and runtime packages. Full loaded dataset, race
  distribution and SITL evidence remain open.
- **Removal condition:** none. A future mutable-map implementation must expose
  equivalent complete change provenance or retain unconditional stale-world
  rejection.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select rog_map_vendor
  navigation_world_model navigation_mapping navigation_planning_backend
  navigation_runtime --cmake-args -DBUILD_TESTING=ON` followed by the focused
  world-store, mapping, trajectory and facade tests.

### 2026-08-27 — DiagnosticArray no longer controls External Mode health

- **Owner:** PX4 External Mode health boundary.
- **Scope:** removed the legacy `/lio/diagnostics` subscription and parser from
  External Mode. Only typed `/lio/health` can establish or invalidate the
  estimator control lease; DiagnosticArray remains published for diagnosis.
- **Safety impact:** eliminates the untyped compatibility path that could make
  safety behavior depend on diagnostic strings before the typed contract was
  available. Missing typed health now fails closed; no recovery grace or gate
  was added.
- **Evidence:** source scan shows no External Mode DiagnosticArray health
  consumer; the PX4 package rebuild and External Mode CTest pass.
- **Removal condition:** none. A future compatibility adapter must not control
  flight state without an explicit typed contract and equivalent evidence.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  px4_navigation_external_mode --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/px4_navigation_external_mode --output-on-failure`.

### 2026-08-27 — Safety-stop replan grace is no longer a mission parameter

- **Owner:** PX4 External Mode mission controller.
- **Scope:** removed `safety_stop_replan_grace_s` from the mission contract,
  parser and all runtime profiles. A stop waypoint now follows the explicit
  braking and stationary-confirmation state; a verified replacement can resume
  before confirmation, otherwise the existing PX4 Hold handover is used.
- **Safety impact:** removes an unowned time-based behavior knob that could
  delay fail-closed handover after a settled stop. No stop confirmation value,
  collision predicate or trajectory certificate was relaxed.
- **Evidence:** mission controller regression covers replacement before
  confirmation and handover after confirmation; PX4 External Mode CTest passes.
- **Removal condition:** none. Any future recovery interval must be an explicit
  state with a safety certificate and measured evidence, not a mission scalar.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mission
  px4_navigation_external_mode --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/px4_navigation_external_mode --output-on-failure`.

### 2026-08-27 — Flatness certification no longer has tunable widening slack

- **Owner:** planner trajectory-dynamics certificate.
- **Scope:** removed `flatness_gate_margin_fraction` from the planner schema,
  runtime YAML and both optimizer paths. Body-rate and thrust limits are now
  checked exactly against the configured physical envelope.
- **Safety impact:** objective tuning can no longer authorize a trajectory
  above the configured dynamic envelope. The change tightens the gate and
  does not raise any limit or add a fallback.
- **Evidence:** nominal/backup trajectory gate code, configuration scan and
  focused planner build/tests verify the single exact threshold contract.
  Repeated dataset/SITL feasibility and dynamic distributions remain open.
- **Removal condition:** none. Any future margin must be part of a separately
  certified physical envelope, not an optimizer or mission parameter.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`.

### 2026-08-27 — Nominal objective and backup parameter ownership are explicit

- **Owner:** `traj_opt::Config` and the nominal/backup trajectory optimizer
  boundaries. **Scope:** `piece_num` and `uniform_time_en` are loaded only for
  the backup profile, where the optimizer owns that parameterization. Nominal
  piece count and timing remain derived from its guide/corridor contract. The
  nominal waypoint quality term is named
  `traj_opt.exp_traj.objective.waypoint_attraction_weight`; backup does not load
  or infer that term.
- **Safety impact:** removes misleading configuration surfaces and prevents a
  quality objective from being mistaken for a corridor certificate or backup
  authorization. No UNKNOWN, occupancy, dynamic, flatness or world-swept gate
  is relaxed, and no safety threshold changes.
- **Evidence:** synchronized planner/runtime build succeeds; focused planner
  configuration, trajectory and optimizer-seed tests cover the new ownership
  and field name. Repeated dataset/SITL conditioning and latency evidence
  remains open.
- **Removal condition:** none. Any future parameter must have one owner, a
  physical unit or explicit objective role, domain validation, and a focused
  test before entering the product profile.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend navigation_runtime --cmake-args -DBUILD_TESTING=ON`
  followed by `test_planner_config`, `test_trajectory`,
  `test_exp_optimizer_seed` and the runtime contract tests.

### 2026-08-27 — Runtime command timestamps use the shared conversion owner

- **Owner:** `navigation_common::secondsToRosTime` at the ROS message boundary.
  **Scope:** runtime command header and world-observation timestamp conversion;
  the node-local forwarding wrapper was removed. Integer nanosecond conversion
  remains owned by the same shared time utility.
- **Safety impact:** consistency/traceability only. The previous wrapper already
  delegated to the shared implementation, so no timestamp, freshness, clock
  domain or fail-closed behavior changed. Invalid conversion still produces the
  existing zero-valued message fallback and is not used to authorize a command.
- **Evidence:** runtime source has one conversion implementation and the
  focused runtime command-clock, timestamp-freshness and shutdown tests pass.
  Closed-loop timestamp/transport evidence remains open.
- **Removal condition:** none. New ROS/PX4/simulation timestamp conversions
  must use the shared utility or introduce a separately documented boundary
  contract before code review.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && cmake --build build/navigation_runtime -j1` followed by
  `test_command_clock`, `test_timestamp_freshness` and
  `test_navigation_runtime_shutdown`.

### 2026-08-27 — Reject oversized floating-point ROS timestamp input before narrowing

- **Owner:** `navigation_common::secondsToRosTime`. **Scope:** conversion from
  user-facing seconds to the ROS `int32` seconds plus nanoseconds message at
  runtime command boundaries.
- **Safety impact:** numerical fail-closed correction. A finite value larger
  than the representable ROS seconds range could previously be narrowed before
  the range check. Such input is now rejected before conversion; valid values,
  timestamp monotonicity, clock-domain selection and freshness gates are
  unchanged.
- **Evidence:** added negative, oversized-finite and maximum-representable
  boundary tests. Runtime direct command-clock and shutdown tests remain green.
- **Removal condition:** none. Any future floating-point-to-integer timestamp
  conversion must validate its destination range before narrowing.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_common
  --cmake-args -DBUILD_TESTING=ON` followed by `build/navigation_common/test_navigation_common
  --gtest_color=no`.

### 2026-08-27 — Runtime consumes product mapping invariants, not vendor configuration

- **Owner:** `navigation_mapping::MappingActor` at the mapping/runtime
  composition boundary. **Scope:** expose only the three facts runtime must
  enforce: mapping callbacks remain runtime-owned, raycast batch updates are
  one observation at a time, and virtual ground/ceiling planes are compatible
  with the selected planning frame. Backend-specific configuration stays
  private to the mapping adapter.
- **Safety impact:** architecture/clarity correction. The same values and
  checks are preserved; no occupancy, UNKNOWN, virtual-plane, timestamp or
  planner gate is relaxed. A malformed backend config still fails closed while
  constructing the actor.
- **Evidence:** runtime no longer reads `rog_map::Config` fields directly;
  rebuilt `navigation_runtime`, command-clock, timestamp-freshness and runtime
  shutdown tests pass. Full public-header vendor isolation is still an open
  extraction phase because the current snapshot adapter remains header-based.
- **Removal condition:** none. Do not add backend-specific fields to this
  product summary; add a separately owned product invariant or keep the check
  inside `navigation_mapping`.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_runtime
  --cmake-args -DBUILD_TESTING=ON` followed by runtime mapping, shutdown and
  world-snapshot tests.

### 2026-08-27 — Mapping boundary rejects ambiguous state and observation provenance

- **Owner:** `navigation_mapping::MappingActor` and the immutable
  `MappingWorldSnapshot` boundary. **Scope:** product mapping inputs now use a
  read-only point cloud; observation time must exactly equal corrected
  odometry time; observation stamps and non-zero scan sequences must advance
  within an epoch; finite pose and epoch checks occur before mutable-map reset
  or update. Backend outcome translation is centralized and unknown enum values
  throw instead of becoming a successful/default outcome. Exported grid storage
  accepts only `UNKNOWN`, `OCCUPIED` and `KNOWN_FREE`; `OUT_OF_MAP`, undefined,
  frontier and future values cannot be stored as evidence.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. This prevents a delayed,
  relabeled or mismatched cloud/pose from mutating a map generation and
  prevents ambiguous backend states from entering planning certificates.
  The deliberate false-reject consequence is dropping malformed or duplicate
  observations; no occupancy, UNKNOWN or world-swept gate is relaxed.
- **Derivation and cost:** timestamp equality and monotonicity derive from the
  shared integer-nanosecond ROS conversion and epoch contract. Point-cloud
  reuse removes one per-update allocation; nearest-offset metadata is cached
  once per actor. Full grid export remains O(N) and its p50/p95/p99 cost is
  still open; no threshold was tuned.
- **Evidence:** added actor contract tests for timestamp mismatch, non-finite
  pose and non-monotonic observation time. Mapping CTest is 2/2 and runtime
  mapping CTest is 7/7 after rebuilding both packages. Dataset, sanitizer and
  repeated SITL evidence remain open.
- **Removal condition:** none. Any future input compatibility adapter must
  preserve immutable ownership, exact timestamp pairing and explicit state
  translation, or remain outside the flight authority.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mapping
  navigation_runtime --cmake-args -DBUILD_TESTING=ON` followed by the sourced
  CTest commands for both build directories.

### 2026-08-27 — World-model traversability is a total fail-closed predicate

- **Owner:** `navigation_world_model::isCellTraversable` and all product
  planner/world consumers. **Scope:** evidence, inflated-grid, corridor and
  A* boundary decisions now use the shared predicate where they consume
  product `CellState`. Unknown/frontier are admissible only under the explicit
  allow-unknown policy; occupied, out-of-map, undefined and future enum values
  are never implicitly free. A* neighbour aggregation now represents a
  missing evidence neighbour as UNKNOWN rather than an implicit undefined
  free cell.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. This removes a false
  accept path in which a malformed/out-of-map/undefined classification could
  enter corridor generation or graph expansion. The deliberate false-reject
  consequence is that malformed state or a stricter known-free policy can end
  a search earlier; no UNKNOWN gate is relaxed.
- **Derivation and cost:** the policy is the existing product contract; no
  threshold or magic margin changed. The predicate is constexpr and the A*
  representation change is constant-time per neighbour. Behavioural parity
  for diagonal/tie cases remains covered by the mapping characterization and
  needs broader randomized replay before certification.
- **Evidence:** added policy/state matrix and invalid stored-state regression;
  planner and runtime rebuild plus focused CTest are required. Dataset,
  sanitizer and repeated SITL evidence remain open.
- **Removal condition:** none. New world-model consumers must call this
  predicate or document a stricter separately owned certificate; do not add
  local comparisons that reinterpret UNKNOWN, OUT_OF_MAP or undefined values.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend navigation_runtime --cmake-args
  -DBUILD_TESTING=ON` followed by the sourced planning/runtime CTest commands.

### 2026-08-27 — Runtime mapping consumes an opaque world view and explicit metrics

- **Owner:** `navigation_mapping::MappingActor` at the public mapping/runtime
  boundary. **Scope:** `MappingUpdateResult` and initial publication now carry
  `WorldModelViewPtr` plus `MappingSnapshotMetrics`; runtime no longer includes
  or calls the concrete snapshot storage class. `PlanningGrid` and
  `MappingWorldSnapshot` are private implementation representations and are no
  longer installed as package headers.
- **Safety impact:** architecture/ABI correction only. World identity,
  publication ordering, cell policy and certificate checks are unchanged.
  Resource counters remain observable through an explicit product value rather
  than through concrete storage statics.
- **Derivation and cost:** metrics are captured at snapshot construction, so
  telemetry does not need a cross-package static counter. Removing the
  representation from the install surface prevents future consumers from
  coupling to the full exported grid.
- **Evidence:** sourced rebuild succeeded; mapping CTest 3/3 and runtime CTest
  6/6 passed. Installed `libnavigation_mapping.so` exposes no vendor-named
  dynamic symbols; a stale incremental install is being cleaned before it is
  used as evidence. Dataset, sanitizer and repeated SITL evidence remain open.
- **Removal condition:** none. New runtime consumers must depend on
  `WorldModelView` and explicit product metrics; they must not include the
  private concrete snapshot header.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mapping
  navigation_runtime --cmake-args -DBUILD_TESTING=ON`, followed by sourced
  CTest in `build/navigation_mapping` and `build/navigation_runtime`.

### 2026-08-27 — Product ray certificates enumerate voxel-boundary ties symmetrically

- **Owner:** immutable `MappingWorldSnapshot::isSegmentTraversable`.
  **Scope:** when a segment reaches an edge or corner, the product DDA checks
  every non-empty subset of the tied axes before advancing. Endpoint and
  same-cell checks remain explicit.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. The former single-axis
  tie break could omit a cell and make the result depend on travel direction.
  The new rule may deliberately reject a segment when any cell touched at a
  boundary is occupied or not admissible; it does not turn UNKNOWN, occupied
  or out-of-map state into free space.
- **Derivation and cost:** the tied-axis set is computed from the first
  parametric boundary and a machine-epsilon-scaled comparison. At most seven
  adjacent cells are checked at a 3-axis corner; ordinary segments retain one
  transition per boundary.
- **Evidence:** added a diagonal corner regression with an occupied side cell;
  both directions reject. Mapping CTest 3/3, planning backend 5/5 and runtime
  6/6 passed after the change. Broad randomized parity/replay and latency
  distributions remain required before declaring performance or flight
  acceptance.
- **Removal condition:** none. Any future traversal optimization must preserve
  the symmetric touched-cell certificate and provide an equivalent adversarial
  test corpus.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ctest --test-dir build/navigation_mapping
  --output-on-failure` with both overlays sourced.

### 2026-08-27 — Corrected CTest count after regenerated mapping boundary

- **Owner:** validation workflow. **Scope:** the earlier ledger entries that
  reported mapping `2/2` and runtime `7/7` described an older generated test
  graph. After moving the characterization test into `navigation_mapping` and
  regenerating CMake, the authoritative current graph is mapping `3/3`,
  planning backend `5/5`, runtime `6/6`.
- **Safety impact:** evidence bookkeeping only. No test was removed to hide a
  failure; the former runtime characterization target is now owned by the
  mapping package and the stale CTest registration is not part of the current
  source graph.
- **Evidence:** sourced CTest completed with all current tests passing. The
  unsourced `ctest` attempt is retained only as an environment error in the
  error history and is not acceptance evidence.
- **Removal condition:** none. Every future validation record must include the
  exact package test graph and the required ROS/workspace overlay setup.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ctest --test-dir build/navigation_mapping
  --output-on-failure && ctest --test-dir build/navigation_planning_backend
  --output-on-failure && ctest --test-dir build/navigation_runtime
  --output-on-failure`.

### 2026-08-27 — Changed-region provenance includes quantization shells

- **Owner:** `navigation_mapping::MappingActor` at the mutable-map to
  immutable-snapshot boundary. **Scope:** the `WorldChangeRecord` envelope now
  expands backend-reported metric bounds by occupied/unknown inflation and by
  half the evidence plus inflated voxel widths, then steps each bound outward
  with `nextafter`.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A raw point/update AABB
  does not prove that every affected discrete cell was inside the recorded
  region. The old envelope could incorrectly classify a candidate as disjoint
  at a voxel boundary. An over-approximation can only trigger extra
  recertification; invalid bounds mark the update as whole-world.
- **Derivation and cost:** the shell is the maximum quantization uncertainty
  from both grid layers; inflation uses the larger occupied/unknown step. The
  calculation is constant time per axis and does not add a new runtime
  parameter.
- **Evidence:** source rebuild and sourced mapping 3/3, planning 5/5 and
  runtime 6/6 CTest pass. Repeated map-update/replay evidence remains open.
- **Removal condition:** none. Any future provenance optimization must prove
  the same discrete-cell coverage or conservatively mark the update as
  whole-world.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ctest --test-dir build/navigation_mapping
  --output-on-failure`.

### 2026-08-27 — Candidate role schedule has no implicit fallback

- **Owner:** `navigation_planning_backend::validateExecutableCandidate`.
  **Scope:** role intervals must start at zero, be finite and strictly
  increasing, meet exactly at every boundary, end exactly at trajectory
  duration, and resolve through an optional role lookup. There is no default
  MAIN role for a gap or an invalid boundary.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A tolerant boundary or
  implicit MAIN fallback could let a segment intended for the known-free
  BACKUP suffix inherit the exploratory UNKNOWN policy. Gaps, overlaps and
  malformed final intervals are rejected before certification.
- **Derivation and cost:** role resolution is exact because the producer owns
  the same stored interval endpoints; the validator performs no new tolerance
  decision. The extra schedule walk is linear in the small role list.
- **Evidence:** planning backend CTest 5/5 and runtime CTest 6/6 pass after
  the change. A producer-level closed-loop test with floating-point switch
  times and repeated SITL remains required.
- **Removal condition:** none. Do not restore a default role or boundary
  grace; if a producer cannot emit an exact partition it must be fixed at its
  contract boundary.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ctest --test-dir build/navigation_planning_backend
  --output-on-failure`.

### 2026-08-27 — Symmetric DDA tie scale excludes infinite sentinels

- **Owner:** immutable `MappingWorldSnapshot::isSegmentTraversable`.
  **Scope:** the machine-epsilon tie tolerance is scaled only by finite
  parametric boundary values; axes with no remaining transition use the
  infinite sentinel only for ordering, never for the tolerance scale. Every
  tied-axis subset is still checked.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. The first symmetric
  supercover implementation used `DBL_MAX` in its scale, making the tolerance
  effectively enormous whenever one axis was stationary and potentially
  omitting a real transition. This correction removes that numerical
  acceptance path; it does not classify any unknown cell as free.
- **Derivation and cost:** finite boundary values determine the scale and
  ordinary paths retain the same one-boundary step. Corner/edge crossings
  check at most seven adjacent cells.
- **Evidence:** the diagonal corner regression rejects both travel
  directions; sourced mapping CTest 3/3, planning 5/5 and runtime 6/6 pass.
  Randomized numerical parity and latency distributions remain open.
- **Removal condition:** none. Any DDA optimization must retain finite-only
  scaling and the symmetric touched-cell regression.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && ctest --test-dir build/navigation_mapping
  --output-on-failure`.

### 2026-08-27 — Product metadata no longer exposes backend version or key names

- **Owner:** product package metadata and runtime diagnostics. **Scope:** the
  PX4 package description no longer embeds a release version, the runtime
  error names the product mapping concept instead of a backend configuration
  key, and runtime no longer declares an unused vendor test dependency.
- **Safety impact:** vocabulary/packaging only. Loader behavior, virtual-plane
  validation and all safety gates are unchanged; backend names remain confined
  to the private adapter and provenance/tooling boundary.
- **Evidence:** navigation mapping/runtime rebuild and sourced CTest pass;
  complete vendor/provenance cleanup is intentionally a separate phase so
  attribution and parity evidence are not deleted.
- **Removal condition:** none. Product-facing names must remain backend-neutral;
  do not rename the private config root until a typed mapping configuration
  owns and verifies the full schema.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mapping
  navigation_runtime --cmake-args -DBUILD_TESTING=ON`.

### 2026-08-27 — Planner configuration has deterministic neutral defaults

- **Owner:** product planner and trajectory configuration value types. **Scope:**
  every scalar that was previously default-constructed without initialization
  now has an explicit neutral value; file-based loaders still provide the
  product baseline and reject missing required physical limits.
- **Safety impact:** numerical hygiene. A default-constructed test/helper or
  future boundary object can no longer read indeterminate mass, gravity,
  flatness, deadline, or planner-limit values. No valid YAML behavior or
  safety threshold is raised.
- **Derivation and cost:** zero/false means “not configured” for values that
  the loaded configuration validates as positive, and is constant-initialized
  with no runtime cost.
- **Evidence:** planner source rebuild and sourced planning CTest 5/5 pass.
  Full sanitizer and end-to-end evidence remain required.
- **Removal condition:** none. New configuration fields must be initialized
  and validated at their owning boundary.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON` followed by
  sourced planner CTest.

### 2026-08-27 — Backup visibility always certifies the terminal point

- **Owner:** `Planner::generateBackupTrajectory` visibility sampling. **Scope:**
  the sampled candidate list now appends the exact `total_dur` position after
  the strictly-before-terminal sampling loop; a non-finite terminal point
  fails the solve.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. The previous all-visible
  fast path could return `FINISH` after checking only samples with
  `t < total_dur`, leaving the actual executable endpoint uncertified. The
  change adds a required existing trajectory point to the same known-free
  check and does not weaken the policy.
- **Derivation and cost:** one endpoint insertion and one endpoint ray/check;
  no new parameter or tolerance is introduced.
- **Evidence:** planner source rebuild and sourced planning CTest are required;
  a closed-loop visibility/backup regression and repeated SITL remain open.
- **Removal condition:** none. Any future visibility fast path must include
  the exact terminal point in its certificate.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend --cmake-args -DBUILD_TESTING=ON` followed by
  sourced planner CTest.

### 2026-08-27 — Bounded provenance retains the newest change window

- **Owner:** `MappingActor` change-history publication. **Scope:** provenance is
  stored as an immutable newest-first window of records. Each update copies at
  most the retained window, prepends the new record, and drops only the oldest
  record. Records older than that window are intentionally unavailable for
  stale-certificate recertification.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A singly linked bounded
  chain cannot truncate its tail in O(1): the earlier implementation skipped
  the newest predecessor at saturation. The flat window now preserves every
  newest record in order; a missing link still rejects recertification rather
  than assuming unchanged geometry.
- **Derivation and cost:** the bounded window is fixed at 256 records, so
  memory and copy cost are bounded. The retention value is an implementation
  bound, not a permission to accept an older certificate.
- **Evidence:** mapping source rebuild and sourced mapping/world-model tests
  are required; long-run history rollover and stale-candidate replay remain
  open characterization work.
- **Removal condition:** none. Any future history storage must preserve newest
  records and fail closed across an unavailable provenance interval.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mapping
  --cmake-args -DBUILD_TESTING=ON` followed by sourced mapping CTest.

### 2026-08-27 — Provenance rollover no longer skips the newest revision

- **Owner:** immutable world-model history contract. **Scope:** all stale-map
  recertification requires contiguous newest-first revision records; a history
  window shorter than the requested interval fails closed.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. This closes the concrete
  rollover defect found during adversarial review where revision `N-1` could be
  skipped while revision `N` was retained.
- **Evidence:** mapping source rebuild and the newest-record/truncation
  regression are required; long-run rollover and repeated SITL remain open.
- **Removal condition:** none. Do not replace the bounded flat window with a
  singly linked list unless tail truncation preserves all newest records.
- **Verification command:** sourced mapping build followed by mapping CTest.

### 2026-08-27 — Mapping test targets use the product C++20 contract

- **Owner:** `navigation_mapping` CMake test targets. **Scope:** the
  `test_mapping_world_model` target now explicitly requests C++20, matching
  the library and the other mapping tests.
- **Safety impact:** build-contract only. This prevents a compiler default from
  changing the language semantics of a world-model safety regression test; no
  runtime behavior or gate changed.
- **Evidence:** fresh mapping build and sourced mapping CTest are required.
- **Removal condition:** none. Keep C++20 explicit on every product and test
  target.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_mapping
  --cmake-args -DBUILD_TESTING=ON` followed by sourced mapping CTest.

### 2026-08-27 — Candidate admission rechecks measured-state anchoring

- **Owner:** runtime execution boundary and planner state snapshot. **Scope:**
  a candidate is sampled at the admission timestamp and compared with the
  newest fresh propagated state before entering the execution bundle store;
  the planner also copies one state snapshot at the start of each solve and
  does not read the concurrently updated ingress state during that solve.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A solve completing after
  a vehicle-state update must not publish a geometrically detached command.
  The existing product geometric anchor limit is reused; no independent
  tracking-lag or velocity-limit parameter is introduced. The measured state
  remains the authority for position continuity, while trajectory dynamics and
  the PX4 consumer retain their own physical checks.
- **Derivation and cost:** one immutable state load, one candidate evaluation,
  and one finite position-envelope check at the commit boundary. The solve
  snapshot removes a data race without holding the ingress mutex throughout
  optimization.
- **Evidence:** planner/runtime rebuild and sourced CTest are required;
  delayed-state, concurrent-state and closed-loop SITL tests remain open.
- **Removal condition:** none. Any future candidate path must perform the same
  fresh-state and world-identity checks before exposure.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select
  navigation_planning_backend navigation_runtime --cmake-args
  -DBUILD_TESTING=ON` followed by sourced planner/runtime CTest.

### 2026-08-27 — Quaternion gates use scale-stable finite checks

- **Owner:** mapping, runtime odometry ingress and planner kinematic-state
  boundary. **Scope:** quaternion validity uses the largest finite coefficient
  as the scale, then normalizes a bounded quaternion; it no longer relies on
  a squared norm that can overflow for otherwise finite input.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. Zero, non-finite and
  numerically degenerate orientations are rejected, while very large finite
  representations cannot pass into an invalid normalized state.
- **Evidence:** mapping/planner/runtime rebuild and sourced CTest are required;
  explicit overflow-scale quaternion regressions and sanitizer coverage remain
  open.
- **Removal condition:** none. Keep scale-stable validation at every ingress
  boundary; do not revert to unchecked `normalized()` on external values.
- **Verification command:** sourced selected-package build followed by mapping,
  planner and runtime CTest.

### 2026-08-27 — Snapshot geometry rejects invalid inflation metadata

- **Owner:** immutable mapping snapshot constructor. **Scope:**
  `occupied_inflation_radius_m` must be finite and nonnegative before a snapshot
  can be published.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A NaN radius can no longer
  make a downstream `radius < robot_radius` check evaluate false and silently
  bypass the clearance contract.
- **Evidence:** mapping/planner rebuild and sourced CTest are required.
- **Removal condition:** none. Any world view supplying geometry must validate
  all physical clearance metadata before publication.
- **Verification command:** sourced mapping, planner and runtime CTest.

### 2026-08-27 — Candidate validity never extends caller expiry

- **Owner:** planner candidate export. **Scope:** candidate validity ends at the
  earlier of the caller's freshness deadline and the trajectory endpoint; an
  invalid interval is rejected.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. The previous `max()` could
  extend a caller-owned freshness deadline to the end of a long trajectory.
- **Evidence:** planner build and sourced CTest are required; direct boundary
  tests for future, expired and shortened candidates remain open.
- **Removal condition:** none. Caller expiry is a hard upper bound and may not
  be widened by a producer.
- **Verification command:** sourced planner build and planner CTest.

### 2026-08-27 — Failed planner-history ACK clears both command owners

- **Owner:** runtime execution commit boundary and planner staged-candidate
  owner. **Scope:** when the execution bundle commits but the planner-history
  ACK fails, runtime now clears the planner staged candidate before invalidating
  the execution bundle.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. The two ownership
  boundaries can no longer leave a private staged candidate alive after its
  execution counterpart was withdrawn; a later cycle cannot mistake that
  candidate for a fresh solve result.
- **Derivation and cost:** one discard operation on the existing planner
  ownership boundary; no new state, parameter or timing gate.
- **Evidence:** sourced runtime build and CTest 6/6 pass; an explicit injected
  ACK-failure integration test remains follow-up coverage.
- **Removal condition:** none. Any future execution/planner two-phase commit
  must clear both owners on every failed ACK.
- **Verification command:** sourced selected-package build followed by runtime
  CTest.

### 2026-08-27 — Snapshot adapter removes redundant cell conversion pass

- **Owner:** `navigation_mapping` immutable snapshot adapter. **Scope:** the
  private snapshot storage now moves the backend's already-byte-sized evidence
  codes directly and decodes them only when answering a world-model query.
- **Safety impact:** performance-only with unchanged semantics. Snapshot
  validation still rejects every code outside UNKNOWN/OCCUPIED/KNOWN_FREE;
  conversion is deferred, not removed.
- **Derivation and cost:** removes one allocation and byte-to-enum loop per
  full grid export. The immutable grid remains owned by the snapshot, and map
  publication/commit gates are unchanged.
- **Evidence:** fresh mapping build and sourced mapping CTest 3/3 pass; p50/p95
  callback and observed-duration distributions remain required before claiming
  an end-to-end performance improvement.
- **Removal condition:** none. Keep storage representation private; do not
  expose backend cell codes through the product API.
- **Verification command:** sourced selected-package build followed by mapping
  CTest and runtime performance replay.

### 2026-08-27 — Runtime profile has one workspace source

- **Owner:** `navigation_runtime` package configuration and launch tooling.
  **Scope:** `config/runtime/mapping.yaml` is the only hand-edited runtime
  profile. CMake copies it into the build/install tree as
  `navigation_runtime.yaml` for the package launch default; the former second
  source file is removed.
- **Safety impact:** configuration ownership only. No parameter value, clock
  policy, planner gate or hardware permission changes. A future profile edit
  cannot silently diverge between workspace runner and installed launch.
- **Derivation and cost:** one configure-time file copy; no runtime parsing or
  callback cost is added.
- **Evidence:** runtime CMake configure/build and the Python runtime contract
  test are required; launch/install smoke coverage remains part of the next
  SITL phase.
- **Removal condition:** none. Do not add a second hand-edited runtime profile;
  update the canonical workspace file and regenerate the install tree.
- **Verification command:** `source /opt/ros/jazzy/setup.bash && source
  install/setup.bash && colcon build --packages-select navigation_runtime
  --cmake-args -DBUILD_TESTING=ON` followed by
  `python3 -m pytest -q tools/runtime/tests`.

### 2026-08-27 — World evidence freshness gates executable commands

- **Owner:** runtime planning, candidate admission and command publication
  boundaries. **Scope:** the latest immutable world snapshot observation time
  must be fresh in the ROS clock domain before solving, committing or
  continuing to publish a trajectory. The same existing input age limit is
  checked at each race-prone boundary.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. Fresh propagated odometry
  cannot by itself certify that the map evidence is current. A stale or missing
  world snapshot cancels active solving, clears command exposure and rejects
  candidate admission; a later fresh mapping publication may recover it.
- **Derivation and cost:** one identity load and timestamp classification per
  boundary; no new parameter and no relaxed gate. The limit remains a
  provisional source-age contract and requires distribution evidence before
  future tuning.
- **Evidence:** runtime build, CTest and timestamp freshness tests are
  required; stale-world interleaving and repeated SITL evidence remain open.
- **Removal condition:** none. Do not allow an execution-state freshness check
  to substitute for world-evidence freshness.
- **Verification command:** sourced selected-package build followed by runtime
  CTest and repeated SITL/dataset timing validation.

### 2026-08-27 — Mapping callback telemetry measures its full envelope

- **Owner:** mapping runtime telemetry and report renderer. **Scope:**
  `mapping_callback_total_us` is measured around `MappingActor::process`, so it
  includes map update, snapshot export and dependent publication/finalization;
  the backend update duration remains a separate metric.
- **Safety impact:** observability only. No command gate or planner behavior is
  changed.
- **Derivation and cost:** two steady-clock reads per callback and one integer
  subtraction. Report timing descriptions now label snapshot export as an
  observed compute phase.
- **Evidence:** runtime build, CTest and Python runtime tests are required;
  repeated artifacts are still required for performance conclusions.
- **Removal condition:** none. Do not report backend update time as the full
  mapping callback envelope.
- **Verification command:** sourced runtime build/CTest followed by
  `python3 -m pytest -q tools/runtime/tests`.

### 2026-08-27 — Unused waypoint optimizer removed from product build

- **Owner:** `navigation_planning_backend` source list. **Scope:** the private
  `GcopterWayptS3/S4` optimizer, which had no production consumer, is removed
  from the target and source tree. The active optimizer path and shared
  transform utility remain unchanged.
- **Safety impact:** build surface and maintenance only. No trajectory behavior,
  parameter, certificate or fallback path changes.
- **Derivation and cost:** removes one unreferenced translation unit and header;
  no runtime cost is added.
- **Evidence:** repository-wide consumer search, backend rebuild and backend
  CTest are required.
- **Removal condition:** none. New optimizer implementations require a named
  product owner and a production call site before entering the target.
- **Verification command:** sourced backend build followed by planner CTest.

### 2026-08-27 — Startup snapshot export is performed once

- **Owner:** `navigation_mapping::MappingActor` startup path. **Scope:** the
  initial immutable snapshot obtains nearest-cell metadata and product grid
  storage from one backend export; construction no longer performs a separate
  full-grid export merely to initialize offsets.
- **Safety impact:** no map, certificate, unknown-policy or command-gate
  change. The snapshot remains detached and immutable before publication.
- **Derivation and cost:** removes one full-grid copy/translation at startup;
  steady-state per-update export remains a separate open performance item.
- **Evidence:** mapping actor/world-model tests and sourced mapping CTest are
  required; repeated callback timing evidence remains open.
- **Removal condition:** none. Do not reintroduce an initialization-only full
  export without measuring its ownership and latency impact.
- **Verification command:** sourced mapping build followed by mapping CTest.

### 2026-08-27 — Runtime freshness window has one explicit name

- **Owner:** `navigation_runtime` timing contract. **Scope:** rename the
  ambiguous `input_max_age_s` parameter to `data_freshness_window_s`; its
  nanosecond conversion is used by the runtime's world-source, execution-state
  and command-lease checks. PX4 and bridge age parameters remain separate
  because they measure different clocks and boundaries.
- **Safety impact:** naming/ownership only; no threshold or freshness predicate
  changed. Keeping one runtime value avoids adding three behavior knobs without
  evidence for distinct policies.
- **Derivation and cost:** one seconds-to-nanoseconds conversion at startup;
  no runtime allocation or additional check.
- **Evidence:** canonical profile contract, runtime build/CTest and Python
  contract tests; repeated timing distributions remain required before tuning.
- **Removal condition:** none. Split the value only when measured evidence and
  a documented ownership contract prove separate policies are necessary.
- **Verification command:** sourced runtime build and CTest followed by
  `python3 -m pytest -q tools/runtime/tests`.

### 2026-08-27 — Product planner profile no longer advertises a dead ROS callback

- **Owner:** product planner configuration. **Scope:** remove the disabled
  `rog_map/ros_callback` topic block from the product planner profile. Runtime
  mapping admission is now exclusively the atomic `RegisteredScan` contract;
  vendor parser fixtures retain their isolated callback schema for parser tests.
- **Safety impact:** clarity and ownership only. The callback was disabled and
  no runtime subscription or certificate predicate is changed.
- **Derivation and cost:** removes two obsolete topic names and one unused
  timeout from the product profile; no runtime allocation or branch is added.
- **Evidence:** source/config search plus mapping/runtime build and CTest are
  required. Do not infer product callback behavior from vendor fixtures.
- **Removal condition:** none. Reintroduce a callback only with a named
  product-owned message contract, epoch policy and safety review.
- **Verification command:** sourced mapping/runtime build followed by CTest and
  `python3 -m pytest -q tools/runtime/tests`.

### 2026-08-27 — Mapping finalization failure is accounted as failure

- **Owner:** mapping worker and world/execution publication boundary. **Scope:**
  a failed dependent finalization no longer marks the epoch ready or lets the
  worker count the callback as published. The callback records its measured
  envelope and then fail-stops because the mutable map cannot roll back safely.
- **Safety impact:** fail-closed safety and truthful lifecycle accounting. The
  new world is not visible when its execution certificate transition fails.
- **Derivation and cost:** one exception path and one duration write; no new
  behavior parameter or relaxed gate.
- **Evidence:** mapping/runtime build and CTest plus injected finalization-failure
  coverage are required.
- **Removal condition:** none. Never report a non-published snapshot as ready.
- **Verification command:** sourced mapping/runtime build followed by CTest and
  `python3 -m pytest -q tools/runtime/tests`.

### 2026-08-27 — Public estimator epoch is tied to reset and process instance

- **Owner:** FAST-LIO public output contract. **Scope:** seed the public epoch
  from the monotonic host steady-clock process instance and advance it when the
  estimator pipeline generation changes after a full reset. Both RegisteredScan
  and EstimatorHealth continue to read the same owner.
- **Safety impact:** prevents a restarted estimator with a reset source clock or
  scan sequence from being relabeled as the active localization epoch. Existing
  stale commands and maps are invalidated at the runtime boundary.
- **Derivation and cost:** one monotonic-clock read at startup and one generation
  comparison per processed result; no sensor timestamp offset is introduced.
- **Evidence:** FAST-LIO public-generation tests, build and cross-epoch replay
  with Health-before-Scan and Scan-before-Health ordering are required.
- **Removal condition:** none. Do not derive localization epoch from ROS/sensor
  time or reset it to a fixed literal on producer restart.
- **Verification command:** sourced FAST-LIO/runtime build and CTest followed by
  recorded reset/replay and repeated SITL validation.

### 2026-08-27 — Transient world evidence gaps remain recoverable

- **Owner:** navigation runtime world-freshness gate. **Scope:** stale/missing
  world evidence clears command exposure and cancels the active solve without
  setting the terminal planner-request latch. A later fresh snapshot can drive
  a new solve for the still-active goal; localization epoch reset uses the same
  recoverable barrier and resets the old execution lease.
- **Safety impact:** no stale command can be published; liveness is restored
  without weakening occupancy, UNKNOWN or execution-state certificates.
- **Derivation and cost:** no new threshold or parameter; only separates the
  recoverable world-evidence state from terminal planner/lease failures.
- **Evidence:** runtime interleaving tests must prove stale→fresh recovery and
  that no old command is exposed during the gap.
- **Removal condition:** none. If recovery is changed to terminal, update this
  contract and its mission-level handover evidence first.
- **Verification command:** sourced runtime build/CTest and stale-world race
  replay followed by repeated SITL.

### 2026-08-27 — Mapping history retention includes the current record

- **Owner:** mapping provenance history. **Scope:** retain at most 256 total
  newest-first change records, including the record for the current revision;
  missing or non-contiguous history still invalidates a stale certificate.
- **Safety impact:** bound and provenance clarity only; no stale certificate is
  newly admitted.
- **Derivation and cost:** one subtraction from the existing retention limit;
  memory remains bounded and no runtime parameter is added.
- **Evidence:** rollover test beyond 256 updates and boundary stale-candidate
  checks are required.
- **Removal condition:** none. Keep fail-closed behavior when an interval falls
  outside retention.
- **Verification command:** sourced mapping build/CTest and long-run provenance
  replay.

### 2026-08-27 — Role diagnostics do not default gaps to MAIN

- **Owner:** planner facade product output. **Scope:** the diagnostic role
  evaluator returns EMERGENCY for invalid times or role gaps, and uses the same
  half-open terminal convention as the validator. It no longer masks malformed
  role metadata as MAIN.
- **Safety impact:** observability and boundary consistency; executable command
  admission remains governed by the independent complete schedule validator.
- **Derivation and cost:** bounded scan over the existing role intervals; no new
  gate or parameter.
- **Evidence:** facade role translation tests and existing planner CTest are
  required.
- **Removal condition:** none. Do not reintroduce an optimistic MAIN fallback.
- **Verification command:** sourced planner backend build followed by CTest.

### 2026-08-27 — Mapping observation owns its immutable cloud and frame contract

- **Owner:** product mapping boundary. **Scope:** MappingObservation now moves a
  unique immutable cloud into the sole mapping actor; the actor rejects missing
  frame IDs and, when supplied by runtime, requires the configured world/body
  frame pair.
- **Safety impact:** prevents producer aliases from mutating admitted evidence
  and prevents a frame-mismatched cloud/pose from entering the mutable map.
- **Derivation and cost:** one ownership transfer and string comparisons per
  admitted observation; no copy of the point cloud and no tuning parameter.
- **Evidence:** mapping actor frame/ownership regressions and runtime CTest are
  required.
- **Removal condition:** none. Do not restore shared mutable cloud ownership at
  the product boundary.
- **Verification command:** sourced mapping/runtime build followed by CTest.

### 2026-08-27 — Registered cloud serialization is performed once

- **Owner:** FAST-LIO output publisher. **Scope:** construct one registered
  PointCloud2 message and reuse it for optional visualization publication and
  the atomic RegisteredScan message.
- **Safety impact:** observability/performance only; typed mapping content is
  unchanged and remains the authoritative admission path.
- **Derivation and cost:** removes one O(points) conversion/allocation per
  corrected scan when both outputs are enabled.
- **Evidence:** FAST-LIO build/tests and repeated dataset callback timing are
  required; raw visualization output is not a mapping authority.
- **Removal condition:** none. Keep the raw topic optional and separate from
  the typed runtime contract.
- **Verification command:** sourced FAST-LIO build/CTest and dataset timing
  replay.

### 2026-08-27 — Disabled product CLI controls are removed

- **Owner:** runtime validation tooling. **Scope:** remove the public
  `frontier-debug` option and `_mapping_params` arguments that were silently
  ignored (`interactive`, `simulation`, `obstacle_evidence`). Mapping profile
  generation now has one explicit input contract.
- **Safety impact:** false-control-surface removal only; no product behavior or
  safety gate is relaxed.
- **Derivation and cost:** removes dead argument plumbing and misleading tests;
  no runtime allocation or branch is added.
- **Evidence:** full Python runtime contract tests are required.
- **Removal condition:** none. Add a CLI option only when it changes a named,
  tested product behavior.
- **Verification command:** `python3 -m pytest -q tools/runtime/tests`.


### 2026-08-27 — Steady-state world snapshots use bounded immutable patches

- **Owner:** navigation mapping snapshot publication. **Scope:** after the
  initial/full snapshot, a non-sliding map update exports only the affected
  base and inflated index windows and layers that patch over the previous
  immutable snapshot. Patch ancestry is bounded to eight updates; the next
  update flattens to a full snapshot. A map slide, invalid/empty patch or
  whole-world change always uses full export.
- **Safety impact:** no occupancy, UNKNOWN or certificate predicate is
  relaxed. Parent snapshots remain alive and immutable, and patch geometry,
  state domain, identity successor and array sizes are validated before
  publication. Any export/validation exception poisons the mutable actor and
  is handled by the existing fail-stop path.
- **Derivation and cost:** removes repeated O(N) allocation/translation from
  the normal local-update path; the bounded depth prevents unbounded query
  chains. The depth is a structural implementation bound, not a runtime
  tuning control.
- **Evidence:** mapping actor patch-depth and semantic-continuity tests,
  sourced mapping CTest, package build, and repeated callback timing over
  representative SITL/recorded data are required. This entry does not claim
  the performance threshold is certified until those distributions exist.
- **Removal condition:** none. Keep full export as the conservative recovery
  path for map slides, whole-world changes and patch failures.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`, build `rog_map_vendor`/`navigation_mapping`, run their
  sourced CTest suites, then run repeated dataset/SITL timing.

### 2026-08-27 — Remove the unused planner interface from the product package

- **Owner:** `navigation_planning` public contract. **Scope:** remove the
  unused `LocalPlanner`/`local_planner.hpp` abstraction. The package exposes
  value contracts (`PlanningRequest`, `PlanningOutcome`, `CandidateBundle`)
  while the concrete planner entry point is the installed
  `navigation_planning_backend::PlannerFacade`; no second virtual interface is
  advertised or installed.
- **Safety impact:** architecture clarity only. No runtime path, planner
  predicate, fallback, timing budget or parameter is changed.
- **Derivation and cost:** repository-wide reference search found no production
  or test consumer of the header; deleting it removes a dead public API and
  prevents a future implementation from diverging from the facade contract.
- **Evidence:** `rg` reference audit, C++20 planning/backend build and focused
  planning CTest are required.
- **Removal condition:** none. Add a new abstraction only when a real second
  implementation needs the same contract and its ownership/test boundary is
  specified first.
- **Verification command:** source the ROS/workspace overlays, build
  `navigation_planning` and `navigation_planning_backend`, then run their
  sourced CTest suites.

### 2026-08-27 — Remove reference-comparison code from product tooling

- **Owner:** product maintenance tooling. **Scope:** remove the executable
  parity checker that encoded the historical reference repository's directory
  and symbol names. The source tree keeps the provenance/license record in
  documentation only; build and runtime code no longer depends on a reference
  checkout or a versioned implementation comparison.
- **Safety impact:** tooling and vocabulary cleanup only. No planner source,
  gate, fallback, parameter or validation predicate is changed by this removal.
- **Derivation and cost:** repository search found no build, test or runtime
  consumer of the checker. Removing it prevents accidental reintroduction of a
  second implementation authority and reduces maintenance surface.
- **Evidence:** reference search, Python tooling tests and product source-name
  audit are required. Provenance documents remain intentionally reviewable.
- **Removal condition:** none. Any future conformance tool must compare
  explicit product contracts and must not become a runtime/source dependency.
- **Verification command:** `python3 -m pytest -q tools/runtime/tests` and a
  product-source search excluding vendor/provenance documentation.

### 2026-08-27 — Remove a duplicated CIRI endpoint correction

- **Owner:** planner corridor geometry. **Scope:** remove the identical second
  endpoint-`b` correction in the zero-radius numerical branch. Each endpoint is
  now corrected at most once before the tangent plane is emitted.
- **Safety impact:** numerical determinism and bounded work only. The branch
  keeps the same finite/degenerate rejection checks and does not relax corridor,
  world, dynamic or backup validation.
- **Derivation and cost:** the second block had the same predicate and formula,
  so it could only repeat the same calculation or amplify round-off; removing it
  reduces redundant work and makes the endpoint contract explicit.
- **Evidence:** CIRI geometry regression, backend CTest and source review are
  required.
- **Removal condition:** none. Do not restore duplicated numerical branches;
  express repeated endpoint handling as a named bounded operation if needed.
- **Verification command:** sourced `navigation_planning_backend` build followed
  by its CTest suite.

### 2026-08-27 — Require typed health before PX4 accepts state or commands

- **Owner:** PX4 External Mode input boundary. **Scope:** propagated odometry
  now passes the actual typed-health readiness state to the source-time epoch
  barrier; navigation commands require both a current healthy health sample and
  a matching public epoch before they can be cached.
- **Safety impact:** closes a pre-health admission path. A queued or early
  untagged `nav_msgs/Odometry` sample can no longer establish the execution
  state before the typed estimator contract; pre-health commands are discarded
  and the existing stationary/hold behavior remains authoritative.
- **Derivation and cost:** this is the existing health/epoch contract, not a
  new parameter. It adds one boolean conjunction per input callback and avoids
  treating the absence of health as an implicit valid epoch.
- **Evidence:** typed contract tests, PX4 External Mode CTest and loaded
  epoch-transition replay are required.
- **Removal condition:** none. Do not make health optional unless the state
  message itself carries an equivalent producer-owned epoch and validity
  certificate.
- **Verification command:** sourced PX4 External Mode build followed by its
  CTest suite and repeated epoch-reset replay.

### 2026-08-27 — Propagated odometry carries its producer-owned identity

- **Owner:** FAST-LIO propagated-state ROS contract and its runtime/PX4
  consumers. **Scope:** replace the untagged `nav_msgs/Odometry` propagated
  topic with `navigation_contracts/msg/PropagatedOdometry`, carrying the
  producer-owned `localization_epoch` and monotonic per-epoch `sequence`.
  The nested odometry header remains the sole source timestamp/frame payload.
  Remove the obsolete source-time barrier helper; `EstimatorHealth` retains
  its last propagated stamp for diagnostics only.
- **Safety impact:** closes the reset ambiguity that allowed a delayed
  untagged sample to be relabeled as the new estimator epoch. Runtime and PX4
  now reject zero/regressed epoch or sequence and require the existing health
  contract at the PX4 execution boundary. This does not certify the complete
  continuous backup contingency.
- **Derivation and cost:** one typed message identity check and one sequence
  comparison per consumer callback; no new parameter, timestamp offset or
  safety relaxation.
- **Evidence:** typed contract test, affected package build/CTest and
  repeated epoch-reset replay are required; SITL remains required for E2E
  continuity.
- **Removal condition:** none. Do not restore an untagged propagated-state
  topic or infer producer identity from source time alone.
- **Verification command:** source ROS/workspace overlays, build
  `navigation_contracts fast_lio_ros navigation_runtime
  px4_navigation_external_mode px4_odometry_bridge`, run their CTest suites,
  then run recorded reset replay and repeated SITL.
