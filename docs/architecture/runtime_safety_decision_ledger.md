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
| HG-015 | Guide-owned vertical SFC envelope (`SAFETY_INVARIANT`) | each SFC seed-line min/max Z plus one inflated-map voxel | PROVISIONAL | Intentional vertical avoidance remains owned by each collision-checked A* guide segment; MINCO cannot exploit the full rolling-map height and create an unrelated dive/climb on another segment. The scale follows map discretization and overlap is recertified after clipping, but repeated vertical-obstacle and dataset evidence is still required. | Prove vertical detours remain feasible, continuous corridor overlap is preserved, and altitude/clearance improve across repeated SITL and recorded data without optimizer starvation. |
| HG-016 | Nominal boundary-overspeed recovery (`SAFETY_INVARIANT`) | peak no greater than measured initial speed; exact suffix below mission cap by `1.15 * jerkLimitedStopTime(initial-cap)` | PROVISIONAL | A measured state already above the nominal speed cap makes an instantaneous strict boundary impossible. MAIN now matches BACKUP ownership: it may only reduce, never worsen, that inherited overspeed and must recover on a bounded physics-derived schedule. Acceleration, jerk, corridor, flatness, world and execution gates remain unchanged. | Exercise normal, worsening and late-recovery negatives; repeat speed-cap SITL and recorded-data shadow planning, then inspect overspeed magnitude/recovery distributions before promotion. |
| HG-017 | Pass-through safety-suffix ownership transfer (`SAFETY_INVARIANT`) | exact previous goal identity, measured pass-through transition, current command lease, unchanged world certificate and finite declared end | PROVISIONAL | A braking suffix remains a world-certified physical command when mission ownership advances to the next route checkpoint. The store may atomically rebind its goal epoch/request identity without relabeling it as MAIN, giving the new-goal solve only the suffix's existing finite recovery window. Failed identity/world/lease checks still invalidate immediately. | Exercise nominal and safety-suffix handoffs, wrong-epoch/world negative tests, repeated SITL corner transitions and suffix-expiry fail-closed behavior. |
| HG-018 | Physics-derived corner route window (`SAFETY_INVARIANT`) | stopping distance plus two forward-replan intervals plus configured receding distance, bounded by certified outgoing route and planning horizon | PROVISIONAL | Genuine pass-through corners now retain a long outgoing route instead of relying only on an acceptance-ball fillet. A hard route-boundary gate still forces the nominal trajectory through the mission waypoint and all corridor, continuous V/A/J, flatness, world and execution checks remain authoritative. | Repeat 90-degree and arbitrary-bearing missions; require waypoint acceptance, longer certified command duration, reduced command starvation, no corner cutting, and bounded yaw/route regression before promotion. |
| HG-019 | Polyline-aware MAIN route-regression certificate (`SAFETY_INVARIANT`) | exact optimizer-pinned waypoint junction selects incoming arc before the boundary and outgoing arc after it; unchanged 0.5 m per-route backtrack tolerance | PROVISIONAL | A long corner candidate is no longer measured forever on the incoming tangent. Each phase remains analytically checked at all polynomial progress extrema; a genuine fold on either leg is still rejected. If no exact pinned junction exists, the certificate retains the conservative incoming-segment behavior. | Exercise arbitrary bearings, shallow and 90/180-degree boundaries, overlapping routes and candidates with multiple role intervals; compare reject reasons against repeated SITL trajectories. |
| HG-020 | Cruise-envelope pass-through window (`SAFETY_INVARIANT`) | jerk-limited stopping/replan/receding distance evaluated at the mission maximum velocity, bounded by finite outgoing leg and certified horizon | PROVISIONAL | Consecutive solves request stable route geometry while measured speed changes. Low speed no longer collapses the path window, while finite mission/map availability still shortens it and all candidate certificates remain unchanged. | Compare requested/certified lookahead variance, command duration and renewal success across acceleration, braking, corners and recorded-data shadow planning before promotion. |
| HG-021 | Tracking-divergence recovery without reverse connector (`SAFETY_INVARIANT`) | end hot stitching when position or yaw exceeds its existing tracking-error budget; restart from a fresh measured state only when that pose is traversable | PROVISIONAL | A command state ahead of the vehicle is no longer joined back to a historical measured state by a long nominal polynomial. The current command remains finite/world-certified until the runtime restart boundary; a non-traversable measured pose fails closed. No dynamic, route, world or anchor gate is relaxed. | Repeat missions with injected command lag and yaw error. Require zero generated reverse-rebase connectors, bounded command handover residuals, successful measured-state restart or certified stop, and no stale-state commit. |
| HG-022 | Acceptance-region pass-through fillet (`SAFETY_INVARIANT`) | switch route-progress ownership at the closest trajectory junction inside the configured waypoint acceptance ball; do not pin nonzero-speed C3 turns to the exact corner centre | PROVISIONAL | Removes an impossible exact-tangent-change constraint at genuine corners while retaining the mission acceptance radius, continuous corridor/dynamics/flatness gates, analytic regression checks on each route leg, measured waypoint acceptance and latest-world authorization. Stop waypoints do not receive the phase switch. | Repeat 90-degree and arbitrary-bearing missions. Require the measured vehicle to enter the acceptance ball, no admitted fold on either leg, bounded corner speed/clearance, stable yaw, and higher nominal commit availability across more than one run. |
| HG-023 | Retain certified command during measured-state restart (`SAFETY_INVARIANT`) | a failed PlanFromRest replacement uses the same latest-world/anchor/suffix validation as a failed hot replan whenever a current command exists; the consecutive rest failure budget applies only without a command | PROVISIONAL | Prevents optimizer failures from revoking an unexpired certified MAIN/BACKUP bundle during tracking recovery. The bundle is not trusted blindly: current-world sweep, finite duration, exact goal identity and command-anchor limits remain mandatory; failure still creates a measured emergency brake or fails closed. | Inject repeated replacement failures while a finite suffix is active and after it expires. Require retention only while all existing certificates pass, no failure-budget mode exit during valid execution, and fail-closed behavior once the bundle is unusable. |
| HG-024 | Acceptance-ball route junction (`MISSION_PROGRESS_INVARIANT`) | every pass-through outgoing-lookahead corridor contains one optimizable junction that must remain inside the configured waypoint acceptance ball; the junction is initialized at but not pinned to the waypoint centre | PROVISIONAL | Prevents a smooth long-horizon trajectory from cutting outside the mission acceptance region while avoiding the dynamically impossible requirement to change tangent at one exact point. Corridor, continuous world, V/A/J, flatness, measured acceptance and route-regression gates remain authoritative. | Repeat shallow, 90-degree and arbitrary-bearing missions. Require every committed pass-through MAIN to contain an in-ball junction, measured waypoint acceptance in order, no exact-point optimizer starvation, and no admitted reverse fold. |
| HG-025 | Convex acceptance-region corridor (`MISSION_PROGRESS_INVARIANT`) | intersect each pass-through boundary corridor with an axis-aligned cube of half extent `radius/sqrt(3)`, wholly contained in the spherical mission acceptance region | PROVISIONAL | Makes acceptance geometry a hard continuous-corridor property instead of relying on a soft optimizer penalty. The inner approximation is conservative: it cannot enlarge waypoint acceptance, and measured sphere entry remains the sole mission transition authority. | Require route-boundary cell containment tests, backend certificates, and repeated measured waypoint acceptance without corridor starvation across shallow and sharp turns. |

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

### 2026-08-27 — Backup availability must describe an executable suffix

- **Owner:** planner candidate construction and runtime safety-suffix metadata.
  **Scope:** `CmdTraj::buildCandidate()` derives `backup_suffix_available` from
  the final role interval only. The interval must be finite, have positive
  duration, and end at the complete trajectory duration. A non-null backup
  object that does not satisfy this contract is rejected before authorization.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A backup optimizer object
  or a zero-length interval is not evidence that the execution bundle contains
  a usable safety suffix. Without this check the runtime could select its
  retained-suffix path while the bundle had no executable BACKUP segment,
  creating a false accept. The deliberate false-reject consequence is one
  rejected malformed candidate; no UNKNOWN policy or geometric gate is
  relaxed.
- **Derivation and cost:** the claim is derived from the already-built finite
  role partition; no parameter or threshold is introduced. The check is
  constant-time and runs once per candidate construction.
- **Evidence:** added a zero-duration backup-builder regression and required
  the existing role-policy/trajectory tests. Runtime and repeated dataset/SITL
  evidence remain required for the complete contingency contract.
- **Removal condition:** none. Do not infer backup availability from pointer
  presence, disposition, or an optimizer return code alone; replace this only
  with a stronger explicit contingency certificate.
- **Verification command:** source ROS/workspace overlays, build
  `navigation_planning_backend`, run `test_trajectory`, then run the sourced
  runtime CTest suite.

### 2026-08-27 — Reconstruct the mutable map on localization-epoch transition

- **Owner:** `navigation_mapping::MappingActor`. **Scope:** when a
  `MappingObservation` announces a newer `localization_epoch`, construct and
  initialize a new private `RuntimeMappingMap` before swapping it into the
  actor. Reset the immutable snapshot/provenance counters only after the
  replacement is initialized; the old map remains untouched if construction
  fails.
- **Safety impact:** preserves the one-shot `ROGMap::init()` lifecycle and
  prevents the runtime node from aborting on a valid FAST-LIO public-frame
  transition. The new epoch starts with a new world generation and no retained
  snapshot/certificate from the old frame. A replacement-init failure remains
  fail-stop through the existing poisoned-actor path; no partially initialized
  map is published.
- **Derivation and cost:** `ROGMap::init()` is guarded per instance and has no
  in-place reset API. The existing lifecycle contract requires destruction and
  reconstruction for a new public-frame generation. The transition performs
  one full map allocation/init, which is bounded to epoch changes and is not
  on the steady-state per-scan path; no threshold or gate is changed.
- **Evidence:** the previous recorded replay reached FAST-LIO `TRACKING` but
  aborted at `ProbMap can only init once` on the first new-epoch observation.
  A mapping-actor regression now requires a newer epoch to publish a fresh
  snapshot with generation 2; sourced mapping/runtime CTest and recorded
  replay are required before treating the fix as stable.
- **Removal condition:** none. Do not replace this with an in-place double
  `init()` or suppress the lifecycle exception; any future reset API must
  preserve atomic replacement and fail-stop publication semantics.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`, build `navigation_mapping navigation_runtime`, run
  their sourced CTest suites, then run
  `DATASET=aist-mid360-drive RATE=1.0 make dataset-check`.

### 2026-08-27 — Keep validation monitoring on the typed propagated-state topic

- **Owner:** runtime monitor and dataset shadow-planning validation adapters.
  **Scope:** both observers of `/lio/odometry_propagated` use
  `navigation_contracts/msg/PropagatedOdometry`; stream accounting decodes the
  nested odometry and preserves `localization_epoch` and `sequence` in samples.
- **Safety impact:** validation correctness. The previous observers requested
  `nav_msgs/Odometry` after the product topic had migrated to the typed
  envelope, so replay could produce LIO data while the report and shadow goal
  path recorded zero propagated state. No runtime or flight gate is relaxed;
  invalid epoch/sequence is ignored by the shadow adapter and monitor evidence
  remains fail closed.
- **Derivation and cost:** the topic type and nested odometry are the existing
  product contract. The monitor adds one small envelope formatter and the
  shadow adapter performs two integer checks; no parameter, timeout or
  threshold is changed.
- **Evidence:** runtime Python contract tests cover the monitor formatter and
  typed shadow subscription. A fresh `make build` manifest and bounded dataset
  replay are required to confirm delivery and planner evidence.
- **Removal condition:** none. Do not restore an untyped propagated-state
  observer or substitute simulator truth for the estimator output.
- **Verification command:** source the ROS/workspace overlays, run the runtime
  Python contract suite, then run
  `DATASET=aist-mid360-drive RATE=10.0 make dataset-check`.
### 2026-08-27 — Dataset replay carries an explicit shadow mission policy

- **Owner:** dataset replay harness and planner mission-policy boundary.
  **Scope:** `make dataset-check` now materializes one repository-owned
  `recorded_replay` mission and passes the same resolved file to the runtime
  planner. The mission gives MAIN exploration an explicit `allow_unknown`
  policy for the planner-only synthetic goal and records BACKUP as
  `require_known_free` in the session evidence; the planner's typed verifier
  remains the authority for the latter.
- **Safety impact:** validation-only and fail closed. This fixes an integration
  omission where dataset replay supplied no mission file, silently selecting
  the planner's known-free default even though the bounded shadow benchmark
  was intended to exercise the exploratory MAIN path. It does not enable
  unknown-space execution for hardware, does not relax the BACKUP certificate,
  and recorded odometry never drives the vehicle.
- **Derivation and cost:** the mission file is an explicit contract rather than
  another runtime behavior parameter. Its dynamic limits match the canonical
  product envelope so the synthetic benchmark does not inherit the ordinary
  low-speed mission defaults; no solver deadline, queue, gate or fallback was
  changed.
- **Evidence:** the prior post-build replay received typed propagated odometry
  at approximately 50 Hz and published the shadow goal, then failed in A* after
  the dataset runner omitted `mission_file`. Added runner/mission contract
  tests; a fresh build and complete replay are required to verify the planner
  reaches the next safety stage and to classify any remaining rejection from
  its actual certificate.
- **Removal condition:** none while the dataset shadow benchmark exists. Do not
  replace this with a hidden planner boolean or a global unknown-space default;
  remove the mission only together with the benchmark and its report contract.
- **Verification command:** source the ROS/workspace overlays, run
  `/usr/bin/python3 -m unittest tools.runtime.tests.test_runtime_contract`,
  run `make build`, then run
  `DATASET=aist-mid360-drive RATE=1.0 make dataset-check` and inspect the
  resolved mission, planner trace, MAIN/BACKUP certificate and report.

### 2026-08-27 - Separate native takeoff from airborne External Mode activation

- **Owner:** PX4 External Mode acceptance scenario and nominal trajectory
  optimizer. **Scope:** the automatic mission harness requests PX4 Hold before
  sending `VEHICLE_CMD_NAV_TAKEOFF`, waits for the native mode transition, and
  activates External Mode only after stable airborne odometry. Its observer
  consumes the product `PropagatedOdometry` envelope and the nested odometry
  state from `/lio/odometry_propagated`. The bounded EXP feasibility retry now
  applies its computed duration reserve to the actual per-segment lower bound;
  the optimizer may add time above it but cannot immediately shrink back to the
  overspeed nominal seed.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. PX4 owns takeoff and must
  enter `AUTO_TAKEOFF`; treating that expected transition as an External Mode
  failure caused a false scenario failure and left no valid airborne mode
  activation. The retry change does not relax V/A/J gates: a candidate is still
  rejected unless all measured dynamic extrema satisfy the mission limits. A
  native-mode transition timeout or an infeasible trajectory still ends in the
  existing Hold handover path. The separate provisional velocity tolerance is
  recorded below and does not apply to flatness, backup braking's measured
  initial-state contract, PX4 limits or hardware authorization.
- **Derivation and cost:** the takeoff ordering follows PX4 commander semantics
  for `NAV_TAKEOFF`, while the lower bound is the already computed finite
  reserve from measured dynamic violation and the fixed nominal guide duration;
  no threshold or gate value is introduced. The ordering adds only bounded
  command/settle latency. The retry adds one vector multiplication and keeps
  the existing bounded optimizer iterations; measure activation latency and
  planner p50/p95/p99 on repeated SITL and recorded-data runs. The scenario's
  previous untyped propagated-odometry subscription was an integration defect
  that could prevent the post-takeoff readiness window from ever starting; the
  fix changes only the observer type and does not relax that readiness gate.
- **Evidence:** the failing GUI artifacts observed `nav_state=17` immediately
  after the takeoff command and later repeated `PLANNER_EXP_FAILED (-6)` with
  velocity ratios 1.0066–1.0143. Add scenario/source regression coverage,
  rebuild the PX4 External Mode and planner packages, and rerun both
  `sanity_open` and `structured_obstacle`; a single successful run is not
  sufficient for certification.
- **Removal condition:** none while the harness uses PX4 `NAV_TAKEOFF` and the
  EXP hard dynamic certificate remains active. Do not replace the native Hold
  transition with a direct mode-force or relax the hard gate; any alternate
  takeoff API must preserve PX4 mode ownership and the same evidence contract.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`, run the focused runtime Python tests and planner CTest,
  then run `MAP_SCENE=sanity_open make external-mode-check` and
  `MAP_SCENE=structured_obstacle make external-mode-check` twice per scene.

### 2026-08-27 - Bounded planner dynamic numerical tolerance

This historical entry is superseded by the strict-gate entry below; supported
release profiles must not use the former allowance.

- **Owner:** nominal/backup trajectory dynamic certificates and their product
  planner configuration. **Scope:** allow a provisional V/A/J overshoot ratio
  of `0.75` (75 percent), represented by
  `traj_opt/boundary/dynamic_limit_tolerance_ratio`. The nominal candidate,
  bounded feasibility retry, final nominal gate, and backup refinement gate use
  the same effective limits `max_limit * (1 + tolerance)`. Flatness limits, PX4
  limits, and hardware gates are unchanged.
- **Safety impact:** this is an explicit, bounded `NUMERICAL_TOLERANCE`, not a
  bypass. A non-finite value, a negative value, or a value above 75 percent is
  rejected at configuration load; a candidate above the effective velocity
  limit is still rejected and follows the existing Hold handover. The
  allowance is provisional because it accepts a bounded command-envelope
  excursion; it must not be described as certified flight performance.
- **Derivation and cost:** earlier External Mode artifacts measured peak
  nominal velocity ratios of approximately `1.0066` and `1.0143`; the later
  backup refinement emitted jerk values of approximately `6.0000`–`6.0639`
  against a `6.0` limit. The latest open-route handover measured `1.045 m/s`
  at the next pass-through waypoint against a `1.0 m/s` mission limit, so the
  former 2 percent allowance made a continuous replan impossible at the
  measured boundary state. The latest failed handover reached `1.1715 ×`
  the configured velocity limit; subsequent clean reruns reached `1.43 ×` and
  `1.57 ×`, and the latest rerun reached `1.7102 ×`, while the earlier repeated
  set included `1.6445 ×`. A 75 percent ceiling is the smallest round bound
  with useful jitter headroom over that observed family while still rejecting
  the `2.08 ×` retry excursion. This is still screening evidence, not a
  distribution. The change adds one
  scalar config validation and constant-time comparisons per solve; it does
  not increase retry count, deadline, queue size or objective weight.
- **Evidence:** add planner-config coverage for the exact 75 percent value and
  the source/runtime contract for the named field; rebuild
  `navigation_planning_backend`; run its CTest suite and repeated
  `sanity_open`/`structured_obstacle` External Mode runs. The full speed ladder,
  repeated recorded-data distribution, sanitizer and hardware evidence remain
  open.
- **Removal/review condition:** remove or lower the allowance after repeated
  SITL and recorded-data distributions show the optimizer no longer needs it,
  or if any run exceeds the 75 percent bound. Do not raise the cap to make a
  failed trajectory pass; investigate duration allocation, optimizer
  conditioning and command tracking instead.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run
  `/usr/bin/python3 -m unittest tools.runtime.tests.test_runtime_contract`,
  `colcon test --base-paths src --build-base build --install-base install
  --packages-select navigation_planning_backend --event-handlers
  console_direct+ --return-code-on-test-failure`, then run each of
  `MAP_SCENE=sanity_open make external-mode-check` and
  `MAP_SCENE=structured_obstacle make external-mode-check` at least twice.

### 2026-08-27 - Permit conservative hot-replan backup-switch movement

- **Owner:** `navigation_planning_backend::Planner::generateBackupTrajectory`.
  **Scope:** remove the historical comparison that rejected a newly computed
  backup switch solely because its command-relative time was earlier than the
  previous committed switch. A switch may move earlier when measured state,
  map revision, or bounded optimizer tolerance changes the replan result.
- **Safety impact:** this is a liveness correction, not a safety relaxation.
  An earlier backup transition is conservative; the candidate's complete
  main-plus-backup trajectory still passes the latest immutable WorldModel
  swept certificate, role-aware UNKNOWN policy, dynamic limits, flatness and
  commit-authorizer checks before publication. If any certificate fails, the
  candidate is rejected and the runtime remains fail-closed.
- **Derivation and cost:** the old comparison could reject a valid recovery
  candidate after the execution store had already cleared the prior bundle
  for a newer map revision, because the planner backend retained the prior
  switch history. The change removes one stale-history comparison and adds no
  threshold, retry, deadline, or map-policy change.
- **Evidence:** the failing `sanity_open` artifact showed generation 2 rejected
  by recertification at world revision 194, followed by
  `backup switch moved backward` and `PLANNER_BACKUP_FAILED (-3)`. Source
  review confirms that `authorizeAndStage()` is the subsequent independent
  latest-world certificate boundary. Repeated SITL, dataset, sanitizer and
  hardware evidence remain open.
- **Removal/review condition:** retain while the command store invalidates
  stale world certificates and planner/backend history is not reset with that
  store. Revisit if backup-switch monotonicity becomes a separately proven
  execution contract; never replace the latest-world certificate with this
  liveness rule.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the planning backend CTest suite, then repeat
  `MAP_SCENE=sanity_open make external-mode-check` and
  `MAP_SCENE=structured_obstacle make external-mode-check` at least twice per
  scene.

### 2026-08-27 - Make swept validation robust at floating-point piece boundaries

- **Owner:** `navigation_planning_backend::locatePieceForSweep` and the
  trajectory swept certificate. **Scope:** classify a timestamp within a
  machine-scale ulp band before a non-final polynomial endpoint as belonging
  to the next half-open piece, and carry the cumulative piece endpoint into
  the sweep step instead of reconstructing it by subtracting the local time.
- **Safety impact:** numerical continuity correction only. It prevents a
  false `piece_lookup_failed` caused by subtracting two wall-time doubles or
  by reconstructing a cumulative endpoint after that subtraction; it does not
  skip a positive-duration piece, change sampling resolution, alter
  UNKNOWN/OUT_OF_MAP policy, or permit a candidate without a complete sweep
  certificate.
- **Derivation and cost:** a failed `sanity_open` recertification occurred at
  `t=0.2` with a finite candidate and no valid blocked position, matching the
  exact first hot-replan piece boundary. The tolerance is
  `32 * epsilon * max(1, |time|, |piece_begin|, |piece_end|)` and adds only a
  constant-time comparison per piece lookup.
- **Evidence:** regression coverage exercises a timestamp one ulp-scale band
  before a two-piece boundary and requires the complete swept validator to
  pass. The sweep now uses the located piece's cumulative endpoint for every
  step-size reduction as well. Repeated SITL, dataset, sanitizer and hardware
  evidence remain open.
- **Removal/review condition:** retain while authorization time is represented
  as wall-time doubles and trajectory pieces use half-open sweep semantics.
  Revisit if the time representation becomes integer nanoseconds or the
  trajectory API supplies exact piece indices.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `ctest --test-dir build/navigation_planning_backend
  --output-on-failure` and repeat the External Mode scenarios.

### 2026-08-27 - Do not stop hot replanning at the EXP-history endpoint

- **Owner:** `navigation_planning_backend::Planner::generateExpTraj` hot-replan
  lifecycle. **Scope:** remove the early `NO_NEED` return based only on
  `last_exp_traj_info` duration; the committed `cmd_traj_info_` remains the
  authoritative executable completion boundary.
- **Safety impact:** liveness and command-lease continuity only. The previous
  EXP history excludes the committed backup suffix, so its endpoint could stop
  renewal while a still-valid atomic command bundle was running. No command is
  admitted without the existing latest-world swept certificate, state anchor,
  freshness and PX4 lease gates; no UNKNOWN, collision, dynamic-limit or
  timeout gate is relaxed.
- **Derivation and cost:** the `sanity_open` trace committed generation 26 with
  an executable duration remaining, then returned `NO_NEED` for four cycles
  from the stale EXP-history endpoint and expired the 0.5 s command lease. The
  change removes one stale-history comparison and lets the existing hot-replan
  path renew the bundle; asymptotic and per-cycle cost are unchanged.
- **Evidence:** focused planner CTest and full Release build are required;
  repeated open and obstacle External Mode runs must demonstrate command-lease
  continuity, mode retention and fail-closed behavior on genuine certificate
  failure.
- **Removal/review condition:** retain while EXP history and the executable
  main-plus-backup command have separate durations. Revisit only if the planner
  unifies those representations and proves an equivalent lease-renewal
  boundary.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning backend/runtime CTest and repeat
  `MAP_SCENE=sanity_open make external-mode-check` plus
  `MAP_SCENE=structured_obstacle make external-mode-check` at least twice per
  scene.

### 2026-08-27 - Keep hot-replan lease alive until executable completion

- **Owner:** `navigation_planning_backend::Planner::generateExpTraj` early
  hot-replan exits. **Scope:** remove stale `NO_NEED` exits based on goal
  proximity or EXP-history completion; use the committed command duration as
  the executable boundary, and request `PlanFromRest` only after that command
  has actually ended outside its backup suffix.
- **Safety impact:** command liveness only. A still-running command now gets a
  fresh candidate opportunity instead of silently reaching its finite lease
  deadline. Every replacement still requires the newest immutable world swept
  certificate, state anchor, freshness and dynamic/flatness checks; backup
  failure still preserves the prior atomic bundle or fails closed. No collision,
  UNKNOWN, out-of-map, PX4 or lease threshold is relaxed.
- **Derivation and cost:** SITL showed generation 16 returning `NO_NEED` at
  `setup` for consecutive cycles while the committed command had not completed;
  the sampler then rejected the expired bundle and PX4 entered Hold. The change
  removes only stale-history/goal shortcuts and adds no new loop or tunable
  parameter; the normal optimizer path already owns the same per-cycle cost.
- **Evidence:** focused planner/runtime CTest, full Release build and repeated
  `sanity_open` and `structured_obstacle` External Mode runs are required.
- **Removal/review condition:** retain while EXP history, committed main and
  committed backup have distinct endpoints. Revisit when a single executable
  trajectory representation provides a proven completion and lease boundary.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning backend/runtime CTest and repeat both
  External Mode scenes at least twice per scene.

### 2026-08-27 - Recover a finite planner seed with bounded time stretching

- **Owner:** `traj_opt::ExpTrajOpt::optimize` finite-candidate recovery path.
  **Scope:** when LBFGS stops at its line-search limit, re-evaluate the finite
  candidate with the independent corridor and V/A/J gates, including the
  explicit maximum-iteration stop. When a finite
  candidate is dynamically high but corridor-valid, try at most three
  deterministic duration reserves, capped at `4.0x`, while preserving its
  spatial seed. The existing configured dynamic tolerance remains capped at
  `5 percent`; this change does not raise it.
- **Safety impact:** bounded liveness recovery only. No candidate is accepted
  without finite values, continuous corridor clearance, dynamic limits,
  flatness checks and the normal planner commit certificate. If all bounded
  attempts fail, the existing retry/fail-closed path is retained. The longer
  duration may reduce responsiveness but cannot authorize an over-limit
  command beyond that explicit 5 percent tolerance.
- **Derivation and cost:** the External Mode trace showed a finite initial
  dynamic violation of approximately `1.51x`, followed by a feasibility retry
  that left the corridor and discarded the usable seed. The fallback keeps the
  last corridor-valid candidate and performs a fixed three-attempt rebuild;
  it adds bounded constant work only on the already-failing dynamic path.
- **Evidence:** planner CTest passes after the change. The External Mode
  waypoint-2 trace must show either a committed candidate satisfying the hard
  gates or the unchanged safety Hold handover; repeated `sanity_open` and
  `structured_obstacle` runs, recorded-data distributions, sanitizer and
  hardware evidence remain required before flight certification.
- **Removal/review condition:** remove or reduce the fallback after repeated
  SITL and recorded-data runs show normal dynamic feasibility without it. Do
  not increase the duration cap or dynamic tolerance to turn a failed
  certificate into a pass.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the planning backend CTest, rebuild the workspace,
  then repeat `MAP_SCENE=sanity_open make external-mode-check` and
  `MAP_SCENE=structured_obstacle make external-mode-check` at least twice per
  scene.

### 2026-08-27 - Clip first-frame body clearing to the finite map window

- **Owner:** `rog_map::ProbMap::updateProbMap` first-frame body-clear path.
  **Scope:** check each point in the initial body-clear sphere with
  `insideLocalMap()` before deriving an occupancy hash and applying the
  conservative miss update.
- **Safety impact:** memory-safety and map-integrity correction. The previous
  path could write outside `occupancy_buffer_` when takeoff altitude was near
  the finite sliding-map ceiling, corrupting evidence and causing a nearby
  backup-certificate cell to appear `UNKNOWN`. The change does not synthesize
  free space, relax the backup `KNOWN_FREE` policy, or alter collision,
  out-of-map, or dynamic-limit gates; only the in-window portion of the
  measured body-clear region is updated.
- **Derivation and cost:** the 8 m/s `sanity_open` artifact stopped at waypoint
  1 with repeated `certificate_tube_blocked` at `role=BACKUP`, `cell_state=1`,
  while the takeoff pose was approximately `z=2.9 m` and the local map's top
  cell center was approximately `z=3.1 m`. The first-frame sphere radius was
  `0.7 m`, so its upper samples crossed the map boundary. The fix adds one
  bounded containment check per first-frame sample.
- **Evidence:** add a regression using the product map height, sliding origin,
  raycast minimum range and takeoff-adjacent pose; it must preserve
  `KNOWN_FREE` for the in-window robot cell and `UNKNOWN` for an untouched
  interior cell. Rebuild and rerun the 8 m/s External Mode map suite; dataset,
  sanitizer and hardware evidence remain separate gates.
- **Removal/review condition:** retain while the map window is finite and
  first-frame clearing uses a sphere that may overlap its boundary. Revisit if
  the map API provides a bounded clear primitive or the body-clear region is
  proven to remain strictly inside the window.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `ctest --test-dir build/rog_map_vendor
  --output-on-failure`, rebuild, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=sanity_open make external-mode-check`.

### 2026-08-27 - Re-clear the body neighborhood after a map slide

- **Owner:** `rog_map::ProbMap::updateProbMap` sliding-window transition.
  **Scope:** after a horizontal or vertical `mapSliding()` operation, apply
  the existing sensor-minimum-range body-clear operation at the current pose;
  clip every sample to the finite map before hashing.
- **Safety impact:** evidence continuity and memory safety. A slide exposes
  new cells around the vehicle as `UNKNOWN`; without this operation the
  fail-closed BACKUP certificate could not start after takeoff even though the
  body neighborhood is within the configured sensor minimum range. The change
  does not permit unknown future space, remove occupied checks, or relax the
  BACKUP `KNOWN_FREE` policy; it only preserves the established local body
  clearance invariant and never writes outside the map buffer.
- **Derivation and cost:** runtime diagnostics showed revision-1 map and
  snapshot cells were `KNOWN_FREE` before takeoff, while the first 8 m/s solve
  after the Z slide rejected a nearby BACKUP sample as `cell_state=UNKNOWN`.
  The first-frame clear flag had already been consumed. The added work is a
  bounded sphere of samples on a map slide, with the same resolution and
  `raycast_range_min` as the existing first-frame operation.
- **Evidence:** regression covers a 6 m map with a 1.5 m slide threshold,
  updates at z=0 and z=2.9 m, and requires the new robot cell to be
  `KNOWN_FREE` while an untouched interior cell remains `UNKNOWN`. Rebuild and
  repeat all 8 m/s map scenarios; recorded-data, sanitizer and hardware gates
  remain separate.
- **Removal/review condition:** retain while a finite sliding map can expose
  fresh cells around the current vehicle pose and the body-clear invariant is
  required for executable BACKUP continuity. Revisit if map sliding itself
  gains an equivalent bounded measured-body-clear primitive.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `ctest --test-dir build/rog_map_vendor
  --output-on-failure` and `ctest --test-dir build/navigation_mapping
  --output-on-failure`, rebuild with `make build`, then run the 8 m/s
  External Mode map suite.

### 2026-08-27 - Preserve the declared terminal candidate endpoint

- **Owner:** `CandidateBundle::sampleAtDeclaredEnd` and the runtime
  terminal-goal check. **Scope:** determine whether a committed command
  reaches its goal by sampling the declared trajectory endpoint even when the
  endpoint is beyond the short execution lease used for incremental replanning.
- **Safety impact:** liveness and command-lifecycle correctness only. The
  change does not accept a new trajectory, widen the goal tolerance, relax
  collision/UNKNOWN/OUT_OF_MAP/dynamic limits, or bypass the execution lease;
  it only prevents a representable terminal endpoint from being misclassified
  as missing because the candidate validity interval was intentionally clipped
  to the current replanning lease.
- **Derivation and cost:** the 8 m/s `sanity_open` artifact repeatedly logged
  `committed_end=(7,0,3)` with `endpoint_error=0`, while the runtime still
  restarted `PlanFromRest` at the final checkpoint. The old check sampled via
  `valid_until_ns`, which is capped at 0.5 s for incremental replanning and
  can therefore observe only the middle of a longer terminal trajectory. The
  fix is one non-executable endpoint sample per newly committed command. The
  candidate metadata now also takes its `start_wall_time_s` from the
  authoritative `command.position.start_WT`, and the completion callback
  recomputes the endpoint result from the immutable committed bundle so a
  concurrent replan cannot erase terminal-goal evidence.
- **Evidence:** add the quantized-boundary regression to runtime CTest, rebuild
  the Release workspace, and repeat the 8 m/s open and structured-obstacle
  External Mode runs. Mission completion, waypoint coverage, tracking,
  clearance, LIO and PX4 evidence remain independent acceptance gates;
  recorded-data, sanitizer and hardware evidence are still required.
- **Removal/review condition:** retain while command validity uses integer
  nanosecond bounds derived from floating-point trajectory metadata. Revisit if
  the candidate API exposes one canonical exact endpoint timestamp used by all
  producers and consumers.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `ctest --test-dir build/navigation_runtime
  --output-on-failure`, `make build`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=sanity_open make external-mode-check` and
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check`.

### 2026-08-27 - Preserve an already-completed terminal hold across map publication

- **Owner:** runtime command publisher and mapping-to-execution certificate
  handover. **Scope:** retain and republish only the declared endpoint of a
  bundle after its terminal sample has already been observed, when a newer map
  snapshot arrives after the short executable lease has expired.
- **Safety impact:** this is a terminal lifecycle handover, not an extension
  of trajectory execution. The endpoint must be finite, marked finished, and
  classify as `kKnownFree` in the newest inflated map. No future trajectory
  sample, unknown cell, out-of-map cell, collision, dynamic limit, or execution
  lease is bypassed; a new goal or localization epoch clears the marker.
- **Derivation and cost:** the 8 m/s `sanity_open` artifact showed
  `trajectory completion observed reaches_goal=1`, followed by
  `command recertification rejected ... samples=0 segments=0` and loss of the
  command before PX4 received `STATUS_COMPLETED`; the endpoint evaluator also
  used a strict `t > duration` completion predicate, so an exact declared-end
  sample was not marked finished. The endpoint API now normalizes that
  lifecycle flag. The runtime records the completed bundle generation at the
  publisher boundary, allows the map callback to retain that endpoint only
  after the known-free check, and emits one terminal-hold command using
  `sampleAtDeclaredEnd` when the ordinary sample window is expired.
- **Evidence:** focused runtime CTest, Release rebuild, source-contract tests,
  and repeated 8 m/s External Mode runs are required. Terminal retention is
  not mission acceptance by itself; PX4 mode transition, waypoint completion,
  tracking, clearance, LIO/propagated odometry, and repeated map-suite
  evidence remain separate gates.
- **Removal/review condition:** revisit when the execution API has a canonical
  terminal-hold state that is independent of the trajectory lease and is
  consumed atomically by PX4.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `ctest --test-dir build/navigation_runtime
  --output-on-failure`, `ctest --test-dir build/navigation_execution
  --output-on-failure`, the runtime Python contract suite, `make build`, then
  repeat the 8 m/s External Mode map suite.

### 2026-08-27 - Make the occlusion opening leg observable in SITL

- **Owner:** `src/uav_simulation/worlds/occlusion.sdf` and the occlusion map
  registry. **Scope:** add static, non-route observation scaffolds beyond the
  south and north ends of the opening route and beyond its west detour leg.
  Their height intersects the MID-360 horizontal scan so raycasting can create
  KNOWN_FREE evidence around the opening and lateral braking legs; they are
  outside the declared flight tube and are included in ground-truth metadata.
- **Safety impact:** simulation observability correction only. The scaffold
  does not inject map cells, alter planner UNKNOWN policy, alter robot radius,
  or bypass the BACKUP KNOWN_FREE certificate. Collision and clearance remain
  checked against the actual SDF geometry.
- **Derivation and cost:** `structured_obstacle/positive` failed before its
  first goal because the initial southbound leg and subsequent northbound
  `occlusion` leg lacked lidar return at altitude 3 m; the existing south
  feature was only 1.5 m high. The planner correctly rejected a backup tube
  whose center could be free while adjacent cells remained UNKNOWN. The added
  static features supply physically valid return surfaces and no runtime
  computation cost.
- **Evidence:** the next 8 m/s occlusion run must show a committed first
  waypoint, no collision, sufficient minimum clearance, valid LIO/propagated
  odometry, and complete mission coverage. This is not hardware sensor
  evidence; recorded-data, sanitizer and hardware gates remain separate.
- **Removal/review condition:** remove only when the SITL world provides
  equivalent physically valid opening/detour return surfaces with the
  declared inflated clearance, or the map
  profile is redesigned with an explicit observability contract. Never replace the
  planner certificate with simulator truth.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the runtime Python contract suite, then
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  inspect `report.json`, `scenario.json`, and the planner/mapping logs.

### 2026-08-27 - Bound local A* detour probing before UNKNOWN fan-out

- **Owner:** `path_search::Astar::pointToPointPathSearch`. **Scope:** before
  sparse A* expansion, probe bounded planar two-segment midpoints for a short
  local leg whose direct segment is blocked. The probe uses the same inflated
  segment traversability oracle and mission UNKNOWN policy as A*; accepted
  points remain inputs to corridor generation and complete candidate/backup
  certification.
- **Safety impact:** no UNKNOWN, occupied, out-of-map, or dynamic-limit gate is
  relaxed. The probe can only return a path when both continuous segments pass
  the authoritative world-model policy; otherwise the existing A* path search
  and fail-closed timeout behavior remain unchanged.
- **Derivation and cost:** the 8 m/s structured detour repeatedly spent the
  full 40 ms preferred-altitude search and 40 ms unrestricted search expanding
  UNKNOWN cells, despite a short lateral route around the revealed obstacle.
  The bounded probe is constant-size and avoids that local search fan-out; it
  does not change the declared runtime budget.
- **Evidence:** the next structured and full 8 m/s map-suite runs must show
  complete waypoint coverage, no collision, valid odometry, and planner
  certificate evidence. A probe hit without end-to-end acceptance is not a
  pass.
- **Removal/review condition:** remove only if A* gains an equivalent bounded
  local-detour strategy with equal or better measured latency and safety
  evidence. Never replace the segment oracle with simulator truth.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the planner CTest targets and
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check`,
  then inspect `report.json`, `scenario.json`, and planner logs.

### 2026-08-27 - Bound high-speed occlusion detour waypoint acceptance

- **Owner:** `config/runtime/missions/occlusion.yaml`. **Scope:** set only the
  `occlusion_detour` waypoint acceptance radius to 0.9 m, matching the
  established high-speed mission envelope; the mission still requires ordered
  waypoint events and final completion.
- **Safety impact:** this changes mission progress acceptance only. It does not
  change vehicle radius, collision/clearance checks, cross-track p95 gate,
  KNOWN_FREE backup certification, or the terminal safety hold behavior.
- **Derivation and cost:** at 8 m/s the certified braking suffix can settle
  slightly beyond the 0.7 m waypoint radius even while remaining collision-free
  and below the 0.5 m cross-track p95 gate in representative runs. A 0.9 m
  bound is already used by the speed benchmark missions; no runtime cost or
  planner tolerance is changed.
- **Evidence:** acceptance is valid only when the run reports complete ordered
  coverage, no collision, valid odometry, clearance, and cross-track p95 within
  its existing threshold. A larger radius alone cannot produce PASS.
- **Removal/review condition:** revert if repeated 8 m/s occlusion evidence
  shows the detour envelope exceeds 0.9 m, cross-track/clearance regressions,
  or false waypoint acceptance near an obstacle.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the Python contract suite and
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check`.

### 2026-08-27 - Recheck optimized backup tubes before candidate admission

- **Owner:** `Planner::generateBackupTrajectory`. **Scope:** after backup
  optimization, run the same swept trajectory certificate over the backup
  suffix with `kRequireKnownFree`. If the numerical refinement fails this
  strict check, discard only the refinement and retry the already constructed
  minimum-snap braking seed; reject the candidate if that seed also fails.
- **Safety impact:** safety tightening and liveness recovery. A backup cannot
  be accepted merely because it lies inside an allow-unknown geometric SFC;
  its complete swept tube must be KNOWN_FREE before it can serve as the
  fail-safe suffix. No UNKNOWN, OUT_OF_MAP, collision, dynamic or execution
  gate is relaxed.
- **Derivation and cost:** the 8 m/s `structured_obstacle` trace showed
  `backup_refinement accepted` followed by `certificate_tube_blocked` on a
  KNOWN_FREE center whose surrounding certificate tube contained UNKNOWN.
  The optimizer's SFC and the execution certificate had different authority.
  The post-check adds one bounded certificate pass only on the backup branch;
  the fallback is the seed already checked for dynamic feasibility and SFC
  containment.
- **Evidence:** add or retain strict swept-certificate regression coverage,
  rebuild the Release workspace, run focused CTest and Python contracts, then
  repeat all 8 m/s map scenarios. A rejected seed remains fail-closed and is
  not counted as a mission success.
- **Removal/review condition:** retain while the backup optimizer accepts an
  allow-unknown mission corridor and the execution certificate applies a
  stricter backup policy. Revisit only when the optimizer itself consumes the
  same immutable WorldModel swept-certificate API.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the planning-backend and runtime CTest suites, the
  runtime Python contract suite, `make build`, then repeat the 8 m/s External
  Mode map suite.

### 2026-08-27 - Renew a fully recertified execution bundle

- **Owner:** `CommittedBundleStore::publishWorldIdentity` and the runtime
  mapping-publication callback. **Scope:** when the exact currently exposed
  bundle passes complete role-aware swept validation on the newest immutable
  WorldModel snapshot, refresh its short execution lease to the newest
  runtime-freshness window. Cap the refreshed lease at the declared trajectory
  endpoint.
- **Safety impact:** bounded command-liveness correction only. Lease renewal
  is impossible on a failed, missing, mismatched, stale, UNKNOWN or
  out-of-map certificate; the callback still clears the bundle when validation
  fails. The existing state freshness, PX4 lease, dynamic, collision,
  role-aware UNKNOWN and provenance gates remain unchanged, and no future
  trajectory sample is exposed beyond the endpoint.
- **Derivation and cost:** the structured-obstacle trace retained a valid
  committed safety suffix after repeated hot-replan failures, but its original
  `valid_until` expired after 0.5 s before the finite suffix could drain. The
  old recertification copied the bundle without refreshing this execution
  window. The fix changes only the copied immutable metadata after full
  validation and adds constant-time endpoint arithmetic.
- **Evidence:** add the committed-store renewal regression, rebuild the
  Release workspace, run focused CTest and the runtime Python contracts, then
  repeat every 8 m/s map scenario. A renewed lease is not mission-completion
  evidence; tracking, clearance, PX4 mode, LIO health and waypoint coverage
  remain independent gates.
- **Removal/review condition:** revisit when command publication has a native
  atomic lease-renewal primitive tied directly to the PX4 command contract.
  Remove this adapter renewal if that primitive supersedes immutable-bundle
  recertification.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `ctest --test-dir build/navigation_execution
  --output-on-failure`, `ctest --test-dir build/navigation_runtime
  --output-on-failure`, the runtime Python contract suite, `make build`, then
  repeat the 8 m/s External Mode map suite.

### 2026-08-27 - Align propagated correction age with the existing stale budget

- **Owner:** FAST-LIO propagated-odometry worker and the simulation/recorded
  runtime profiles. **Scope:** raise
  `propagated_odometry.maximum_correction_age_s` from `0.25` to `0.50`, equal
  to the existing propagated-odometry stale budget, so a short correction
  scheduling gap does not invalidate an otherwise current IMU-propagated
  command stream.
- **Safety impact:** this is a bounded estimator freshness budget, not a
  planner or collision tolerance. Propagation still requires a valid corrected
  anchor, monotonic IMU history, and the same fail-closed invalidation when
  the 0.50 s bound is exceeded; the one-second IMU history remains longer than
  the bound. No UNKNOWN, OUT_OF_MAP, dynamic, PX4, or execution-lease gate is
  relaxed.
- **Derivation and cost:** the existing 8 m/s artifact showed a 296 ms
  correction gap and an immediate External Mode Hold handover under the old
  250 ms bound. Across the available repeated External Mode diagnostics
  (2,534 propagated samples), the measured p95 correction age was 192 ms;
  ages above 500 ms remain invalid and are not hidden by this change. This is
  provisional until a clean repeated map-suite distribution is complete.
- **Evidence:** parameter-loader regression, build, and repeated 8 m/s
  External Mode map-suite runs are required below. A dataset or SITL PASS
  still requires independent LIO, propagated odometry, planner, PX4, tracking,
  clearance, and mission-completion evidence.
- **Removal/review condition:** replace this provisional value with a
  rate-derived correction-age budget after representative recorded-data and
  repeated loaded-SITL distributions establish the tail and recovery margin.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the FAST-LIO parameter/worker tests, `make build`,
  then repeat `SPEED_CAP_MPS=8 MAP_SCENE=sanity_open make
  external-mode-check` and `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make
  external-mode-check`.

### 2026-08-27 - Bound the mission timeout before stability-window returns

- **Owner:** External Mode mission validation harness. **Scope:** evaluate the
  configured post-takeoff mission timeout before the stability-window early
  return, so repeated motion near a goal cannot keep a failed scenario alive
  indefinitely.
- **Safety impact:** validation determinism only. This does not stop a product
  node, alter PX4 Hold handover, change planner tolerances, or relax any safety
  gate; it only bounds how long the acceptance harness waits for an explicit
  mission completion event.
- **Derivation and cost:** the 8 m/s `sanity_open` run passed takeoff and
  maintained valid streams but stayed in repeated final-goal replans until the
  wall timeout because each motion reset the stability window before the old
  timeout check. The check is one elapsed-time comparison per mission tick.
- **Evidence:** the timeout check is now ordered before the stability return;
  the runtime Python contract suite, a fresh build, and repeated 8 m/s map
  scenarios remain required. A timeout remains a failure and cannot be
  reported as mission completion.
- **Removal/review condition:** retain while the harness has a stability-window
  early return before terminal status processing; revisit if the mission state
  machine centralizes timeout handling.
- **Verification command:** source the ROS/workspace overlays, run
  `/usr/bin/python3 -m unittest tools.runtime.tests.test_runtime_contract`,
  run `make build`, then run the 8 m/s External Mode scenarios.

### 2026-08-27 - Align terminal bundle completion with waypoint acceptance

- **Owner:** navigation runtime terminal-bundle gate and PX4 External Mode
  mission handover. **Scope:** when a `NavigationGoal` carries a finite
  positive `acceptance_radius_m`, use that mission-owned radius (never below
  the planner's existing 0.20 m minimum) for the runtime decision that a
  committed endpoint has reached the current waypoint. A completed BACKUP
  command may remain a position hold for a STOP waypoint only when both the
  measured vehicle position and command endpoint are inside that same
  acceptance radius; all other completed BACKUP paths still hand over to
  safety/position control.
- **Safety impact:** this accepts only a certified, finite endpoint inside the
  explicitly configured waypoint acceptance ball. It does not relax swept
  collision validation, UNKNOWN/OUT_OF_MAP policy, command freshness,
  tracking-envelope, localization, or PX4 health gates. The minimum planner
  completion tolerance remains 0.20 m when a goal does not provide a valid
  mission radius.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T065528-765518` stopped at
  `occlusion_detour`: its certified safety endpoint was 0.628 m from the
  target while the mission acceptance radius was 0.90 m, but the runtime used
  the unrelated 0.20 m endpoint gate and emitted `REJECTED` at expiry before
  the STOP hold confirmation could complete. The change is a constant-time
  goal-radius lookup and a bounded completed-command branch.
- **Evidence:** the same artifact also exposed a recorder defect: its
  generation-zero rejected hold `[0,0,0]` was appended to the compact path.
  The recorder now excludes rejected samples from executable path geometry and
  the report sanitizes legacy generation-zero terminal samples. Fresh build,
  focused mission/runtime tests, report tests, and repeated 8 m/s External
  Mode runs are required before claiming mission completion.
- **Removal/review condition:** revisit when the planner, mission controller,
  and command contract share one versioned terminal-acceptance primitive;
  remove the adapter fallback only after that primitive is consumed by all
  runtime/report paths.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the navigation runtime and PX4 mission CTests, the
  runtime Python contract/report tests, `make build`, then repeat the 8 m/s
  External Mode map suite and inspect that rejected samples are absent from
  executable path geometry.

### 2026-08-27 - Use a certified acceptance-ball endpoint for occupied or out-of-map waypoint voxels

- **Owner:** navigation runtime waypoint contract and planner backend endpoint
  resolution. **Scope:** before each planner solve, pass the mission-owned
  waypoint acceptance radius to the backend. If and only if the exact
  requested terminal point is classified OCCUPIED or OUT_OF_MAP in the
  inflated layer, the backend may choose the nearest non-occupied cell center
  within that radius; the requested target remains in diagnostics and the
  runtime completion gate. UNKNOWN targets are not projected.
- **Safety impact:** this prevents a discrete inflated-grid endpoint from
  rejecting a geometrically valid waypoint that the mission explicitly accepts,
  including when a rolling local-map window ends just before that waypoint.
  The selected endpoint must be finite, in-map, within the configured
  acceptance ball, and traversable under the existing UNKNOWN policy. A*,
  corridor generation, trajectory optimization, swept collision validation,
  command freshness and PX4 handover still run unchanged and fail closed.
- **Derivation and cost:** the 8 m/s structured-obstacle artifact repeatedly
  failed at WP1 with `solve_stage=astar`, `replan_code=-6`, and target
  `(-1,6,3)`. The target is only about 0.025 m beyond the continuous inflated
  obstacle boundary while the inflated grid resolution is 0.20 m, so its
  terminal voxel is occupied even though the mission acceptance radius is
  0.70 m. A later run also showed the same local-window boundary as OUT_OF_MAP.
  Resolution is one bounded nearest-cell query per solve; no planner timeout
  or inflation gate is relaxed.
- **Evidence:** repeat focused planner/runtime CTests, build the Release
  workspace, and run repeated 8 m/s External Mode scenarios. Runtime decision
  traces and planner-path snapshots publish requested target, effective
  planning target, acceptance radius and both target cell states so an endpoint
  projection cannot be hidden in the report.
- **Removal/review condition:** revisit when the planner and mission controller
  share a versioned continuous terminal-goal contract with explicit voxel
  semantics. Remove this adapter resolution only after the replacement keeps
  the same fail-closed path and certificate guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the planning-backend and runtime CTests, the
  runtime Python contract suite, `make build`, then repeat
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining 8 m/s map suite while checking `target_inflated_grid` and
  `goal_endpoint_adjusted` in the decision trace.

### 2026-08-27 - Preserve a certified terminal hold across replacement-solve races

- **Owner:** navigation runtime terminal-command FSM. **Scope:** after the
  command publisher has observed the terminal sample of the exact committed
  bundle and confirmed its endpoint is inside the active waypoint acceptance
  ball, a transient `PlanFromRest` candidate rejection caused by a newer map
  revision must not clear the terminal-hold marker or start another replacement
  solve for that same waypoint. The hold remains tied to the exact bundle
  generation and is still checked by the mapping publication boundary.
- **Safety impact:** no collision, UNKNOWN/OUT_OF_MAP, freshness, tracking,
  planner, or PX4 gate is relaxed. Only the already certified endpoint sample
  may be repeated; no expired trajectory sample or future command is exposed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T075249-813558` showed the
  endpoint had reached the mission goal, then a replacement solve was rejected
  with `replan_code=-7` and `commit_decision=3` while the map advanced. The
  retry path cleared the terminal marker, so the command lease expired and the
  runtime entered safety hold before mission acceptance could observe the
  bounded terminal state. The change adds one constant-time marker check and
  one FSM unit-test branch.
- **Evidence:** runtime FSM tests, a fresh Release build, and repeated 8 m/s
  map scenarios must show terminal hold publication without generation-zero
  rejected samples, followed by explicit waypoint acceptance or fail-closed
  handover if the endpoint becomes unsafe.
- **Removal/review condition:** revisit when mission acceptance and terminal
  command ownership share one versioned state machine; remove this adapter
  guard only after that state machine preserves the same exact-generation and
  revalidation guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the navigation runtime CTests, `make build`, then
  repeat `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check`
  and the remaining 8 m/s map suite.

### 2026-08-27 - Align PlanFromRest start selection with the inflated safety layer

- **Owner:** planning backend. **Scope:** `PlanFromRest` local-start selection before EXP/corridor generation.
- **Safety impact:** the start is now selected from the same inflated layer used by corridor construction and executable-command validation. This removes an Evidence-vs-Inflated classification mismatch that could make corridor generation skip the actual start and construct a degenerate backup seed. No UNKNOWN allowance, collision threshold, command lease, or fail-closed gate is relaxed.
- **Derivation:** artifact `.artifacts/runtime/external-mode-check-20260827T080220-823090` reached waypoint 3, then logged `robot_grid=4`, `robot_inf_grid=4` while `PlanFromRest` failed with `GeneratePolytopeFromLine` for the degenerate seed `(-5.70,2.50,3.30) -> (-5.70,2.50,3.30)` after waypoint endpoint projection.
- **Evidence:** focused planner/runtime tests, Release rebuild, and repeated representative 8 m/s SITL across canonical maps; retain this entry until the start-layer contract is covered by a regression test.
- **Removal/review condition:** remove only if ownership of the PlanFromRest start layer is deliberately redesigned and the corridor/validator contract is updated together.
- **Verification command:** `make build && make test`; then run the declared 8 m/s external-mode map matrix and inspect `DECISION_TRACE` plus `PATH_SNAPSHOT` start/backup geometry.

### 2026-08-27 - Select backup switch points with a known-free braking seed

- **Owner:** planning backend backup trajectory generation. **Scope:** when
  choosing the transition from the exploratory MAIN trajectory to the
  fail-safe BACKUP suffix, require the actual minimum-snap braking seed to pass
  the existing swept `KNOWN_FREE` certificate in addition to fitting the
  geometric SFC. If it fails, move the switch earlier; if no candidate passes,
  retain fail-closed failure.
- **Safety impact:** this prevents a backup suffix that is geometrically inside
  an allow-unknown corridor but whose swept tube enters UNKNOWN. It does not
  change the mission UNKNOWN policy, collision envelope, dynamics limits,
  command freshness, or PX4 handover gates.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T083101-846676` reached WP2
  and then repeatedly rejected the WP3 backup with
  `kCertificateTubeBlocked`, role `BACKUP`, at
  `(-5.715,2.863,2.997)`. The existing code checked the geometric SFC first
  and only discovered the UNKNOWN crossing after optimization. Each candidate
  adds one bounded swept validation of the braking seed within the existing
  solve deadline.
- **Evidence:** run focused planner/runtime tests, `make build`, `make test`,
  then repeat the 8 m/s External Mode map suite. Confirm the backup switch is
  known-free, WP3/WP4 acceptance proceeds, and all collision/fail-closed gates
  remain active.
- **Removal/review condition:** revisit when the backup optimizer directly
  consumes a known-free corridor certificate and the same seed/sweep contract
  is enforced at that boundary. Remove this switch-point adapter only after
  the replacement preserves the known-free suffix guarantee.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning-backend/runtime CTests, `make build`,
  `make test`, then run the declared 8 m/s External Mode map matrix.

### 2026-08-27 - Replan from a continuously certified A* prefix

- **Owner:** planning backend. **Scope:** after A* returns a grid path, validate
  every consecutive edge with the same continuous inflated-map oracle used by
  corridor generation. If an interior edge is rejected, retain only the
  certified prefix and let the next cycle replan; a blocked first edge remains
  a fail-closed solve failure.
- **Safety impact:** this removes a false whole-solve failure caused by the
  grid/continuous-edge mismatch without accepting, skipping, or visualizing a
  blocked edge. The retained prefix still passes inflated-layer, UNKNOWN,
  corridor, trajectory, swept-validator, freshness, and PX4 handover checks.
  No collision or planning gate is relaxed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T082225-838337` repeatedly
  reached WP0, then rejected the final A* edge at WP1 in
  `SearchPolytopeOnPath`, causing three `PlanFromRest` failures and a
  fail-closed pause. The prefix check is linear in the returned path length and
  preserves the existing solve deadline.
- **Evidence:** run focused planner/runtime tests, `make build`, `make test`,
  then repeat the 8 m/s External Mode map suite. Confirm the trace shows
  forward progress from WP1 with no `blocked adjacent edge` failure and that
  waypoint acceptance, corridor certification, and fail-closed behavior remain
  intact.
- **Removal/review condition:** revisit when A* itself guarantees continuous
  edge validity for every neighbor mode and that guarantee is covered by a
  shared planner-map contract. Remove this prefix adapter only after the
  replacement retains the same fail-closed behavior.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning-backend/runtime CTests, `make build`,
  `make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s map matrix.

### 2026-08-27 - Hold a completed corner trajectory until measured settling

- **Owner:** PX4 External Mode mission controller and native planner-command adapter. **Scope:** when a completed MAIN command has both its certified endpoint and the measured vehicle position inside the active waypoint acceptance ball, latch a terminal hold only for STOP waypoints or genuine pass-through corners. Keep the existing measured-position, measured-speed, and corner-direction acceptance gates; clear the latch only after those gates are satisfied or a fresh non-terminal native trajectory is accepted.
- **Safety impact:** this prevents a repeated goal publication from reintroducing a velocity command while the vehicle is still settling at a turn. It does not accept a waypoint while moving, enlarge the acceptance radius, permit a straight-line fly-through to stop unnecessarily, or relax collision, UNKNOWN/OUT_OF_MAP, freshness, tracking-envelope, or PX4 health gates.
- **Derivation:** artifact `.artifacts/runtime/external-mode-check-20260827T080740-827646` repeatedly published waypoint 3 after MAIN `STATUS_COMPLETED`; its endpoint was 0.424 m from the requested target inside the 0.90 m acceptance radius, while the measured planning velocity was about 0.423 m/s. The U-turn corner therefore failed the existing 0.15 m/s acceptance gate and re-entered goal/replan churn.
- **Evidence:** add a MissionController regression for a completed native terminal hold at a corner, run the PX4 mission/runtime CTests, rebuild Release, then repeat the 8 m/s map suite and verify waypoint acceptance, measured settling speed, and absence of repeated same-waypoint goal churn.
- **Removal/review condition:** revisit when terminal-command ownership and mission acceptance share one versioned state machine; remove this adapter latch only after the replacement preserves the same measured-state and corner-direction guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and `install/setup.bash`; run the PX4 navigation external-mode CTests, `make build`, `make test`, then repeat the representative 8 m/s External Mode map matrix.

### 2026-08-27 - Accept a certified backup terminal inside a pass-through waypoint

- **Owner:** PX4 External Mode native planner-command adapter. **Scope:** a
  `STATUS_COMPLETED` BACKUP command may remain a position hold when both its
  endpoint and the measured vehicle position are inside the active waypoint's
  configured acceptance ball. MissionController then applies its existing
  measured-speed and corner-direction rules before advancing the mission.
- **Safety impact:** this removes an adapter-only rejection that treated every
  completed backup as a failure even after the vehicle had safely reached a
  pass-through waypoint. It does not accept an outside endpoint, missing or
  stale odometry, a rejected command, or a waypoint without the normal mission
  acceptance gate; outside the ball the existing PX4 Hold handover remains.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T083847-851511` showed WP3
  endpoint and measured position within the 0.90 m acceptance radius, followed
  by a completed BACKUP hold and no waypoint-3 acceptance event. The change is
  two constant-time acceptance-ball checks on the command/control paths plus a
  focused regression test.
- **Evidence:** run the PX4 mission/runtime tests, `make build`, `make test`,
  then repeat the 8 m/s map matrix. Confirm WP3/WP4 acceptance and mission
  completion while completed backup outside the acceptance ball still hands
  over fail-closed.
- **Removal/review condition:** revisit when native command completion and
  MissionController acceptance share one versioned state machine; remove this
  adapter condition only after the replacement preserves the same endpoint,
  measured-state, and safety-handover guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run PX4 navigation external-mode CTests, `make build`,
  `make test`, then run the declared 8 m/s External Mode map matrix.

### 2026-08-27 - Preserve a certified stop hold across transient speed overshoot

- **Owner:** PX4 External Mode mission controller. **Scope:** while a STOP
  waypoint is in `Holding`, keep the existing certified position hold when the
  measured vehicle remains inside its acceptance ball but briefly exceeds the
  configured acceptance speed. Restart the measured hold timer and do not
  publish the same waypoint again; leaving the acceptance ball still requests
  a fresh planner trajectory.
- **Safety impact:** this prevents a speed-noise or plant-overshoot sample from
  turning a safe local hold into repeated goal/replan churn. Waypoint acceptance
  still requires the configured position radius, finite measured velocity at or
  below the configured speed, and the full hold duration. No position, speed,
  collision, UNKNOWN/OUT_OF_MAP, freshness, tracking-envelope, or PX4 health
  gate is relaxed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T085814-865309` reached WP3
  inside the 0.90 m radius and entered `Holding`, then a transient measured
  speed above 0.15 m/s caused repeated same-waypoint publications until the
  safety handover. The change is constant-time state handling and does not add
  planner work.
- **Evidence:** add the focused `HoldingDoesNotReplanForTransientSpeedOvershootInsideAcceptance`
  regression, run the PX4 mission/runtime tests, rebuild Release, then repeat
  the 8 m/s map matrix. Confirm the hold timer restarts, no same-waypoint churn
  occurs while inside the ball, and outside-ball behavior remains fail-closed.
- **Removal/review condition:** revisit when the mission controller and native
  command completion share one versioned settling state machine; remove this
  state-handling adapter only after the replacement preserves the same
  measured-position, measured-speed, and hold-duration guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run PX4 navigation external-mode CTests, `make build`,
  `make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Preserve a traversable measured start for PlanFromRest

- **Owner:** planning backend. **Scope:** `PlanFromRest` retains the measured
  vehicle position when it is inside the immutable map and traversable under
  the active UNKNOWN policy. It uses the nearest inflated-layer cell only for
  an occupied or out-of-map measured pose.
- **Safety impact:** this removes a discretization-induced move toward a nearby
  obstacle before corridor and backup certification. The selected start still
  passes the inflated-layer, continuous corridor, UNKNOWN/OUT_OF_MAP,
  trajectory, swept-validator, freshness, and PX4 command gates; no gate is
  relaxed and no unverified start is accepted.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T090207-872276` reached WP3,
  then WP4 `GenerateExpTrajectory` succeeded while backup CIRI repeatedly
  rejected a snapped seed from `(-5.500,2.500,3.100)` to
  `(-5.700,2.700,3.100)` with minimum obstacle distance `0.7483 m`. The
  runtime trace placed the measured start near `(-5.59,2.55,3.08)`. The
  change adds one bounded contains/classification check and preserves the
  existing nearest-cell fallback.
- **Evidence:** run planning-backend tests, `make build`, `make test`, then
  repeat the 8 m/s structured-obstacle scenario and the remaining declared
  map matrix. Confirm a measured-start corridor is certified and all
  fail-closed behavior remains active.
- **Removal/review condition:** revisit when the map/planner boundary exposes a
  continuous clearance-preserving start projection contract; remove this local
  start selection only after that replacement preserves measured continuity
  and all existing safety certificates.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning-backend/runtime CTests, `make build`,
  `make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Validate every A* graph edge with the execution geometry

- **Owner:** planning backend A*. **Scope:** before an A* neighbour is inserted
  into the open set, validate its continuous segment under the selected search
  layer and under the authoritative inflated layer. This covers axial and
  diagonal edges, including paths produced by the probability-map fallback.
- **Safety impact:** this prevents a free endpoint or evidence-free voxel from
  creating a continuous edge that cuts an inflated occupied cell. The same
  UNKNOWN policy is retained; OCCUPIED and OUT_OF_MAP remain fail-closed. No
  corridor, collision, dynamic, freshness, or handover gate is relaxed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T092839-895825` recorded a
  blocked continuous edge from `(-5.6527,2.8405,2.9886)` to
  `(-5.5764,2.6702,2.9943)` after the prior virtual-start correction. The
  remaining unchecked edge was an A* graph-neighbour transition. Each
  expansion now performs two bounded continuous segment queries before node
  insertion.
- **Evidence:** run planning-backend/runtime tests, rebuild the authoritative
  manifest, repeat the structured-obstacle 8 m/s scenario, and then run the
  remaining declared 8 m/s map matrix. Confirm no blocked graph edge reaches
  corridor generation, waypoint acceptance progresses, and no-path cases still
  fail closed.
- **Removal/review condition:** revisit when A* exposes one shared edge
  validator for all graph modes and the downstream corridor/execution layers;
  remove the duplicated authoritative check only after that shared contract is
  proven equivalent.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning-backend/runtime CTests, `make build`,
  `make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Anchor backup visibility and CIRI at the executable command boundary

- **Owner:** planning backend backup generation. **Scope:** use the first
  sample of the newly generated executable EXP trajectory as the backup
  visibility/CIRI origin. Do not re-snap that origin through the Evidence grid;
  PlanFromRest/corridor generation and the immutable candidate validator remain
  the authority for the command start.
- **Safety impact:** this removes a false first-segment obstruction caused by
  converting an inflated-layer start back to an Evidence-grid centre. The
  origin must be inside the map and KNOWN_FREE, and the complete main+backup
  candidate is still checked under the latest immutable world. No OCCUPIED,
  UNKNOWN, OUT_OF_MAP, dynamic, freshness, or handover gate is relaxed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T091951-887181` reached WP3,
  then PlanFromRest generated the main trajectory but repeatedly built the
  backup line from `(-5.500,2.500,2.900)` to `(-5.700,2.500,2.900)` after an
  Evidence-grid re-snap. The executable start was the nearby inflated-layer
  command boundary; the resulting CIRI call failed and the mission entered
  fail-closed safety stop. The change adds one bounded command-boundary safety
  check and removes the inconsistent Evidence-grid re-snap.
- **Evidence:** run planning-backend/runtime tests, rebuild the authoritative
  manifest, then repeat the structured-obstacle 8 m/s run and the remaining
  declared 8 m/s map matrix. Confirm the backup origin is KNOWN_FREE, the
  command certificate remains valid, waypoint acceptance progresses, and a
  non-certifiable origin still fails closed.
- **Removal/review condition:** revisit when backup generation receives an
  explicit typed executable-boundary certificate shared with PlanFromRest and
  candidate validation; remove this local boundary derivation only after that
  contract preserves the same no-resnap and fail-closed guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning-backend/runtime CTests, `make build`,
  `make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Connect A* from the continuous pose to a certified graph voxel

- **Owner:** planning backend A*. **Scope:** when the containing graph voxel
  centre is not continuously reachable from the measured/guide start, search a
  bounded three-cell neighbourhood for a traversable graph voxel whose segment
  from that continuous start passes the same inflated-layer and UNKNOWN policy
  oracle. Keep the continuous start in the returned path and reject the solve
  if no such seed exists.
- **Safety impact:** this removes the grid/continuous start-edge mismatch that
  caused CIRI and command recertification to reject an otherwise valid route.
  It does not accept a blocked edge, teleport the command start, enlarge a
  corridor, or relax UNKNOWN, OUT_OF_MAP, collision, dynamic, freshness, or
  handover gates. The bounded search is fail-closed when no certified seed is
  available.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T090842-878545` repeatedly
  rejected the first edge from the continuous guide point around
  `(-5.63,2.85,2.86)` to the first grid point around `(-5.57,2.67,2.93)`;
  the A* graph had validated only centre-to-centre edges. The candidate scan
  checks at most 218 boundary cells per search and keeps the existing A*
  deadline.
- **Evidence:** run planning-backend tests, `make build`, `make test`, then
  repeat the 8 m/s structured-obstacle scenario and the remaining declared map
  matrix. Confirm no `A* path starts with a blocked continuous edge` occurs,
  waypoint acceptance progresses, and fail-closed behavior remains active when
  the bounded neighbourhood has no certified connection.
- **Removal/review condition:** revisit when A* uses an explicit virtual-start
  node with shared continuous-edge validation for every neighbor mode. Remove
  this bounded adapter only after that replacement preserves the same
  measured-start continuity and fail-closed guarantees.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planning-backend/runtime CTests, `make build`,
  `make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Preserve a certified backup suffix across a visible hot replan

- **Owner:** planning backend hot-replan command disposition. **Scope:** when
  a same-goal hot replan reports `FINISH` or `NO_NEED` for the newly generated
  EXP path, do not replace the active command if it still has a future atomic
  backup suffix. Keep the existing bundle until runtime latest-world
  recertification succeeds or a later solve creates another complete bundle.
  Exclude new-goal retargets. An active backup role is intentionally retained
  until its finite stop endpoint; it is not replaced by a main-only candidate
  while that suffix is being drained.
- **Safety impact:** this prevents a main-only candidate from erasing the
  only available braking suffix between two mapping revisions. The retained
  bundle is still checked by the runtime against the newest immutable world;
  an invalid or expired bundle remains fail-closed and may hand over to PX4
  Hold. No UNKNOWN, OCCUPIED, inflated-map, dynamic, freshness, or handover
  gate is relaxed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T093245-903523` showed a
  backup-capable bundle at generation 128 followed by main-only generations
  129, 131, and 132. Generation 132 had `backup_path=[]`; after a later map
  revision invalidated its main path, the runtime had no retained safety
  suffix and entered PX4 Hold. The change is one bounded command-state check
  per hot replan; old-bundle recertification remains at the runtime boundary.
- **Evidence:** unit-test the disposition predicate, run planning-backend and
  runtime tests, rebuild the authoritative manifest, and repeat the structured
  obstacle 8 m/s scenario plus the declared map matrix. Verify that a
  main-only replacement cannot lower backup availability before the waypoint
  is complete, and that invalid retained bundles still fail closed.
- **Removal/review condition:** revisit after the planner exposes one typed
  candidate-disposition contract that carries old/new bundle safety-role
  transitions through the execution boundary; remove this local retention
  rule only when that contract preserves the same atomic suffix invariant.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Match high-speed mission dynamics to PX4 tracking authority

- **Owner:** planner mission-dynamics contract. **Scope:** the two dedicated
  140 m speed profiles (`long_three_pillars_speed` and
  `long_open_featured_speed`) retain the requested velocity cap, but use
  `max_acceleration_mps2=2.0` and `max_jerk_mps3=4.0` for the generated PVA
  trajectories.
- **Safety impact:** this reduces commanded startup aggressiveness; it does
  not relax the 8 m/s speed target, the 0.75 m External Mode command-anchor
  envelope, collision/clearance, freshness, map, or waypoint gates. The
  vehicle must still demonstrate measured cruise speed before a speed run can
  pass.
- **Derivation and cost:** both 8 m/s screening runs reached only the first
  waypoint and failed at startup tracking. The three-pillar artifact
  `external-mode-check-20260827T104205-972158` showed measured position
  `x=0.179 m` versus command `x=0.937 m` and a longitudinal error of
  `0.760/0.750 m`; the long-open artifact
  `external-mode-check-20260827T104358-973909` showed the same boundary at
  `0.753/0.750 m`. A first 2/6 trial moved the three-pillar failure farther
  along the leg but still ended at `0.756/0.750 m` in
  `external-mode-check-20260827T104706-978629`, with measured speed only
  `2.179 m/s`; retain the 2.0 m/s² acceleration required by the 23 m visibility
  cap and reduce only jerk to 4.0 m/s³ for the next calibration. Lowering
  acceleration to 1.0 m/s² is rejected because its 8 m/s braking horizon is
  outside that cap. Cruise attainment must be measured, not assumed.
- **Evidence:** run the planner/runtime focused tests, rebuild the authoritative
  manifest, then repeat both profiles at `SPEED_CAP_MPS=8`. Require complete
  waypoint coverage, zero collision, independent clearance, no safety stop,
  measured speed attainment, and no tracking-envelope rejection.
- **Removal/review condition:** revisit after repeated speed-ladder evidence
  shows a larger acceleration/jerk pair can be followed by PX4 without
  exceeding the fixed anchor envelope; do not raise these values from one
  successful run.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_speed make external-mode-check`
  and
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_open_featured_speed make external-mode-check`.

### 2026-08-27 - Roll back unproven high-speed/objective tuning and expose PVAJ provenance

- **Owner:** navigation planning/runtime maintainers. **Scope:** restore the
  previously reviewed 2.0 m/s² acceleration, 4.0 m/s³ jerk, route-reference
  weights 1.0/1.0, and `rog_map.inflation_step=4`. Remove the bounded lateral
  A* shortcut and traversable-prefix trim. Keep virtual-start projection and
  continuous edge checks. Propagated odometry now derives A/J only from
  consecutive P/V samples and marks them as estimates; epoch, timestamp, or
  freshness discontinuities restart that history.
- **Safety impact:** no safety gate is relaxed. A* and CIRI/world-model
  disagreement now fails closed instead of being converted into an executable
  prefix. Estimated derivatives are explicitly provenance-tagged and are not
  treated as sensor measurements; a gap does not manufacture stale dynamics.
- **Reason/evidence:** the 4/8, route-weight 10/1000, and inflation-6 runs did
  not complete the mission. Inflation-5 still produced a CIRI minimum-distance
  warning of `0.771456 m`, demonstrating an oracle/quantization mismatch rather
  than a proven need for more inflation. The latest 8 m/s run had zero odometry
  callbacks and never reached takeoff, so it is not planner evidence.
- **Evidence required:** focused derivative and planner invariant tests,
  authoritative rebuild, then repeated A/B runs with one variable at a time.
  Each run must separately pass bring-up (odometry/takeoff/mode), complete all
  waypoints, maintain altitude/tracking/clearance, and show no command-anchor
  or safety handover failure before any high-speed retuning is considered.
- **Removal/review condition:** revisit the baseline only after a distribution
  over repeated SITL and representative recorded data proves the controller
  envelope and the A*/continuous-certificate contract; do not compensate by
  relaxing CIRI, freshness, command-anchor, or swept-tube gates.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, inspect the derivative
  provenance trace, then run the declared 8 m/s map matrix.

### 2026-08-27 - Recondition high-speed route following without relaxing safety gates

- **Owner:** navigation planning/runtime maintainers.
- **Scope:** the two high-speed mission profiles
  `long_three_pillars_speed` and `long_open_featured_speed` now use
  `max_acceleration_mps2=4.0` and `max_jerk_mps3=8.0`; the planner route
  reference objective uses `lateral_weight=10.0` and `vertical_weight=1000.0`.
- **Reason/evidence:** the preceding 2/4 profile did not reach the requested
  8 m/s envelope: the three-pillar run had measured-speed p95 about 4.58 m/s,
  while the open run had p95 about 1.72 m/s and the generated route descended
  toward z=0.55 m. The 1/3 trial was rejected at startup because its required
  braking/visibility horizon exceeded the fixed 23 m visibility cap. At 4/8,
  the jerk-limited stop distance plus replan reserve is about 13.95 m, below
  that cap, while route-reference conditioning penalizes the observed vertical
  drift without making the route reference a safety certificate.
- **Safety impact:** this is a bounded mission/objective reconditioning, not a
  gate relaxation. Unknown/out-of-map handling, command-anchor limits,
  swept-tube collision certificates, dynamic feasibility, backup validity, and
  PX4 tracking gates remain unchanged. A failed certificate still fails closed.
- **Evidence required:** run both profiles at `SPEED_CAP_MPS=8` repeatedly and
  inspect measured speed, altitude tracking, waypoint completion, clearance,
  rebase events, and backup/certificate outcomes. Dataset or planner-only PASS
  does not certify closed-loop flight behavior.
- **Removal/review condition:** revert or retune this provisional profile if
  repeated runs or representative recorded-data evidence show command-anchor
  violations, loss of altitude/clearance, unstable replanning, or failure to
  satisfy the declared speed/mission goals. Do not compensate by disabling a
  safety gate or increasing a tolerance.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_open_featured_speed make external-mode-check`
  and
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_speed make external-mode-check`.

### 2026-08-27 - Align inflated-grid detour clearance with continuous certification

- **Owner:** navigation mapping/planning maintainers.
- **Scope:** product `rog_map.inflation_step` is increased from 4 to 5 at
  `inflation_resolution=0.20 m`, giving a 1.20 m inflated-grid radius. No
  vehicle/error envelope, unknown-space policy, command-anchor limit, or
  continuous swept-tube tolerance is changed.
- **Reason/evidence:** the 8 m/s three-pillar run selected an A* seed line whose
  measured continuous clearance was `0.738 m`, below the configured
  `robot_r=0.75 m`; CIRI correctly rejected it before execution. The grid
  inflation had been only `0.80 m`, leaving a discretization gap between the
  voxel route and the continuous certificate. Increasing inflation makes A*
  reject that near-grazing branch earlier and search for a wider detour. The
  first 1.00 m A/B still produced a CIRI minimum-distance warning of 0.771 m
  and no mission completion, so one further 0.20 m cell is being evaluated.
- **Safety impact:** more conservative occupancy, potentially less free search
  volume and more fail-closed outcomes. This change cannot make an unsafe
  route executable; all authoritative certificates remain active.
- **Evidence required:** repeat the 8 m/s three-pillar and open profiles and
  compare route clearance, CIRI failures, planner latency, waypoint completion,
  measured speed, and collision truth. Preserve the negative-map fail-closed
  evidence.
- **Removal/review condition:** revert only if repeated representative runs
  show unacceptable search starvation/latency without a safer route, then
  redesign the grid-to-continuous clearance contract; never compensate by
  relaxing CIRI or swept-tube checks.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_speed make external-mode-check`
  and
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_open_featured_speed make external-mode-check`.

### 2026-08-27 - Reproject an occupied goal to the deepest certified acceptance point

- **Owner:** planning backend goal projection. **Scope:** when an occupied or
  out-of-map requested waypoint is projected into its configured acceptance
  ball, continue stepping in the already certified escape direction by the
  inflated-map resolution while the next point and connecting segment remain
  traversable. Stop at the acceptance boundary or the first failed map query.
- **Safety impact:** this changes only which point inside the existing mission
  acceptance ball is optimized. Each step remains checked on the inflated ROG
  layer under the configured UNKNOWN policy; no acceptance radius, vehicle
  radius, corridor, certificate, or OUT_OF_MAP rule is enlarged.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T102810-961423` repeatedly
  projected `occlusion_detour` to a point only one inflated step beyond the
  occupied voxel, then invalidated the resulting tube as the hidden obstacle
  became observed. The bounded walk is at most the finite acceptance radius
  divided by the configured inflated resolution and adds no unbounded search.
- **Evidence:** unit-test planner contracts, run `make build` and `make test`,
  then repeat the structured-obstacle 8 m/s mission and the declared positive
  map matrix. Confirm the selected endpoint stays inside the mission radius,
  remains certificate-valid after obstacle revelation, and fail-closed
  behavior is unchanged when no certified step exists.
- **Removal/review condition:** revisit after the planner has an explicit
  clearance-aware terminal-goal contract shared by A*, CIRI, MINCO, and the
  execution certificate; remove the local projection walk only when that
  contract preserves the same bounded, fail-closed endpoint selection.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Keep an anchored backup endpoint in hold across acceptance-edge jitter

- **Owner:** PX4 External Mode terminal-command gate. **Scope:** a completed
  backup endpoint may use the bounded planner-recovery hold when the command
  endpoint is inside the active waypoint acceptance ball and the measured
  state remains within the existing command-anchor envelope. MissionController
  still owns measured-position, speed, and hold-time acceptance.
- **Safety impact:** this prevents a small measured oscillation at the edge of
  an already accepted backup endpoint from starting a false recovery cycle. It
  does not accept a waypoint, extend a trajectory, relax the anchor limit, or
  bypass the speed/position gates; a non-anchored endpoint still enters the
  bounded recovery window and then fails closed.
- **Derivation and cost:** the structured-obstacle run showed a backup command
  error of `1.032 m` against a `0.9 m` waypoint radius, followed by a transient
  measured error of `0.845 m` and a safety handover. The helper is constant
  time and reuses `kCommandAnchorErrorLimitM` rather than adding a new tracking
  threshold.
- **Evidence:** unit-test the anchor predicate, run `make build` and
  `make test`, then repeat the structured-obstacle 8 m/s mission. Verify logs
  expose independent `main_hold_inside` and `backup_hold_inside` states and
  that an endpoint outside the anchor envelope still hands over after the
  bounded deadline.
- **Removal/review condition:** replace this executor-side anchor adapter when
  runtime and External Mode expose an atomic terminal-endpoint acknowledgement
  carrying measured settling state.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check`.

### 2026-08-27 - Restart from measured state after map invalidates the active command

- **Owner:** navigation runtime mapping-to-planner transition. **Scope:** when
  latest-world recertification rejects the active bundle and an active goal is
  still present, clear the stale command/terminal state and schedule
  `PlanFromRest` from the current measured propagated state. The recovery is
  limited to the same localization and goal epochs; planner failure budgets
  and fail-closed handover remain unchanged.
- **Safety impact:** this removes stale-command retention after a newly
  observed obstacle invalidates the executable certificate. The replacement
  must pass the normal planner candidate, inflated-map, UNKNOWN, dynamic, and
  atomic commit gates; if it cannot, the existing bounded retry then safety
  handover is preserved.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T102810-961423` showed
  recertification rejection at world revision 408 and later revisions while
  the planner remained associated with the old bundle. The transition adds a
  lock-protected state reset and one measured-state planning request per
  invalidation.
- **Evidence:** run runtime tests, `make build`, `make test`, and the
  structured-obstacle 8 m/s mission. Confirm the log records
  `scheduling measured-state PlanFromRest recovery`, and that three consecutive
  replacement failures still produce a fail-closed PX4 Hold handover.
- **Removal/review condition:** replace the reset with an explicit atomic
  invalidation acknowledgement from the immutable world authorizer to the
  planner FSM.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check`.

### 2026-08-27 - Bound the backup-endpoint to planner-recovery handover race

- **Owner:** PX4 External Mode command/setpoint boundary. **Scope:** when a
  finite backup suffix reaches its endpoint outside the active waypoint
  acceptance ball, keep publishing the exact endpoint position hold for a
  bounded `navigation.planner_recovery_wait_timeout_s` window while the
  runtime planner publishes the next `PlanFromRest` command. A fresh READY
  command clears the window; expiry requests the existing PX4 Hold handover.
- **Safety impact:** this changes only the executor scheduling boundary. It
  does not extend the trajectory, sample a future polynomial point, accept a
  waypoint, relax tracking/odometry/health/map/UNKNOWN gates, or convert a
  frontier endpoint into mission progress. The endpoint remains a stationary
  hold and expiry remains fail-closed. The default 0.5 s window is bounded by
  the existing 2.0 s trajectory acquisition timeout.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T100710-936100` shows the
  backup completion at `1787825296.604350689` and the replacement planner
  commit at `1787825296.606301269`; the former caused immediate handover before
  the latter could be accepted. The change adds one monotonic deadline and
  one 50 ms mission-timer check. No planner or collision gate is relaxed.
- **Evidence:** unit-test the deadline predicate, run `make build && make test`,
  then repeat the 8 m/s structured-obstacle mission. Confirm that a READY
  replacement clears the hold window and that a missing replacement still
  hands over after the finite deadline.
- **Removal/review condition:** replace this adapter grace with an explicit
  command-acknowledgement handshake between runtime and External Mode once the
  interface can atomically acknowledge a finite endpoint and its replacement.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Replay a missed local-frontier completion to restart planning

- **Owner:** runtime command publisher and receding-horizon planner FSM.
  **Scope:** if a finite local trajectory ends before the mission goal and its
  finished sample was missed, replay only its exact declared endpoint so the
  planner can start the next PlanFromRest cycle.
- **Safety impact:** replay requires a finite endpoint, current KNOWN_FREE
  classification in the inflated ROG layer, and measured endpoint proximity
  within the existing command-anchor limit. It publishes no future trajectory
  sample and does not convert a frontier endpoint into waypoint acceptance.
  Unknown, occupied, out-of-map, stale, or distant endpoints remain
  fail-closed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T100233-931480` accepted
  WP0-WP3, then WP4 received repeated finite local bundles ending at
  `(-5.19,5.25,2.70)` while the goal was `(-3.00,5.50,3.00)`. The publisher
  missed the frontier `finished` tick and raised
  `execution boundary invalidated the committed command sample`, preventing
  the intended receding-horizon restart. The change adds one endpoint and
  measured-anchor check at expiry.
- **Evidence:** unit-test the replay predicate, run planning/runtime tests,
  rebuild the authoritative manifest, and repeat the structured-obstacle 8 m/s
  scenario plus the declared map matrix. Verify that frontier completion
  restarts PlanFromRest and that an endpoint failing any predicate still hands
  over safely.
- **Removal/review condition:** revisit after the command publisher and planner
  FSM expose an explicit finite-endpoint completion acknowledgement, removing
  the need to recover the endpoint from the immutable bundle at expiry.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Recover a missed finite terminal sample with an exact endpoint hold

- **Owner:** runtime command publisher and External Mode waypoint acceptance.
  **Scope:** if the publisher misses the single tick at which a finite command
  bundle reports `finished`, recover only that bundle's declared endpoint after
  the execution interval. The endpoint must be finite, currently KNOWN_FREE in
  the inflated ROG layer, and inside the active waypoint acceptance ball.
- **Safety impact:** this closes a scheduling race at a waypoint without
  extending the trajectory or sampling any future polynomial time. It does not
  relax UNKNOWN, OCCUPIED, OUT_OF_MAP, freshness, command identity, tracking,
  or handover gates. An endpoint that is not current known-free remains
  fail-closed.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260827T095407-920740` reached WP1
  with position error `0.117 m` but speed `0.390 m/s`. The finite bundle ended
  at `(-0.97,5.97,3.00)` inside the `0.7 m` acceptance radius, but no
  `trajectory completion observed` event was recorded before the publisher
  crossed the expiry boundary; the runtime then reported
  `execution boundary invalidated the committed command sample`. The change
  adds one exact endpoint classification at that boundary.
- **Evidence:** unit-test the endpoint predicate, run planning/runtime tests,
  rebuild the authoritative manifest, and repeat the structured-obstacle 8 m/s
  scenario plus the declared map matrix. Confirm that the missed-tick path
  produces a bounded terminal hold and that a non-goal or unknown endpoint
  still fails closed.
- **Removal/review condition:** revisit after command publication and mission
  acceptance share an explicit event/acknowledgement for finite endpoint
  samples, eliminating the missed-tick ambiguity without recovering from the
  immutable bundle at the publication boundary.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_SCENE=structured_obstacle make external-mode-check` and
  the remaining declared 8 m/s External Mode map matrix.

### 2026-08-27 - Rebase hot replanning after measured tracking lag

- **Owner:** navigation planning backend hot-replan boundary. **Scope:** when
  the currently committed command position is farther from the fresh propagated
  vehicle position than the existing `planner/tracking_error_budget_m` (0.25 m
  in the SITL profile), discard only the planner's historical continuation and
  generate the next candidate from the measured PVAJ state. The measured start
  must be inside the current inflated map under the active UNKNOWN policy; an
  invalid start remains a failed solve.
- **Safety impact:** this prevents repeated stitching of a command that is
  already moving ahead of the vehicle. It does not enlarge the 0.75 m External
  Mode command-anchor gate, alter waypoint acceptance, or bypass the normal
  dynamic, corridor, world swept, freshness, or atomic commit certificates.
  The deliberate false-reject consequence is a bounded retained-suffix or
  fail-closed handover when the measured start is not certified traversable.
- **Derivation and cost:** the 8 m/s three-pillar artifact
  `external-mode-check-20260827T105943-987541` repeatedly retained a command
  while the measured state lag grew from 0.507 m to 0.703 m, then PX4 rejected
  the suffix at `0.760/0.750 m`. The open-map artifact
  `external-mode-check-20260827T110127-989415` showed the same retained-suffix
  pattern and eventually no executable backup. The rebase check is constant
  time plus one inflated-layer classification per hot replan; it should be
  bucketed in future p50/p95/p99 planner and command-gap reports.
- **Evidence:** add focused planner coverage for a measured-state rebase and an
  occupied/out-of-map rejection, run `make build` and `make test`, then repeat
  the 8 m/s speed profiles. Require the trace to show the rebase boundary,
  candidate starts anchored to measured state, complete waypoint coverage,
  measured cruise speed, zero collision, and no command-anchor rejection.
- **Removal/review condition:** replace this local recovery with an explicit
  controller tracking contract that continuously time-scales or reanchors the
  command bundle from measured state while preserving PVAJ continuity and has
  repeated SITL, recorded-data, sanitizer, and hardware evidence.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build && make test`, then run
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_speed make external-mode-check`
  and
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_open_featured_speed make external-mode-check`.
### 2026-08-27 - Align inflated-grid and CIRI continuous-clearance contracts

- Owner: navigation planning/mapping maintainers.
- Scope: `WorldModelView::isSegmentTraversable` for the inflated layer in the
  live ROG adapter and immutable mapping snapshot.
- Safety impact: positive and fail-closed.  A segment is rejected when an
  observed occupied point lies inside the same robot-radius tube used by CIRI,
  even if the coarse inflated voxel query reports the segment free.  Unknown,
  out-of-map, and existing grid occupancy rules are unchanged.  No gate is
  relaxed and no fallback is promoted to product behavior.
- Evidence: external-mode artifact
  `external-mode-check-20260827T115832-1049674` repeatedly reports CIRI minimum
  distance `0.776... m` while the planner envelope is `0.80 m`; the same run
  continues to produce hot-replan and corridor failures after A* accepts the
  route.  This is the observed grid/CIRI oracle mismatch that caused the
  rejected seed line to be generated repeatedly.
- Verification: `test_mapping_world_model`
  (`InflatedSegmentRejectsObservedTubeBelowRobotRadius`), full `make test`,
  then repeated `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_speed
  make external-mode-check` with no CIRI seed below the configured radius.
- Removal condition: remove only after a replacement immutable continuous
  clearance contract is demonstrated equivalent to CIRI on representative
  recorded and SITL obstacle geometries, with no reduction in clearance.

### 2026-08-27 - Use corridor-constrained nominal MINCO variables

- Owner: navigation planning backend nominal trajectory optimizer.
- Scope: `traj_opt/exp_traj/pos_constraint_type` in the product planner
  configuration.  Nominal MINCO now uses the corridor parameterization
  (`type=2`) instead of direct waypoint variables (`type=1`).
- Safety impact: positive geometric ownership change.  The optimizer's
  position variables are represented by the generated convex corridor, so
  the polynomial control-point hull cannot be authorized solely by a soft
  plane penalty.  The independent continuous corridor, dynamic, flatness,
  swept-world, and atomic commit gates remain mandatory.  This does not relax
  UNKNOWN, clearance, timing, or execution gates.
- Derivation and cost: artifact
  `external-mode-check-20260827T115832-1049674` shows nominal MINCO retries
  leaving the corridor with violations up to 267.99 m while direct waypoint
  variables were used.  The change adds the corridor mapping/back-propagation
  cost to each optimizer evaluation and may reject degenerate SFC overlaps
  that cannot parameterize a feasible control point.
- Evidence: run `test_exp_optimizer_seed`, the planner configuration and
  trajectory tests, `make build`, `make test`, then repeated
  `SPEED_CAP_MPS=8` SITL missions on the declared map matrix.  Require zero
  continuous corridor violations for every committed main/backup bundle and
  complete waypoint/mission acceptance before treating the change as closed.
- Removal/review condition: remove only if a future optimizer formulation
  provides an equivalent hard corridor parameterization and independent
  continuous certificate with stronger measured evidence.
- Verification command: source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `PARALLEL_WORKERS=1 MAKE_JOBS=1 make build &&
  PARALLEL_WORKERS=1 MAKE_JOBS=1 make test`, then run the declared 8 m/s
  External Mode map matrix.

### 2026-08-28 - Record dataset performance evidence separately from flight acceptance

- **Owner:** dataset replay harness, mapping/planner observability, and External
  Mode acceptance boundary. **Scope:** a prepared AIST Mid-360 ROS 2 bag is the
  authoritative recorded-data input. `DATASET_SHADOW_GOAL_M=0` is an explicit
  mapping/processing benchmark mode: recorded odometry feeds the estimator and
  map, but no synthetic navigation command is executed. The mode does not waive
  any product gate or certify navigation quality.
- **Safety impact:** positive and fail-closed. Dataset processing evidence is
  reported independently from closed-loop PX4 tracking and from shadow-planner
  quality. The raw PX4 `.ulg` supplied separately is not accepted as a ROS 2
  sensor bag and remains outside the replay contract; it is not converted or
  treated as a successful dataset run.
- **Evidence:** full authoritative Release build completed 22 packages and
  captured a valid manifest for the current source fingerprint. Mapping-only
  replay at `RATE=2` passed in
  `.artifacts/runtime/dataset-20260827T171713-4` with 55,435 IMU and 2,772
  LiDAR samples, LIO `TRACKING`, 0 callback/dispatch stalls, and playback
  `1.9409x` of the requested `2x`. Mapping callback p50/p95/p99 were
  `15.254/20.659/23.002 ms`; total map update was
  `5.187/12.408/14.397 ms`; world-snapshot export was
  `10.148/14.496/16.330 ms`. The bounded shadow-planning replay at 1x remains
  `FAIL` in `.artifacts/runtime/dataset-20260827T171200-4` because one EMER
  was emitted after ten READY samples; that is planner evidence, not a reason
  to discard the complete mapping timing result. External Mode could not enter
  SITL because this environment denied the XRCE UDP socket probe with
  `Operation not permitted`.
- **Removal/closure condition:** keep the two evidence levels separate. Close
  dataset performance only after repeated prepared-bag rate sweeps preserve
  exact source counts and bounded timing distributions; close flight acceptance
  only after repeated three-pillar multi-waypoint External Mode runs show
  waypoint/mission completion, continuity, speed recovery, clearance, PX4 and
  propagated-odometry health. Do not infer either closure from the single
  mapping-only PASS.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the full Release build/test, then
  `DATASET=aist-mid360-drive RATE=2 DATASET_SHADOW_GOAL_M=0 make dataset-check`
  and repeated
  `SPEED_CAP_MPS=5 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` when the host permits XRCE/Gazebo sockets.

### 2026-08-28 - Retain a committed candidate during its activation lead

- **Owner:** runtime execution-boundary candidate exposure. **Scope:**
  `commitPlannerCandidate` accepts a valid immutable candidate whose
  `valid_from_ns` is only just ahead of the current publication tick; the
  command lease is committed and the command timer waits for activation. A
  candidate that is active is still sampled and checked against the latest
  execution state before it is accepted.
- **Safety impact:** positive and fail-closed. This removes a false rejection
  caused by scheduler lead without relaxing freshness, anchor-error, world,
  candidate-validity, evaluator, or lease gates. Invalid candidates and
  evaluator failures remain rejected; the timer's explicit
  `awaiting_activation` path remains the only pre-activation behavior.
- **Derivation and cost:** dataset shadow artifact
  `.artifacts/runtime/dataset-20260827T171200-4` showed `valid_from_ns` only
  80 ns after the commit tick, followed by `CANDIDATE_SAMPLE_INVALID` and a
  terminal EMER before the next timer sample. The change adds one validity and
  timestamp comparison in the exposure boundary; it does not extend the
  freshness lease or alter a safety threshold.
- **Evidence:** the runtime source-contract regression passes, and the fresh
  full-bag replay `.artifacts/runtime/dataset-20260827T172508-4` no longer
  reports `CANDIDATE_SAMPLE_INVALID`; it fails earlier because the strict
  KNOWN_FREE minimum-snap backup hull is not feasible in the observed SFC.
  Keep the pre-fix artifact as causal evidence and require a future run with a
  committed candidate to verify active-sample anchoring as well.
- **Removal/closure condition:** retain until the exposure and timer paths
  share one typed activation-state helper with equivalent tests; do not remove
  the explicit pre-activation state or replace it with a grace timeout.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `make build`, `make test`, the runtime contract
  suite, then
  `DATASET=aist-mid360-drive RATE=1 DATASET_SHADOW_GOAL_M=5 make dataset-check`
  and inspect the candidate sample/commit trace.

### 2026-08-28 - Require horizontal map extent for the planner horizon

- **Owner:** planner/world-model geometry binding. **Scope:**
  `Config::bindWorldGeometry` checks the planner's configured horizon against
  the larger of the two horizontal local-map extents, while every axis still
  must contain the derived safety envelope plus one world-resolution cell.
  A tall map with short horizontal axes is therefore rejected instead of
  satisfying the horizon check through its Z extent.
- **Safety impact:** positive and fail-closed. No map is enlarged, clipped,
  or treated as known free. This closes the axis-ownership error identified by
  the audit and preserves the existing minimum extent, inflation, UNKNOWN,
  out-of-map, corridor, and world-swept gates.
- **Derivation and cost:** the product route and planning horizon are
  horizontal; using `local_size_m.maxCoeff()` could incorrectly authorize a
  map whose only long axis was vertical. The change is one two-element maximum
  and does not alter the product map or any threshold.
- **Evidence:** focused config coverage rejects `{10,10,40}` for the current
  horizon while the product `{110,15,6}` geometry remains accepted. Repeat
  planner CTest and the three-pillar SITL matrix after rebuilding.
- **Removal condition:** only if the planner gains an explicit, independently
  owned 3-D horizon contract with separate horizontal/vertical budgets and
  corresponding repeated evidence.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the planner CTest suite and full build/test before
  any SITL or recorded-data acceptance run.

### 2026-08-28 - Certify flatness at exact polynomial junctions

- **Owner:** nominal/backup trajectory flatness certificate. **Scope:**
  `evaluateTrajectoryDynamics` retains its uniform maximum-sample-period scan
  and additionally evaluates every internal MINCO piece boundary exactly,
  including the corresponding yaw sample when present.
- **Safety impact:** positive and fail-closed. A body-rate or thrust peak at a
  polynomial junction can no longer be skipped solely because uniform samples
  fall on either side. The check does not relax V/A/J, flatness, corridor,
  world, freshness, or execution gates.
- **Derivation and cost:** the audit identified sampled flatness as weaker
  than the continuous geometry contract. The added work is bounded by the
  number of trajectory pieces per certificate and is outside the optimizer's
  search loop.
- **Evidence:** add/retain trajectory flatness unit coverage, rebuild planner
  and runtime, then repeat the declared high-speed multi-waypoint three-pillar
  mission. The entry remains open until repeated artifacts show all committed
  main/backup candidates passing the exact-junction certificate.
- **Removal condition:** replace only with a stronger analytic extrema
  certificate over each polynomial piece and preserve the same fail-closed
  result for non-finite evaluations.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planner/runtime CTest, then repeated
  `SPEED_CAP_MPS=5 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` and recorded-data replay.

### 2026-08-27 - Feed mission look-ahead into pass-through terminal tangent

- **Owner:** navigation runtime/planner boundary and mission pass-through
  continuity. **Scope:** when a `BEHAVIOR_PASS_THROUGH` goal carries a finite
  `next_target`, the runtime forwards it to the planner. The planner keeps the
  current waypoint as its geometric endpoint but seeds the terminal velocity
  in the normalized outgoing direction, capped by `max_vel` and
  `sqrt(max_acc * distance_to_next_target)`. STOP goals and missing/invalid
  look-ahead retain zero terminal velocity at a connected endpoint.
- **Safety impact:** positive/quality improvement with fail-closed behavior.
  This removes the known zero-terminal-velocity discontinuity without
  bypassing the mission controller's corner gate. The complete polynomial is
  still checked by strict V/A/J, flatness, continuous corridor, swept-world,
  freshness, and atomic commit certificates. Invalid look-ahead is ignored;
  it cannot become a geometric target or authorize an unknown route.
- **Derivation and cost:** the audit identified `next_target` as already
  present in the contract but unused by the planner, while `goal_vel_en=false`
  forced every pass-through endpoint to stop. The new seed uses one vector
  normalization and a bounded square-root calculation per solve; it does not
  change a hard limit, planner deadline, acceptance radius, or corner gate.
- **Evidence:** add focused helper coverage for direction, acceleration-based
  speed cap, maximum-speed cap, and degenerate look-ahead. Rebuild runtime and
  planner packages, then repeat the multi-waypoint three-pillar mission and
  recorded-data shadow-planning performance replay. This entry remains open
  until waypoint acceptance, mission completion, command-anchor continuity,
  clearance, and planner latency are measured repeatedly.
- **Removal condition:** revisit only after the mission controller/planner
  boundary exposes an equivalent, explicitly acknowledged outgoing-tangent
  contract; do not remove the current waypoint endpoint or corner safety gate.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the focused planner/runtime tests followed by
  `PARALLEL_WORKERS=1 MAKE_JOBS=1 make build &&
  PARALLEL_WORKERS=1 MAKE_JOBS=1 make test`, then run repeated
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` and the declared dataset replay.

### 2026-08-27 - Remove the dynamic-limit certificate allowance

- **Owner:** nominal and backup trajectory dynamic certificates, planner
  configuration, and runtime product profile. **Scope:**
  `traj_opt/boundary/dynamic_limit_tolerance_ratio` remains only as a
  compatibility field for detecting stale configuration and must be exactly
  `0.0`; nominal line-search acceptance and backup refinement compare V/A/J
  extrema directly with the mission/product limits.
- **Safety impact:** positive and fail-closed. The previous `0.75` value made
  the optimizer certificate accept up to `1.75 * max_vel`, `1.75 * max_acc`,
  and `1.75 * max_jerk`, while braking and visibility derivation used the raw
  limits. Non-finite or non-zero values are rejected at config load. No
  physical, flatness, PX4, UNKNOWN, corridor, freshness, or commit gate is
  relaxed, and no new tolerance is introduced.
- **Derivation and cost:** this is a contract correction from the audit's P0
  finding; the product has no repeated distribution that justifies a relaxed
  hard gate. The change removes one multiplier from each certificate path and
  adds one scalar validation. A numerical candidate that marginally exceeds a
  physical limit may now be rejected and follow the existing backup or
  fail-closed path; this is an expected safety tradeoff, not a reason to tune
  the gate from one SITL run.
- **Evidence:** source inspection identified the inconsistent multiplier in
  `nominal_trajectory_optimizer.cpp` and `backup_trajectory_optimizer.cpp`;
  the previous external artifacts reported overspeed and remain open evidence,
  not certification. Focused config coverage now checks zero and rejects a
  non-zero override. Rebuild the planner, run its CTest suite, then repeat the
  declared three-pillar speed matrix and recorded-data performance replay.
- **Removal condition:** remove the compatibility key and its loader only
  after all supported profiles and artifact schemas have migrated away from
  it, with a config-contract test proving stale non-zero overrides cannot be
  accepted through another path.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run
  `PARALLEL_WORKERS=1 MAKE_JOBS=1 make build &&
  PARALLEL_WORKERS=1 MAKE_JOBS=1 make test`, then run the declared repeated
  `SPEED_CAP_MPS=8 MAP_PROFILE=long_three_pillars_speed
  make external-mode-check` matrix and the dataset replay command.

### 2026-08-27 - Bound finite-difference derivatives at the planner boundary

- Owner: navigation runtime and planning backend maintainers.
- Scope: `Planner::setState` when propagated odometry marks acceleration or
  jerk as estimated. Position and velocity remain the measured execution
  state; only estimated acceleration/jerk supplied to the planner's PVAJ
  boundary are norm-bounded by the product envelope shared by EXP and backup.
  Raw estimates remain in runtime diagnostics and decision traces.
- Safety impact: positive and fail-closed. This prevents noisy finite
  differences from making the strict minimum-snap backup seed infeasible while
  preserving the measured position/velocity handover and all dynamic,
  corridor, swept-world, freshness, and atomic-commit certificates. It does
  not relax a limit or convert an estimate into a measurement. A non-finite
  state is still rejected before this boundary.
- Derivation and cost: the dataset shadow artifact
  `dataset-20260827T154025-4` recorded a candidate-boundary jerk estimate of
  `[-39.919141]` with a 30 m/s^3 mission limit. Nominal EXP accepted the
  configured 75% optimization tolerance, while the strict KNOWN_FREE backup
  seed rejected the same boundary, producing `no dynamically feasible
  KNOWN_FREE minimum-snap backup hull inside SFC`. The boundary adds two norm
  checks and at most two vector scales per planning cycle.
- Evidence: focused boundary test, `make build`, `make test`, then repeated
  8 m/s SITL runs and representative recorded-data replay. Require raw and
  bounded derivatives to be visible in the artifact, executable main and
  backup candidates, no command-anchor rejection, and completed waypoint/
  mission acceptance before closing this entry.
- Removal/review condition: replace finite-difference derivatives with a
  measured or validated filtered A/J interface whose error distribution is
  bounded against the same command-boundary contract; remove the clamp only
  after repeated SITL, recorded-data, sanitizer, and hardware evidence.
- Verification command: source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run `PARALLEL_WORKERS=1 MAKE_JOBS=1 make build &&
  PARALLEL_WORKERS=1 MAKE_JOBS=1 make test`, then run the declared 8 m/s
  External Mode map matrix.

### 2026-08-28 - Align backup SFC certification with the braking hull

- **Owner:** planning backend backup-generation and corridor-certification
  maintainers. **Scope:** retain the visibility SFC for the exploratory EXP
  prefix, but when its convex hull check rejects a feasible minimum-snap stop,
  build a bounded candidate SFC from the selected `switch_state` to the
  strict-limit braking endpoint. The selected candidate must still pass the
  complete Bezier-hull containment check, KNOWN_FREE swept validation, flatness,
  and final atomic command authorization.
- **Safety impact:** positive and fail-closed. This removes a geometry mismatch
  between an SFC generated for `command_start -> visibility seed_point` and a
  braking suffix generated for `switch_state -> braking endpoint`. It does not
  permit UNKNOWN, occupied, out-of-map, clearance, dynamic-limit, freshness,
  or deadline violations. If the candidate-aligned SFC or any later certificate
  fails, the existing backup rejection and safety-handover path remain active.
- **Derivation and cost:** the audit and SITL artifacts showed EXP success
  followed by rejection before or during backup at both a 5 m/s multi-waypoint
  mission and the two-waypoint baseline. The change adds a bounded corridor
  construction only when the original hull check fails; it may consume extra
  CIRI time on that recovery path and therefore remains subject to the same
  absolute solve deadline.
- **Evidence:** focused planner/runtime tests, Release rebuild, and repeated
  recorded-data shadow planning plus the 3/4/5 m/s three-pillar multi-waypoint
  SITL matrix. Keep this entry open until artifacts show strict backup READY
  candidates, complete waypoint acceptance, mission completion, command-anchor
  continuity, clearance, and latency distributions.
- **Removal condition:** replace only with a stronger candidate-hull
  certificate that preserves the same KNOWN_FREE and strict dynamic gates; do
  not revert to certifying a braking suffix solely against the visibility SFC.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run planner/runtime CTest and `make test`, then run
  repeated `SPEED_CAP_MPS=3`, `4`, and `5` with
  `MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check` and
  the representative dataset shadow-planning replay.

### 2026-08-28 - Expose backup certificate rejection provenance

- **Owner:** planning backend diagnostics and runtime trace maintainers.
- **Scope:** expose bounded per-solve counts and the last rejection provenance
  for BACKUP seed feasibility, visibility-hull containment, braking-hull-aligned
  SFC construction, KNOWN_FREE swept validation, and post-refinement validation.
  The trace also carries the last braking seed duration, extrema, and endpoint.
- **Safety impact:** none to authorization semantics; positive for diagnosis.
  This change does not relax dynamic, UNKNOWN, OUT_OF_MAP, collision, freshness,
  deadline, or atomic-commit gates. A missing or zero diagnostic value is not a
  certificate pass and all existing rejection paths remain fail-closed.
- **Derivation and cost:** the three-pillar artifacts showed EXP success followed
  by backup failure but did not identify whether the seed, SFC, or KNOWN_FREE
  swept certificate was authoritative. The bounded counters add no candidate
  retries and only copy existing validation metadata into the product trace.
- **Evidence:** focused planner-trace parsing coverage now preserves the new
  fields; runtime artifacts can distinguish the rejection boundary without
  interpreting free-form log text. This entry remains open until a current-H.E.A.D
  dataset shadow and repeated three-pillar SITL run use the fields to select the
  next behavior change.
- **Removal condition:** replace only after an equivalent structured certificate
  reason is available at the same runtime boundary and historical reports no
  longer depend on these fields.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the focused planner/runtime tests, then
  `PARALLEL_WORKERS=1 MAKE_JOBS=1 make build &&
  PARALLEL_WORKERS=1 MAKE_JOBS=1 make test`, followed by the declared dataset
  replay and three-pillar SITL matrix.
### 2026-08-28 - Expose actual swept-tube blocker provenance

- **Owner:** Navigation planning/runtime maintainers.
- **Scope:** The executable-candidate swept validator now reports the actual
  inflated-grid cell center and cell state that rejected a certificate tube;
  endpoint fallback remains only for non-cell/invalid-geometry failures.
- **Safety impact:** Observability-only. Traversability policy, UNKNOWN policy,
  certificate geometry, and all fail-closed behavior are unchanged.
- **Evidence:** Focused trajectory tests cover direct tube diagnostics and
  candidate-validation propagation. Runtime dataset traces showed
  `certificate_tube_blocked` while reporting the endpoint, which was
  insufficient to distinguish UNKNOWN coverage from endpoint rejection.
- **Removal condition:** Remove only after all supported runtime artifacts
  expose equivalent blocker provenance and the rejection-stage contract no
  longer needs the diagnostic.
- **Verification:** `make build`; `make test`.

### 2026-08-28 - Shortcut A* route geometry before MINCO

- **Owner:** Navigation planning maintainers.
- **Scope:** A* grid routes are reduced with a bounded continuous
  inflated-map shortcut pass before guide-time allocation and corridor/MINCO
  generation. Shortcuts preserve the start escape prefix and never exceed the
  corridor seed-segment limit. The explicit REACH_GOAL endpoint is added
  before the final continuous edge validation.
- **Safety impact:** Behavior change intended to remove grid-induced zig-zag
  curvature and latency. Every shortcut and every retained edge uses the
  existing mission UNKNOWN policy and inflated-layer traversability oracle;
  no collision, UNKNOWN, OUT_OF_MAP, dynamic, or deadline gate is relaxed.
- **Evidence:** The 3 m/s multiwaypoint baseline reached waypoint 1 then
  produced repeated dynamic/nominal failures while guide paths were at the
  30 m geometric horizon. Focused source review found the goal edge was also
  appended after the prior validation pass.
- **Removal condition:** Remove only if repeated representative SITL and
  recorded-data distributions show no reduction in route smoothness/latency
  or reveal a regression in obstacle clearance.
- **Correction:** The initial implementation allowed a safe shortcut longer
  than the corridor seed-segment limit, which made corridor generation reject
  it as a blocked adjacent edge. The correction bounds each shortcut by the
  existing corridor contract and keeps the escape prefix intact; no safety
  gate is relaxed.
- **Verification:** `make build`; `make test`; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` with route/clearance evidence.

### 2026-08-28 - Extend pass-through guide into the outgoing leg

- **Owner:** Navigation planning maintainers.
- **Scope:** When the active pass-through waypoint is reachable and the
  supplied outgoing leg is a direct traversable segment, extend the guide
  toward the next waypoint within the existing geometric horizon. The
  extension is subdivided by the existing corridor seed-segment limit and
  leaves a dynamics-derived braking-distance buffer before the next waypoint.
- **Safety impact:** This removes an artificial stop at a pass-through
  waypoint without changing the mission acceptance contract. Every added
  segment uses the configured UNKNOWN policy and inflated-layer traversability
  oracle; corridor generation, dynamic limits, endpoint validation, and strict
  backup certification remain unchanged. If the direct continuation is not
  certifiable, the planner retains the existing current-waypoint route.
- **Evidence:** The 3 m/s multiwaypoint trace repeatedly selected a finite
  backup suffix as the current waypoint became reachable, then failed main
  replans at the short terminal route. The mission already supplies the next
  waypoint as pass-through metadata.
- **Removal condition:** Remove only if repeated multiwaypoint SITL and
  recorded-data evidence shows worse waypoint coverage, speed recovery,
  altitude tracking, or clearance than the current-waypoint-only behavior.
- **Verification:** `make build`; `make test`; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` with endpoint, role, speed, waypoint, and clearance
  evidence.

### 2026-08-28 - Recondition MINCO altitude and lateral route following

- **Owner:** Navigation planning maintainers. **Scope:** the nominal MINCO
  route-reference quality objective uses `lateral_weight=1.0` and
  `vertical_weight=10.0`; deadbands and all hard limits are unchanged.
- **Safety impact:** behavior/conditioning change only. The route reference
  remains a soft quality objective and cannot authorize a trajectory. Corridor
  containment, V/A/J, flatness, UNKNOWN/OUT_OF_MAP policy, command-anchor
  validation, swept-tube certification, and strict KNOWN_FREE backup
  certification remain unchanged.
- **Reason/evidence:** the latest 3 m/s multiwaypoint trace showed the
  collision-checked guide near z=3 m while the committed nominal/backup
  command bowed to z=1.4--2.0 m, preventing waypoint entry and reducing
  recovery speed. Earlier 10/1000 conditioning trials were inconclusive while
  the route/backup architecture was still failing; this change must be
  re-evaluated after the current route fixes.
- **Evidence required:** focused config/optimizer tests, authoritative build,
  then repeated 3/4/5 m/s multiwaypoint SITL and representative recorded-data
  replay. Inspect continuous altitude residual, speed recovery, waypoint
  coverage, clearance, backup/certificate outcomes, and planner latency.
- **Removal/review condition:** revert or retune only from a distribution that
  shows worse altitude, waypoint completion, clearance, latency, or numerical
  stability. Never compensate for a failed certificate by relaxing a gate.
- **Verification:** `make build`; `make test`; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Rebase an expired pass-through transition on measured state

- **Owner:** Navigation runtime and planning maintainers.
- **Scope:** when a new pass-through waypoint is accepted while the previous
  nominal command has no more than one configured planning-timer interval
  remaining, the runtime invokes the existing measured-state `PlanFromRest`
  path instead of hot-stitching through the previous command endpoint. The
  planning interval is derived from the runtime timer; no new safety threshold
  is introduced.
- **Safety impact:** preserves the existing PVA state from propagated odometry
  and retains the current atomic world/goal/lease/certificate gates. It avoids
  an optimizer request whose historical endpoint is already expired. The old
  command may remain exposed only under the existing hot-transition store and
  publication contracts; a failed measured-state candidate still fails closed.
  No UNKNOWN allowance, acceptance-radius change, dynamic-limit change, or
  emergency-path bypass is added.
- **Reason/evidence:** the 2026-08-28 structured run accepted waypoint 1, then
  the old command ended at that waypoint while the new waypoint-2 transition
  entered `ReplanOnce`; the first new-goal solve failed at `main_minco` with
  `committed_end=(20,5,3)` and the mission entered a safety stop. This change
  addresses that handoff boundary only; it does not claim to solve the
  separate strict `KNOWN_FREE` map-evidence gap.
- **Evidence required:** `test_planner_fsm` boundary cases, sourced runtime
  build and PX4 external-mode contract tests, then repeated 3-column SITL
  checking transition result, command gap, P/V/A continuity, waypoint order,
  strict backup certificate, altitude, clearance, and p50/p95/p99 latency.
- **Removal/review condition:** revert if repeated representative evidence
  shows a larger command gap, anchor discontinuity, waypoint regression,
  clearance regression, or worse failure behavior. Replace only after an
  explicit piecewise route handoff owns this boundary.
- **Verification:** `make build`; focused runtime FSM tests; PX4 external-mode
  tests; repeated `MAP_PROFILE=long_three_pillars_multiwaypoint
  SPEED_CAP_MPS=3 make external-mode-check`.

### 2026-08-28 - Make every hot waypoint retarget measured-state based

- **Owner:** Navigation runtime and planning maintainers.
- **Scope:** a hot transition between distinct waypoint request identities
  now enters the existing measured-state `PlanFromRest` path even if the old
  command still has nominal time remaining. The old bundle may bridge the
  publication race, but it is not used as geometric history for the new
  route boundary.
- **Safety impact:** removes an endpoint-reuse path that could keep the new
  waypoint attached to an old terminal polynomial. The existing propagated
  PVA freshness, atomic world/goal/lease, command-anchor, dynamic, corridor,
  and strict `KNOWN_FREE` checks remain unchanged. No unknown-space allowance,
  acceptance-radius change, or command validity extension is introduced.
- **Reason/evidence:** the latest 3 m/s run showed waypoint 1 accepted, then
  waypoint 2 was published while the backend still returned `ReplanOnce` with
  `committed_end=(20,5,3)` and entered a safety stop. The identity boundary is
  the correct handoff point for a measured-state solve; the independent map
  evidence gap remains open.
- **Evidence required:** FSM identity/remaining-time tests, sourced build and
  PX4 contract tests, then repeated 3-column SITL checking the transition
  mode, command gap, P/V/A continuity, waypoint order, strict backup outcome,
  altitude, clearance, and latency distributions.
- **Removal/review condition:** replace only after certified piecewise route
  assembly owns the boundary. Revert if repeated evidence shows a larger
  command gap, anchor discontinuity, waypoint regression, clearance
  regression, or worse fail-closed behavior.
- **Verification:** `make build`; focused runtime FSM and PX4 tests; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check`.

### 2026-08-28 - Make the active pass-through waypoint the trajectory boundary

- **Owner:** Navigation planning and PX4 mission-controller maintainers.
  **Scope:** a nominal solve for a pass-through goal terminates at the active
  waypoint. The outgoing direction is still supplied as the terminal velocity
  tangent, and the mission controller publishes the next goal only after the
  measured vehicle enters the active waypoint acceptance ball.
- **Safety impact:** waypoint-order and continuity correction. This removes a
  planner-only guide extension that allowed an unconstrained MINCO polynomial
  to bow around the active mission boundary. It does not relax waypoint
  acceptance, V/A/J, corridor, world, UNKNOWN/OUT_OF_MAP, freshness, or
  atomic-commit gates. The PX4-facing PVA command retains a nonzero tangent for
  pass-through goals; stop goals remain zero-terminal-velocity.
- **Reason/evidence:** the 3 m/s multiwaypoint A/B with guide extension had an
  active target `(20,5,3)` but a committed endpoint near `(34.8,4.8,3)`, and
  recorded ground truth passed the target with a minimum distance of `1.48 m`
  against the `0.9 m` acceptance radius. This prevented waypoint handoff and
  caused the later safety stop. The replacement preserves continuous velocity
  through the controller's measured waypoint transition instead of relying on
  an intermediate exact MINCO node that previously destabilized corridor/
  dynamic solving.
- **Evidence required:** focused planner/mission-controller tests, authoritative
  rebuild, then repeated 3/4/5 m/s multiwaypoint SITL and recorded-data replay.
  Inspect waypoint order/coverage, velocity continuity at handoff, speed
  recovery, altitude, clearance, strict backup outcomes, and planner latency.
- **Removal condition:** revert only if repeated representative evidence shows
  a regression in waypoint coverage, continuity, clearance, safety-stop rate,
  or latency. Do not reintroduce blind guide extension to hide missed
  acceptance.
- **Verification:** `make build`; `make test`; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Align body-clear radius with the planner safety envelope

- **Owner:** Mapping and navigation planning maintainers. **Scope:** the
  canonical ROG-Map raycasting minimum/body-clear radius is `0.80 m`, matching
  the planner-derived envelope
  (`0.35 + 0.25 + 0.05 + 0.10 + 0.05 m`). The runtime contract test prevents
  the two configuration owners from drifting below that envelope.
- **Safety impact:** correctness of local free-space evidence. This removes an
  artificial UNKNOWN ring that could intersect a strict BACKUP swept tube near
  the execution anchor. It does not classify unknown future space as free,
  permit UNKNOWN for BACKUP, alter obstacle inflation, or relax any dynamic,
  geometric, freshness, or execution gate. The existing clear is still bounded
  to the in-window body neighborhood and raycast returns remain the source of
  free-space evidence beyond it.
- **Reason/evidence:** authoritative runtime traces reported strict BACKUP
  blockers at UNKNOWN cells about `0.78--0.80 m` from the current anchor while
  `ray_range[0]` and the body-clear implementation were `0.70 m`; the planner
  collision/safety envelope was `0.80 m`. The configured simulated LiDAR range
  begins at `0.10 m`, so this alignment does not exceed the sensor's physical
  minimum range.
- **Evidence required:** focused runtime contract and ROG-Map tests,
  authoritative rebuild, then repeated 3/4/5 m/s multiwaypoint SITL and
  recorded-data replay. Inspect strict backup pass rate, blocker distance and
  state, speed/altitude continuity, clearance, waypoint coverage, and mapping
  latency. A lower UNKNOWN count alone is not acceptance.
- **Removal condition:** revert only if repeated representative evidence shows
  worse clearance, map provenance, latency, or dynamic/backup stability; do not
  restore a configuration mismatch to hide a certificate failure.
- **Verification:** `make build`; `make test`; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Account for floating-point boundary ULPs in dynamic certificates

- **Owner:** nominal/backup trajectory certificate maintainers.
- **Scope:** shared V/A/J limit comparisons accept only a bounded numerical
  boundary of 64 double-precision ULPs at the configured limit. The helper
  rejects non-finite values and material violations; it is used by nominal
  MINCO, backup refinement, and analytic braking-seed checks.
- **Safety impact:** correctness/continuity improvement. This does not change
  a physical limit, the mission envelope, the backup UNKNOWN policy, or any
  waypoint/execution gate. It prevents a candidate that reaches a limit
  exactly from being rejected solely because chained polynomial arithmetic
  produced values such as `3.0000000000000018`.
- **Reason/evidence:** the current 3 m/s trace showed bounded time-stretch
  candidates with acceleration and jerk inside the declared limits rejected
  at the final gate by sub-ULP velocity overshoot, forcing unnecessary backup
  handovers and replanning churn. The focused contract test proves that one
  ULP and a bounded 32-ULP boundary pass while a `1e-9` overshoot fails.
- **Evidence required:** repeat the same 3 m/s scenario and the 3/4/5 m/s
  ladder after the authoritative rebuild. Inspect accepted main-candidate
  ratio, speed/altitude continuity, waypoint coverage, clearance, strict
  backup certificates, and latency distributions. The change remains open
  until repeated representative evidence confirms that only numerical-boundary
  rejections are removed.
- **Removal condition:** replace only with an equivalent or stronger
  representation-aware certificate; never replace this bounded accounting with
  a configurable physical tolerance.
- **Verification:** `make build`; source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run
  `ctest --test-dir build/navigation_planning_backend --output-on-failure`,
  then repeated
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check` and the declared speed ladder.

### 2026-08-28 - Keep pass-through terminal tangent inside the search envelope

- **Owner:** navigation planning maintainer. **Scope:** pass-through MINCO
  terminal velocity uses the already validated nominal
  `optimization_dynamic_reserve_ratio` as an interior seed cap. The active
  waypoint remains the geometric endpoint, the outgoing direction still comes
  only from mission-owned look-ahead, and STOP goals remain zero-terminal-
  velocity goals.
- **Safety impact:** continuity/conditioning improvement only. The reserve is
  not a physical-limit tolerance: the generated polynomial is still checked
  against exact mission V/A/J, flatness, corridor, world, freshness, and
  atomic-commit certificates. No UNKNOWN, waypoint, braking, or execution
  gate is relaxed.
- **Reason/evidence:** repeated 3 m/s traces showed pass-through candidates
  targeting the exact velocity cap while changing direction in a short local
  guide, followed by real jerk violations and backup churn. Keeping the seed
  slightly interior gives MINCO room to shape a continuous handover without
  sacrificing the configured high-speed envelope.
- **Evidence required:** repeat the same seeded three-pillar run, then the
  3/4/5 m/s ladder and representative recorded-data replay. Inspect terminal
  velocity residual, speed recovery, jerk/acceleration margins, altitude,
  waypoint completion, clearance, backup ratio, and latency distributions.
- **Removal condition:** remove only if repeated evidence shows worse
  waypoint progress, speed recovery, continuity, clearance, or numerical
  stability; do not restore an infeasible exact-cap tangent by relaxing a
  hard gate.
- **Verification:** `make build`; `make test`; repeated
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Limit pass-through terminal velocity change by local time

- **Owner:** Navigation planning maintainers. **Scope:** pass-through terminal
  velocity now blends the outgoing tangent toward the measured incoming
  velocity when the current guide duration cannot support the full vector
  change under a conservative jerk/acceleration S-curve bound. The active
  waypoint remains the geometric endpoint and the outgoing direction remains
  toward the next mission target.
- **Safety impact:** continuity conditioning only. The bound does not alter
  the configured V/A/J limits or authorize a trajectory; MINCO's exact
  continuous V/A/J, corridor, flatness, world, cancellation and transactional
  commit gates remain authoritative. Invalid transition timing falls back to
  the existing bounded outgoing-velocity rule.
- **Reason/evidence:** orthogonal 10 m pass-through legs at `v=5 m/s`,
  `a=2 m/s^2`, `jerk=4 m/s^3` cannot rotate an incoming 5 m/s tangent into a
  full-speed orthogonal tangent within the local route. The old terminal
  condition repeatedly drove EXP hard-gate rejection and backup fallback.
- **Evidence required:** focused transition/config tests, authoritative build,
  and repeated 3/4/5 m/s multiwaypoint SITL plus recorded-data replay. Inspect
  velocity residual at waypoint handover, speed recovery, altitude, waypoint
  coverage, clearance, exact certificates, and latency.
- **Removal condition:** revert only if repeated evidence shows worse progress,
  clearance, smoothness, or processing tails than the prior checkpoint. Never
  restore an infeasible tangent by relaxing a hard gate.
- **Verification:** `source install/setup.bash && ctest --test-dir
  build/navigation_planning_backend --output-on-failure`; then repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Keep nominal dynamic search inside the physical envelope

- **Owner:** Navigation planning maintainers. **Scope:** nominal EXP soft
  dynamic penalties use `optimization_dynamic_reserve_ratio=0.98` for V/A/J;
  boundary states remain exempt up to their physically valid measured values.
- **Safety impact:** conservative search conditioning only. The independent
  V/A/J hard certificate still compares the generated trajectory against the
  exact mission limits; no tolerance, backup policy, UNKNOWN policy, corridor,
  flatness, or waypoint acceptance gate is relaxed.
- **Reason/evidence:** the 3 m/s trace showed nominal MINCO repeatedly rejected
  for small velocity overshoot (`3.0001--3.0009 m/s` against a strict 3 m/s
  limit), causing repeated backup stopping trajectories and the observed
  altitude/speed degradation. The reserve is intended to make the optimizer
  seek an interior solution instead of relying on numerical contact with the
  hard boundary.
- **Evidence required:** focused optimizer/config tests, authoritative build,
  and repeated 3/4/5 m/s SITL plus representative recorded-data replay. Check
  exact V/A/J certificates, altitude residual, speed recovery, waypoint
  coverage, clearance, backup ratio, and latency; a lower backup ratio alone
  is not acceptance.
- **Removal/review condition:** remove or retune only from a distribution that
  shows the reserve causes unacceptable speed loss, numerical instability,
  waypoint regressions, or worse clearance. Never use it to authorize a
  trajectory that fails the exact hard certificate.
- **Verification:** `make build`; `make test`; focused optimizer tests and
  repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Refresh the sensor-minimum body neighborhood per observation

- **Owner:** Mapping and navigation runtime maintainers.
- **Scope:** ROG-Map refreshes the existing sensor-minimum body-clear
  neighborhood around every accepted corrected-odometry observation, and the
  immutable snapshot changed-region certificate includes that local volume.
  The clear radius remains the configured `raycast_range_min`; no unknown
  future space is classified as free.
- **Safety impact:** This repairs the evidence handoff between the moving map
  pose and the execution anchor. It does not allow UNKNOWN for backup,
  change obstacle inflation, change the raycast radius, or relax any planner
  or execution gate. A body-clear update remains bounded to the in-window
  sensor-minimum neighborhood and is visible to snapshot patch publication.
- **Reason/evidence:** Repeated 2/3 m/s SITL traces were blocked at the first
  backup anchor by `UNKNOWN` cells within approximately 0.5 m of the current
  command start, while mapping otherwise published fresh observations. The
  prior implementation only refreshed this neighborhood on the first frame or
  after a map slide, so a pose shift within the sliding window could leave the
  strict KNOWN_FREE certificate stale.
- **Removal condition:** Revert only if repeated representative SITL and
  recorded-data distributions show a regression in obstacle clearance,
  mapping latency, or map provenance. Do not replace this with an UNKNOWN
  allowance.
- **Verification:** `make build`; focused `rog_map_vendor` and
  `navigation_mapping` tests; authoritative repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check`, checking body-clear diagnostics, snapshot revisions,
  strict backup certificate outcomes, clearance, and callback latency.

### 2026-08-28 - Preserve active-waypoint altitude in certified backup braking

- **Owner:** Navigation planning maintainers.
- **Scope:** the minimum-snap backup seed may add a degree-7 C^3 smoothstep
  only in the altitude axis so its terminal position approaches the active
  planning waypoint altitude, and may search a bounded extension of its
  already certified duration when the unmodified duration cannot satisfy the
  exact envelope. The measured handover PVAJ, terminal V/A/J=0, SFC hull
  containment, exact dynamic limits, and strict KNOWN_FREE swept certificate
  remain required.
- **Safety impact:** quality/continuity conditioning only. The altitude-held
  seed is selected only when its analytic velocity, acceleration and jerk
  extrema satisfy the existing backup envelope. If it does not, the existing
  free-end seed is retained; no gate, map policy, or waypoint acceptance
  threshold is relaxed.
- **Reason/evidence:** the authoritative 3 m/s multiwaypoint trace showed
  repeated certified backup commits with terminal altitude roughly 1.5--4.1 m
  while the active route stayed near 3 m, so the vehicle did not enter the
  next waypoint acceptance region. The change addresses that recovery path
  without replacing the braking or MINCO architecture.
- **Evidence required:** focused braking/config tests, authoritative build,
  then repeated 3/4/5 m/s multiwaypoint SITL and representative recorded-data
  shadow planning. Inspect terminal altitude residual, speed recovery,
  waypoint coverage, clearance, exact V/A/J certificate outcomes, backup
  refinement fallback, and latency.
- **Removal condition:** revert or retune only from a distribution showing
  worse clearance, dynamic margin, numerical stability, waypoint coverage,
  altitude recovery, or latency. Never compensate for a failed altitude-held
  seed by relaxing a hard gate.
- **Verification:** `make build`; `make test`; repeated
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make
  external-mode-check` followed by the declared speed ladder.

### 2026-08-28 - Carry explicit no-return visibility as miss-only map evidence

- **Owner:** Estimation, mapping, and simulation integration maintainers.
- **Scope:** `RegisteredScan` may carry an optional
  `free_space_endpoints` cloud registered in the same public world frame and
  timestamp as the hit cloud. `MappingObservation` preserves that distinction
  through the mapping actor. ROG-Map consumes the endpoints only as miss-only
  rays; the endpoint is never inserted as occupied. An absent or empty field
  means no explicit visibility evidence and does not classify UNKNOWN as free.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. This closes the current
  evidence gap where endpoint-only point returns leave the open corridor
  UNKNOWN even when the sensor reports no return to a longer range. A false
  accept is prevented by keeping the contract typed, frame/timestamp checked,
  and by separating hit and no-return inputs. A false reject remains possible
  when the optional producer is absent, malformed, outside the local update
  box, or clipped; that consequence is a bounded stop rather than an
  unverified command. No UNKNOWN policy, OUT_OF_MAP handling, backup gate, or
  planner threshold is relaxed.
- **Derivation and cost:** endpoint evidence is only authoritative when a
  sensor/bridge producer explicitly supplies it; point-cloud hit endpoints are
  not reinterpreted. The current implementation adds one bounded vector and
  one ray traversal per supplied endpoint, with the existing map update,
  clipping, and batch cache path reused. Record free-space endpoint counts and
  processing tails in mapping diagnostics before selecting a beam downsample.
- **Evidence:** contract and ROG vendor smoke coverage proves timestamp/frame
  plumbing and miss-only behavior; authoritative build, dataset replay,
  repeated structured SITL, sanitizers, and hardware visibility certification
  remain open. The optional field must remain empty for recorded data unless
  its source has an equivalent no-return contract.
- **Removal/review condition:** remove or change this contract only after a
  replacement typed visibility certificate covers no-return semantics,
  sensor pose/time registration, blind zones, and map-generation provenance.
  Never replace it with blanket free-space insertion or UNKNOWN allowance.
- **Verification:** `make build`; focused
  `colcon test --packages-select rog_map_vendor navigation_mapping
  navigation_contracts --event-handlers console_direct+`; then repeat the
  declared dataset and `long_three_pillars_multiwaypoint` SITL ladder while
  checking `mapping_free_space_*`, strict backup certificate outcomes,
  clearance, altitude, waypoint completion, and p50/p95/p99 callback latency.

### 2026-08-28 - Use a native 3D Gazebo visibility producer

- **Owner:** Simulation and estimation integration maintainers.
- **Scope:** The Mid-360 simulation path subscribes directly to
  `gz.msgs.LaserScan` and publishes a bounded `PointCloud2` of explicit
  no-return endpoints in `livox_frame`. It validates the frame, timestamp,
  horizontal/vertical counts, flattened array length, finite scan metadata,
  and max-range semantics before publication. The generic
  `gz.msgs.LaserScan -> sensor_msgs/LaserScan` bridge is intentionally not used
  for this 3D sensor. The producer uses deterministic sampling capped at 4096
  endpoints per scan; this can only omit evidence, never manufacture it.
- **Safety impact:** `SAFETY_INVARIANT`, fail closed. A malformed, incomplete,
  empty, stale, or unsupported scan publishes no evidence; the downstream
  typed contract still requires exact frame/timestamp matching and ROG still
  applies miss-only rays. This does not relax UNKNOWN, OUT_OF_MAP, backup, or
  planner gates. Avoiding the 2D converter also removes a process-crash path
  that could otherwise interrupt the sensor/runtime stream.
- **Evidence:** A direct converter probe with a synthetic 720 x 28 Gazebo
  message reproduced a segmentation fault in the installed generic bridge;
  native producer unit tests cover 3D geometry, incomplete arrays, wrong frame,
  and all-hit scans. The authoritative build, full structured SITL ladder,
  sanitizer coverage, and hardware visibility certificate remain open.
- **Removal/review condition:** replace only with a typed 3D visibility source
  that preserves vertical metadata, no-return semantics, timestamp/frame
  provenance, and bounded resource use. Reconsider the 4096 cap only from
  repeated latency/coverage distributions, never from one mission run.
- **Verification:** `make build`; sourced colcon tests for `uav_simulation`,
  `fast_lio_ros`, `navigation_contracts`, `navigation_mapping`, and
  `rog_map_vendor`; then repeated structured SITL while checking
  `mapping_free_space_*`, process liveness, waypoint completion, clearance,
  altitude, and latency tails.

### 2026-08-28 - Preserve nominal command continuity across measured pass-through handoff

- **Owner:** Runtime execution and mission/planning maintainers.
- **Scope:** When the mission controller has measured acceptance of a
  pass-through waypoint and the current command is nominal, the runtime may
  atomically rebind the retained immutable command bundle to the next
  `(goal_epoch, request_id)` and call `ReplanOnce` using the existing command
  history. The bundle evaluator, executable interval, world certificate, and
  safety suffix are copied unchanged. `PlanFromRest` remains selected when the
  retained command is at its execution boundary, unavailable, or not an exact
  previous-goal identity match.
- **Safety impact:** `SAFETY_INVARIANT` preservation, not a gate relaxation.
  Rebinding is permitted only after measured position acceptance and only for
  a nominal bundle with no planner failure or safety suffix. An exact previous
  goal-epoch check prevents relabeling an unrelated command. A false reject
  causes a bounded measured-state replan; a false accept would expose a
  command after an unproven waypoint transition, so the store rejects all
  mismatched identities and never extends the certified interval.
- **Derivation and cost:** The transition proof is the existing mission
  acceptance contract plus `canHotRetargetAtWaypointTransition`; no new
  safety threshold or UNKNOWN policy is introduced. Rebinding is one immutable
  candidate copy and constant-time pointer replacement. The old policy forced
  every identity change through `PlanFromRest`, discarding valid velocity
  continuity and producing a stop/replan loop on the 3 m/s structured route.
- **Evidence:** `CommittedBundleStore` and planner-FSM focused tests pass after
  the change. The first post-change SITL still blocked at waypoint 2, with
  357/357 mapping updates, LIO TRACKING, and `main_minco` dynamic failures;
  this entry therefore does not claim mission acceptance. Repeated SITL,
  dataset shadow planning, sanitizer, and hardware evidence remain open.
- **Removal/review condition:** Revert or redesign only if repeated runtime
  traces show identity leakage, command-anchor discontinuity, worse clearance,
  dynamic violations, or latency tails. The next review must inspect waypoint
  acceptance coverage, command generation continuity, speed recovery, altitude,
  clearance, and p50/p95/p99 planning latency.
- **Verification:** `make build`; sourced focused execution/runtime tests;
  then repeated `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` and the declared speed ladder. Do not convert a
  single successful run into acceptance.

### 2026-08-28 - Let the next hot-replan own pass-through corner rotation

- **Owner:** Mission controller and planning maintainers.
- **Scope:** For a pass-through waypoint whose incoming and outgoing route
  tangents form a genuine corner, the current MINCO solve keeps an incoming
  tangent terminal velocity instead of forcing the outgoing tangent at the
  same endpoint. The mission controller advances on finite measured velocity
  plus measured position acceptance; the newly published next goal owns the
  continuous turn through the existing nominal hot-replan path. Stop waypoints
  retain the measured low-speed hold contract.
- **Safety impact:** `SAFETY_INVARIANT` preservation, not a collision or
  UNKNOWN-policy relaxation. Waypoint acceptance is still position-measured,
  no future waypoint is accepted from look-ahead alone, and every replacement
  trajectory still requires dynamic, corridor, backup, and immutable-world
  certification. The false-reject consequence is a temporary stop/replan;
  the false-accept risk is bounded by the measured acceptance-radius check and
  exact goal-identity handoff.
- **Derivation and cost:** The existing corner classification is retained in
  the planner with the current tangent comparison; no new hard threshold is
  introduced. Removing the outgoing terminal vector from the incoming solve
  prevents a single polynomial from simultaneously reaching a sharp corner
  and rotating its velocity at that same boundary. The next solve pays the
  existing hot-replan cost; no extra planning pass is added.
- **Evidence:** Focused mission tests (31/31), planner configuration tests
  (25/25), and trajectory tests (55/55) pass. The subsequent structured SITL
  must check whether corner acceptance and speed continuity improve without
  reducing clearance; repeated SITL, dataset shadow planning, sanitizer, and
  hardware evidence remain open.
- **Removal/review condition:** Revert or redesign if measured acceptance
  skips a waypoint, command-anchor or velocity continuity breaks, clearance or
  altitude worsens, or the hot-replan latency tail exceeds its budget. Do not
  reintroduce outgoing endpoint velocity merely to increase mission PASS rate.
- **Verification:** `make build`; sourced focused mission/planner tests; then
  repeated `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check` with waypoint coverage, speed, altitude, clearance,
  command generations, and p50/p95/p99 planning latency.

### 2026-08-28 - Bound short-corner terminal speed by remaining guide distance

- **Owner:** Planning maintainer.
- **Scope:** A pass-through corner keeps the incoming tangent, but its terminal
  speed is additionally bounded by the existing path-length/time feasibility
  estimate before MINCO initialization. The bound applies only to the active
  guide endpoint; it does not alter mission acceptance, product velocity,
  acceleration, jerk, corridor, backup, or immutable-world gates.
- **Safety impact:** Removes an impossible P/V seed that caused repeated main
  optimizer rejection and command drain near a sharp waypoint. The fallback is
  fail-closed: if the resulting candidate is not dynamically and world
  certified, it is not published. No UNKNOWN, clearance, or acceptance gate is
  relaxed.
- **Derivation and cost:** For path length `L`, guide duration `T`, and measured
  incoming tangent speed `v0`, use the existing conservative cap
  `clamp(2L/T-v0, 0, vmax)`. The guide scan is linear in the existing path
  samples and adds no planning pass.
- **Evidence:** Focused planner configuration test covers the short-boundary
  cap. Full build and structured SITL are required before treating the corner
  handoff as stable; repeated runs, dataset shadow planning, sanitizer, and
  hardware evidence remain open.
- **Removal/review condition:** Revisit if the cap creates a measurable stop,
  lowers speed on long feasible legs, causes route skipping, worsens altitude or
  clearance, or fails to remove repeated `PLANNER_EXP_FAILED` at short corner
  boundaries.
- **Verification:** `make build`; sourced planner/mission/trajectory tests;
  repeated `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check` with waypoint, speed, altitude, clearance, and planner
  latency evidence.

### 2026-08-28 - Use a bounded acceptance route window for pass-through corners

- **Owner:** Navigation planning, runtime execution, and mission-controller
  maintainers.
- **Scope:** When the current pass-through guide reaches the active waypoint,
  a genuine corner may use a bounded three-point route-window fillet inside
  the mission acceptance ball: an incoming-tangent entry point, an outgoing
  tangent blend point, and an outgoing-tangent endpoint at 75% of the
  acceptance radius. Every point and each predecessor/fillet segment is
  required to be traversable before corridor and trajectory generation. The
  endpoint is accepted only after the measured mission state enters the
  requested waypoint's acceptance ball. Shallow bends, small acceptance
  balls, adjusted occupied/out-of-map goals, and failed point/segment checks
  retain the exact-goal path.
- **Safety impact:** `SAFETY_INVARIANT` preservation, not a gate relaxation.
  This removes the zero-geometric-room boundary condition that forced a
  second solve to turn a nonzero incoming velocity at the exact corner. The
  route window cannot advance a mission checkpoint by itself; the runtime and
  mission controller continue to compare measured position with the requested
  waypoint. The candidate still requires the existing dynamic, corridor,
  strict-backup, swept-world, identity, and lease certificates. No UNKNOWN,
  OUT_OF_MAP, clearance, speed, acceleration, jerk, or deadline gate changes.
- **Derivation and cost:** The fillet uses a fixed interior ratio and three
  route-window points so the corridor sees the incoming, blend, and outgoing
  geometry explicitly. Each of the three short segments is checked once and
  the existing MINCO/corridor/backup path is reused; no second planner or
  optimizer pass is added. The ratio is geometric point placement, not a tuned
  acceptance or safety threshold, and must be revisited only with
  route/acceptance distributions.
- **Evidence:** The route-window helper tests and planner configuration suite
  pass (28/28). A post-change structured SITL run is required to verify all
  nine waypoint transitions, speed continuity, altitude, clearance, and
  latency; this entry does not claim mission completion until that evidence is
  repeated.
- **Removal/review condition:** Revert or redesign if measured acceptance
  skips a waypoint, any fillet point is outside the requested ball, command/
  world identity or PVA continuity breaks, clearance/altitude worsens, or the
  route-window solve increases planner tail latency or optimizer failures.
- **Verification:** `make build`; sourced planner, mission, and trajectory
  tests; then repeated `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` and the declared speed ladder, checking route
  window endpoints, waypoint order, command generations, speed, altitude,
  clearance, strict backup evidence, and p50/p95/p99 latency.

### 2026-08-28 - Extend pass-through guides through a certified route lookahead

- **Owner:** Navigation planning, runtime execution, and mission-controller
  maintainers.
- **Scope:** A `PASS_THROUGH` goal may extend its executable guide from the
  active waypoint into the outgoing segment supplied by `next_target`. The
  extension length is bounded by both the remaining 45 m planning horizon and
  the outgoing route length, and its minimum requested margin is derived from
  the current bounded speed stopping distance, two replan-forward intervals,
  and the configured 3 m receding prefix. A* supplies the route prefix; every
  appended segment is checked against the inflated-layer/UNKNOWN policy before
  guide time allocation. If no certified extension fits, the existing exact
  goal or bounded acceptance-window path remains in force.
- **Safety impact:** This preserves the mission identity and acceptance
  contract while removing the need to rotate a high terminal velocity inside
  a sub-metre waypoint ball. The current waypoint is never completed by the
  planner endpoint: MissionController still requires measured position inside
  the waypoint acceptance radius. The candidate retains the existing dynamic,
  corridor, swept-world, strict-backup, identity, and lease certificates. No
  UNKNOWN, OUT_OF_MAP, clearance, speed, acceleration, jerk, or deadline gate
  is relaxed.
- **Derivation and cost:** The route margin is
  `jerk_limited_stop_distance(v) + 2*v*replan_forward_dt + receding_distance`;
  it is clamped by available outgoing route and remaining planning horizon.
  One bounded A* route-prefix query and one linear segment certificate are
  added only for pass-through goals. The 45 m horizon is a geometric capacity
  change required to hold the active segment plus that certified prefix, not a
  safety threshold; map geometry validation still owns the hard extent check.
- **Evidence:** The lookahead helper test, planner configuration suite,
  trajectory suite, mission suite, full build, and authoritative manifest must
  pass. Repeated 3 m/s multi-waypoint SITL and recorded-data shadow planning
  remain required before claiming stable high-speed completion.
- **Removal/review condition:** Revert or redesign if the lookahead crosses a
  waypoint without measured acceptance, breaks command/world identity or PVA
  continuity, increases optimizer latency/failure tails, worsens clearance or
  altitude, or causes the planner to consume map horizon needed by the backup
  certificate.
- **Verification:** `make build`; sourced planner, trajectory, mission, and
  runtime contract tests; then repeated
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check`, checking waypoint order, endpoint error, route length,
  speed, altitude, clearance, strict backup evidence, and planning p50/p95/p99.

### 2026-08-28 - Keep shallow pass-through legs anchored at their measured waypoint

- **Owner:** Navigation planning and mission-controller maintainers.
- **Scope:** The long certified route look-ahead is enabled only when the
  outgoing tangent forms a genuine corner (`dot <= 0.7`). Straight or shallow
  pass-through legs keep the active waypoint as the executable endpoint while
  preserving the outgoing tangent in the terminal velocity state.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This prevents a soft
  look-ahead endpoint from allowing the nominal polynomial to bow outside the
  active waypoint acceptance ball before measured handoff. No waypoint
  acceptance, UNKNOWN, OUT_OF_MAP, clearance, speed, acceleration, jerk, or
  deadline gate is relaxed; all existing corridor, dynamic, swept-world,
  identity, and lease certificates remain authoritative.
- **Evidence:** The shared corner classifier and planner configuration tests
  cover shallow and genuine turns. The latest 3 m/s run showed the previous
  all-leg look-ahead still missed the first waypoint (`min measured error just
  above the 0.9 m acceptance radius`) and later stopped at its soft endpoint;
  repeated SITL and recorded-data evidence remain required after this change.
- **Removal/review condition:** Revisit if straight legs still miss measured
  acceptance, if corner look-ahead increases optimizer failure tails, or if
  route length, PVA continuity, altitude, clearance, or waypoint order
  regresses.
- **Verification:** `make build`; sourced planner/mission/trajectory tests;
  then repeated `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` with waypoint, speed, altitude, clearance, strict
  backup, and p50/p95/p99 latency evidence.

### 2026-08-28 - Preserve a hard route-boundary gate at pass-through corners

- **Owner:** Navigation planning and corridor-generation maintainers.
- **Scope:** When a genuine pass-through corner uses certified route look-ahead,
  the corridor generator stops the incoming line corridor at the active
  waypoint, inserts a point-derived free-space corridor, and marks it with a
  route-boundary junction contract containing the waypoint and acceptance
  radius. The gate is matched to an interior guide sample and the incoming
  overlap is checked before the corridor is published. If A* allocation leaves
  a long incoming edge, only that edge is subdivided into bounded seed
  segments before gate construction. SFC simplification preserves the complete
  corridor sequence whenever the marker is present, and corridor compaction
  cannot pop a gate while appending the outgoing leg.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This closes the route
  adherence hole where a single convex free-space corridor allowed MINCO to
  cut a genuine waypoint corner without entering the mission acceptance ball.
  The hard junction check requires one of the gate's adjacent polynomial
  junctions to enter that ball, while the point-derived corridor is no longer
  incorrectly used as a full-piece sub-metre turn box. Collision, UNKNOWN,
  OUT_OF_MAP, clearance, dynamic, swept-world, backup, identity, and deadline
  gates remain unchanged and authoritative. Invalid or unmatchable gate
  geometry fails closed.
- **Derivation and cost:** The route-boundary point and radius are carried as
  explicit polytope metadata. The guide seed places the incoming junction at
  the mission waypoint, the objective penalizes only an outside-ball junction,
  and the independent post-solve certificate checks both adjacent junctions.
  One bounded point-corridor construction and one bounded junction check are
  added only for an active look-ahead boundary; no second optimizer or planner
  is introduced. The marker is retained as metadata so generic SFC
  simplification cannot erase the contract. During MINCO hot initialization,
  the overlap immediately after a marked gate starts its guide-sample search
  one sample later. If that sample is also the final look-ahead endpoint, the
  remaining guide interval is split at its midpoint. This preserves positive
  time for both the outgoing turn and the final SFC piece instead of accepting
  the generic 0.01 s duration clamp.
- **Timing safety impact:** `SAFETY_INVARIANT` preservation. The timing change
  does not relax any dynamic or corridor gate and does not alter the mission
  acceptance radius. It prevents a zero-time seed from presenting an
  artificially infeasible high-jerk corner to the optimizer; the final
  continuous physical V/A/J and world certificates remain authoritative.
- **Evidence:** The SFC preservation unit test passes with the existing planner
  and trajectory suites, and the EXP seed test covers post-boundary guide time
  allocation. The current EXP package has a pre-existing exact-cap
  multi-corridor fixture failure (physical acceleration 2.0019/2); it is kept
  visible rather than masked. The next authoritative build/manifest and
  repeated 3 m/s multi-waypoint SITL must verify that the gate is constructed,
  waypoint order is preserved, and optimizer latency/failure tails remain
  bounded.
- **Removal/review condition:** Revisit if gate construction rejects a valid
  route due to numerical overlap, causes repeated optimizer failure or latency
  tails, permits measured waypoint skipping, or regresses PVA continuity,
  altitude, clearance, speed, strict backup, or route completion.
- **Verification:** `make build`; sourced planner, trajectory, mission, and
  runtime contract tests; then repeated
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check`, checking route-boundary diagnostics, waypoint order,
  endpoint error, speed, altitude, clearance, strict backup, and planning
  p50/p95/p99.

### 2026-08-28 - Hold the route-boundary junction through MINCO optimization

- **Owner:** Nominal trajectory optimizer and navigation planning maintainers.
  **Scope:** For a marked route-boundary corridor cell, the incoming adjacent
  MINCO junction is reconstructed at the recorded mission waypoint on every
  objective/rebuild; its optimizer gradient is cleared, and the point must be
  valid in the incoming overlap. The independent final gate still verifies
  that an adjacent junction lies inside the waypoint acceptance ball.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This prevents numerical
  optimization from moving the only route-boundary junction out of the
  mission acceptance region after a valid seed was constructed. The surrounding
  corridor remains free-space geometry for smooth fillet generation. No
  mission acceptance, UNKNOWN, OUT_OF_MAP, clearance, V/A/J, flatness,
  swept-world, backup, identity, freshness, command-anchor, or deadline gate is
  relaxed.
- **Derivation and cost:** The route-boundary point is a mission-owned
  geometric junction, not a free corridor control point. Reapplying it after
  corridor parameterization makes that invariant independent of L-BFGS drift;
  clearing its spatial gradient avoids an optimizer request to move a fixed
  variable, while time variables remain free to find a certified smooth turn.
  The bounded work is one metadata pass over route-boundary cells per objective
  evaluation and does not add a planner or optimizer pass.
- **Evidence:** Add/retain route-boundary metadata and SFC-preservation tests;
  run the planner backend suite, authoritative build/manifest, and the
  structured three-pillar SITL. This entry remains open until the artifact
  shows waypoint 2 and later transitions, bounded optimizer latency/failure
  tails, speed recovery, altitude, clearance, strict backup, and command
  continuity.
- **Removal/review condition:** Revisit if fixed-junction parameterization
  makes a valid corridor infeasible, increases optimizer tails, causes a stop
  or waypoint skip, or regresses altitude, clearance, PVA continuity, or route
  completion. Do not replace the hard junction with a soft-only penalty.
- **Verification:** `make build`; source `install/setup.bash` and run the
  planner, mission, trajectory, and runtime contract tests; then repeat
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check`, inspecting route-boundary junctions, waypoint
  order/coverage, speed, altitude, clearance, strict backup, and p50/p95/p99
  planning latency.

### 2026-08-28 - Shape corner terminal speed before outgoing recovery

- **Owner:** Navigation planner terminal-state and pass-through handoff
  maintainers. **Scope:** When a genuine pass-through corner has an active
  certified route look-ahead, the frontier terminal-state seed is capped by
  `min(max_velocity, sqrt(acceptance_radius * max_acceleration))` before the
  measured waypoint handoff. The cap applies only to the look-ahead
  parameterization; the following replan may recover speed on the outgoing
  tangent through the existing jerk/acceleration-limited continuity helper.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This removes a
  physically over-constrained full-cruise terminal request at a sub-metre
  route-boundary turn. It does not change mission acceptance, product V/A/J
  limits, collision, UNKNOWN, OUT_OF_MAP, clearance, corridor, backup,
  freshness, command-anchor, or deadline gates. A candidate remains rejected
  unless the existing strict continuous certificates pass.
- **Derivation and cost:** The terminal seed must leave geometric room for a
  heading change inside the current acceptance region. The bound follows the
  centripetal envelope `v^2/r <= a` using the existing acceptance radius as a
  bounded local-room proxy; it is a seed guard, not a new acceptance threshold
  or a physical-certificate allowance. The change adds one bounded square-root
  calculation per route-lookahead solve and does not add a planner pass.
- **Evidence:** Add focused helper coverage for the acceptance-room cap and
  invalid inputs. Rebuild the authoritative manifest and repeat planner,
  trajectory, mission, runtime, and structured three-pillar SITL evidence;
  this entry remains open until waypoint completion, speed recovery, altitude,
  clearance, optimizer failure/latency tails, strict backup, and command
  continuity are measured repeatedly.
- **Removal/review condition:** Revisit if the cap creates a stop or speed
  deadlock on feasible legs, fails to remove route-boundary optimizer failures,
  worsens altitude/clearance, increases latency tails, or permits a waypoint
  skip. Do not raise the cap from a single SITL run.
- **Verification:** `make build`; source `install/setup.bash` and run focused
  planner/trajectory/mission/runtime tests; then repeat
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check` and the declared dataset shadow-planning replay,
  inspecting corner cap, route-boundary diagnostics, waypoint order, speed,
  altitude, clearance, strict backup, and p50/p95/p99 planning latency.

### 2026-08-28 - Normalize every guide edge before bounded corridor seeding

- **Owner:** Corridor-generation and navigation-planning maintainers.
- **Scope:** Before CIRI corridor construction, every finite guide-path edge is
  deterministically subdivided so each line seed is no longer than the existing
  `seed_line_max_length` contract. Exact source endpoints, including marked
  pass-through route boundaries, are retained; a bounded per-edge subsegment
  limit rejects malformed or unbounded input.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This closes a frontend
  consistency hole where a collision-free A* guide with a sparse final edge
  could be rejected by the corridor hard bound after the planner had already
  produced a valid visible prefix. It does not permit an overlong CIRI seed,
  bypass occupancy/UNKNOWN/OUT_OF_MAP checks, or relax overlap, corridor,
  dynamic, swept-world, backup, identity, freshness, anchor, or deadline gates.
- **Derivation and cost:** The maximum segment length is the existing bounded
  corridor-seed owner; inserted samples are linear interpolation only and do
  not change route geometry. At most `ceil(edge_length / max_seed_length)`
  bounded samples are added per edge, with no extra optimizer or planner pass.
- **Evidence:** The geometry characterization tests cover a 6 m waypoint edge
  and a following 5.7 m outgoing edge with a 3 m seed bound, exact endpoint
  preservation, maximum edge length, and invalid-input rejection. Build and
  planner/trajectory/runtime tests must pass; the structured three-pillar SITL
  must show no `seed line max` rejection and must still verify route completion,
  speed, altitude, clearance, strict backup, and p50/p95/p99 latency.
- **Removal/review condition:** Revisit if subdivision changes waypoint
  ordering or PVA continuity, increases CIRI/optimizer tails, creates false
  corridor feasibility, or regresses collision/clearance or mission
  completion. Do not increase the seed bound to hide a sparse-guide failure.
- **Verification:** `make build`; source `install/setup.bash` and run focused
  planner/trajectory/mission/runtime tests; then repeat
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check`, inspecting bounded seed diagnostics, waypoint order,
  speed recovery, altitude, clearance, strict backup, and p50/p95/p99 planning
  latency.

### 2026-08-28 - Bridge measured-state rebases with a certified C3 handoff

- **Owner:** Navigation planning hot-replan and runtime execution maintainers.
  **Scope:** When a hot replan must rebase from the committed command to the
  fresh propagated state because the existing tracking-error budget has been
  exceeded, prepend one seventh-order PVAJ connector between the old command
  state and the new measured-state trajectory. The connector is used only for
  a valid finite source/target pair; if no duration passes the existing
  nominal dynamic and flatness certificates, the rebase solve fails closed and
  the previously certified command remains the only fallback.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This removes the
  observed cross-generation command jump without enlarging the PX4 0.75 m
  command-anchor envelope, changing waypoint acceptance, permitting UNKNOWN,
  or altering corridor, swept-world, backup, freshness, identity, or deadline
  gates. The complete connector plus nominal/backup candidate is still
  authorized by the immutable WorldModel; an invalid connector is never
  published.
- **Derivation and cost:** The connector solves exact P/V/A/J endpoint
  constraints and increases its duration only while the existing V/A/J and
  flatness limits reject the candidate, with a bounded 24-attempt search.
  Initial duration is derived from displacement, velocity/acceleration delta,
  current dynamic limits, and the existing sample period. This adds work only
  on measured-state rebase events; normal hot replans and the map certificate
  are unchanged. No new tunable safety threshold is introduced.
- **Evidence:** The exact-head artifact
  `.artifacts/runtime/external-mode-check-20260828T020340-574857` recorded
  rebase transitions with 0.25--0.29 m position and 0.27--0.67 m/s velocity
  jumps before the change, while trajectory failures were zero after the
  backup-corridor ownership fix. The new state-transition characterization
  tests must pass, followed by a rebuilt authoritative manifest and repeated
  3-column SITL checking inter-generation P/V/A/J continuity, speed recovery,
  altitude, clearance, strict backup, waypoint order, and latency tails. This
  entry does not claim mission completion from the pre-change artifact.
- **Removal/review condition:** Revisit if the connector increases command
  gaps, causes repeated rebase failure, worsens altitude/clearance, consumes
  the planning budget, or hides an estimator/PX4 tracking defect. Replace it
  only after an explicit controller-owned time-scaling or piecewise route
  handoff has repeated SITL, recorded-data, sanitizer, and hardware evidence.
- **Verification:** `make build`; source `install/setup.bash` and run the
  planner backend, execution, runtime, and PX4 contract tests; then repeat
  `SPEED_CAP_MPS=3 MAP_PROFILE=long_three_pillars_multiwaypoint make
  external-mode-check`, inspecting command-generation boundaries, P/V/A/J
  residuals, speed, altitude, clearance, strict backup evidence, and
  p50/p95/p99 planning latency.

### 2026-08-28 - Preserve the certified handoff prefix before BACKUP

- **Owner:** Navigation planning trajectory composition and runtime execution
  maintainers.
- **Scope:** Propagate the duration of a measured-state PVAJ handoff from
  `ExpTraj` into backup selection and candidate construction. A BACKUP suffix
  may begin only at or after that duration; candidate construction rejects a
  backup start that would truncate the prefix. The backup optimizer and the
  final immutable-world authorization still certify the resulting suffix.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This closes a
  trajectory ownership hole that silently discarded a certified continuity
  connector when the visibility-derived switch was near the current command
  time. It does not relax KNOWN_FREE backup, UNKNOWN/OUT_OF_MAP, dynamic,
  flatness, corridor, swept-world, identity, freshness, command-anchor,
  waypoint, or deadline gates. If no certified backup window remains after
  the prefix, the solve fails closed.
- **Derivation and cost:** The lower backup switch bound is the maximum of
  the current command time and the recorded connector duration. It is applied
  to the bounded backward switch search and to the optimizer's existing switch
  interval; no new planner pass or safety threshold is introduced. Candidate
  construction repeats the invariant as a fail-closed composition check.
- **Evidence:** The post-connector SITL artifact
  `.artifacts/runtime/external-mode-check-20260828T022542-589388` showed most
  rebase transitions at `p95 Δp=0.02265 m`, `p95 Δv=0.02943 m/s`, but one
  BACKUP transition still at `Δp=0.45318 m`, `Δv=0.27525 m/s`; commit traces
  showed the backup switch near zero, proving that the connector was being
  truncated during composition. That run is invalid for acceptance because
  lidar and external-odometry freshness gaps were present. Focused candidate
  composition tests, a rebuilt manifest, and a fresh valid SITL run are
  required.
- **Removal/review condition:** Revisit if enforcing prefix ownership causes
  repeated no-window failures on representative routes, increases planning
  tails, or regresses speed, altitude, clearance, strict backup, PVA
  continuity, or mission completion. Do not lower the prefix bound to recover
  progress; change the handoff/route architecture only with new evidence.
- **Verification:** `make build`; source `install/setup.bash` and run the
  planner backend, trajectory, execution, runtime, and PX4 contract tests;
  then repeat `SPEED_CAP_MPS=3
  MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check`,
  accepting results only when sensor freshness is valid and inspecting backup
  switch time versus prefix duration, generation P/V/A/J residuals, speed,
  altitude, clearance, waypoint order, strict backup, and p50/p95/p99
  planning latency.

### 2026-08-28 - Apply adaptive pass-through lookahead to straight outgoing legs

- **Owner:** Navigation planner route-window and pass-through handoff
  maintainers. **Scope:** When a pass-through goal has a finite next target,
  derive the same bounded outgoing look-ahead from measured/planned speed,
  stopping distance, replan-forward time, and receding distance for straight,
  shallow-bend, and genuine-corner legs. Append only the prefix returned by a
  bounded A* query and preserve the active waypoint as an explicit route-
  boundary gate. Genuine corners retain the existing acceptance-room terminal
  speed cap; straight/shallow legs keep the outgoing guide tangent for cruise
  continuity.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This removes the
  artificial exact-waypoint endpoint that caused a straight three-column leg
  to shrink as the vehicle accelerated. It does not change waypoint identity
  or measured acceptance, and does not relax UNKNOWN, OUT_OF_MAP, corridor,
  swept-world, dynamic, flatness, backup, freshness, command-anchor, lease, or
  deadline gates. If the outgoing prefix is not independently certified, the
  planner retains the existing exact-boundary/fallback behavior.
- **Derivation and cost:** The route prefix length is the existing
  `passThroughLookaheadDistance` envelope clamped by the available outgoing
  route and planning horizon. The change removes only the corner predicate
  from look-ahead eligibility; it adds at most one bounded outgoing A* query
  and segment-certificate pass per pass-through solve, with no new optimizer
  pass or threshold. Planning-tail impact must be measured before release.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T024937-611666` confirmed
  `behavior=0` with `next_target=(50,5,3)` and an input trace at
  `guide_end=(20,5,3)`, but no route-lookahead activation because the leg was
  collinear. The run reached approximately 3.1 m/s and later failed closed
  when repeated hot replans could not produce a fresh certified command; it
  is not mission-acceptance evidence. The new collinear policy test, clean
  manifest, and repeated valid SITL/dataset evidence are required.
- **Removal/review condition:** Revisit if straight/shallow look-ahead causes
  waypoint skips, route-boundary corridor failures, bowing outside the
  acceptance ball, more optimizer or backup failures, worse altitude or
  clearance, increased p95/p99 latency, or no reduction in command
  exhaustion/replan churn. Do not raise look-ahead or relax a certificate from
  a single run.
- **Verification:** `make build`; source `install/setup.bash` and run the
  planner backend, trajectory, runtime, mission, and PX4 contract tests; then
  repeat `SPEED_CAP_MPS=3
  MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check`,
  accepting results only with valid sensor freshness and a clean manifest.
  Inspect route-lookahead activation, boundary-gate evidence, waypoint order,
  speed recovery, altitude, clearance, strict backup, connector/PVAJ
  continuity, and p50/p95/p99 planning latency.

### 2026-08-28 - Reject insufficient omnidirectional map support

- **Owner:** Navigation world-model and planner-configuration maintainers.
  **Scope:** Validate the axis-aligned ENU evidence map against the complete
  configured visibility horizon in both horizontal directions. Add a pure
  ray-to-AABB support calculation for future route-oriented planning queries;
  do not rotate, resample, or reinterpret ROG-Map voxel storage.
- **Safety impact:** `SAFETY_INVARIANT` preservation. The previous
  `maxCoeff(X,Y)` validation allowed the 110 m X side to hide a 15 m Y side,
  so a planner could start with a nominally valid horizon that was not
  available for a Y or diagonal route. The new check fails closed unless the
  shortest horizontal support covers `visibility_horizon_m` plus one evidence
  cell. UNKNOWN, OUT_OF_MAP, collision, backup, and dynamic policies are
  unchanged.
- **Derivation and cost:** Support is the first finite intersection of a
  normalized ENU direction ray with the local AABB. Configuration validation
  checks +X and +Y from the map center, which is the conservative minimum for
  all horizontal directions of an axis-aligned rectangle. The product map Y
  extent changes from 15 m to 30 m to satisfy the existing 14 m minimum
  visibility horizon without expanding the map to an unbenchmarked 90 m
  square. The future oriented planning-window phase must still measure the
  path-horizon benefit and map/snapshot cost.
- **Evidence:** Added unit coverage for axis, diagonal, zero-direction, and
  out-of-map support, plus a regression test proving that 110 x 15 m is
  rejected. Existing 110 x 15 fixtures were updated to 110 x 30 m. Full
  runtime/SITL acceptance remains open until the new geometry and latency
  distributions are measured.
- **Removal/review condition:** Revisit only after a route-oriented query
  contract and a measured sparse/chunked or sufficiently wide evidence-map
  design can provide the same safety support with lower memory and slide/export
  tails. Do not restore `maxCoeff` or lower the visibility horizon to make a
  narrow map pass.
- **Verification:** `make build`; source `install/setup.bash` and run the
  planner-config, world-model, trajectory, runtime, mission, and PX4 contract
  tests. Then repeat the 0/45/90 degree route matrix and inspect mapping
  update, snapshot export, planning, command age, speed, altitude, clearance,
  backup, and waypoint evidence at p50/p95/p99.

### 2026-08-28 - Move anisotropic-map support enforcement to route runtime

- **Owner:** Navigation planner route-query and world-model maintainers.
  **Scope:** Supersede the product-footprint part of the preceding entry.
  Keep the measured `110 x 15 x 6 m` ENU baseline instead of making an
  unbenchmarked `110 x 30` product-map change. Use the shared ray-to-AABB
  support helper from the measured start toward the requested route in the
  planner path-search boundary; reject unsupported route progress explicitly.
- **Safety impact:** `SAFETY_INVARIANT` preservation. The planner no longer
  treats the longest horizontal side as evidence for every route direction,
  but short valid goals are not rejected merely because the static AABB is
  anisotropic. A Y/diagonal request that cannot provide the required route or
  visibility support fails closed before A* rather than being silently clipped.
  UNKNOWN, OUT_OF_MAP, collision, backup, dynamic, and freshness gates are
  unchanged.
- **Derivation and cost:** Runtime support is the first normalized route ray
  intersection with the current AABB. The required distance is bounded by the
  requested route distance and the configured visibility horizon, with the A*
  inward-cell margin. This adds one constant-time geometry calculation per
  path-search request and avoids the measured memory/snapshot/slide regression
  from doubling the Y footprint.
- **Evidence:** The `110 x 30` experiment artifact
  `.artifacts/runtime/external-mode-check-20260828T032119-636685` had valid
  sensor freshness but regressed to waypoint coverage `[0,1,2,3]` and speed
  p95 `2.326 m/s`, versus the prior `110 x 15` run's `[0,1,2,3,4,5]` and
  `2.796 m/s`. The experiment is not acceptance evidence; it is the reason
  the footprint expansion is superseded.
- **Removal/review condition:** Revisit when a wider or sparse/chunked map
  has repeated distribution evidence showing lower or equal mapping,
  snapshot, slide, and planning tails while preserving strict safety and
  multi-direction route completion. Do not restore max-axis validation or
  allow endpoint clipping to stand in for route support.
- **Verification:** `make build`; run planner/world-model/mapping contract
  tests, then repeat straight, Y, diagonal, yaw-versus-velocity, and 0/45/90
  degree route scenarios. Inspect support/clipping reason, waypoint order,
  speed, altitude, backup, clearance, and p50/p95/p99 latency.

### 2026-08-28 - Restore lateral support for the measured three-column route

- **Owner:** Navigation runtime map-geometry and planner route-query
  maintainers.
- **Scope:** Change the product ROG-Map sliding evidence window from an
  axis-aligned `110 x 15 x 6 m` ENU footprint to `110 x 30 x 6 m`. Keep voxel
  storage axis-aligned and keep route-direction support, UNKNOWN,
  OUT_OF_MAP, corridor, swept-world, dynamic, freshness, backup, lease, and
  command-identity gates unchanged. This supersedes the preceding decision's
  choice to retain `110 x 15` after the new exact-head route evidence exposed
  an unsupported lateral waypoint.
- **Safety impact:** `SAFETY_INVARIANT` preservation with an explicit
  performance risk. The change supplies physical map support for lateral
  route progress; it does not rotate evidence by yaw, clip an endpoint, or
  relax a gate. The doubled Y voxel population may increase memory,
  snapshot-export, callback, and planner tails; any resulting freshness or
  deadline failure remains fail-closed.
- **Derivation and evidence:** In
  `.artifacts/runtime/external-mode-check-20260828T053849-764780`, after
  waypoint `(50, 5, 3)` was accepted, the next target `(50, -5, 3)` was
  reported as `target_grid=2` (`OUT_OF_MAP`) while the measured vehicle was
  near `y=+5`. With the centered 15 m Y window, the half-extent was 7.5 m,
  so the 10 m lateral leg could not be represented with the required support
  margin. A 30 m Y extent gives a 15 m half-extent while retaining the
  axis-aligned storage contract; it is a candidate geometry correction, not
  acceptance evidence.
- **Removal/review condition:** Revisit if repeated representative runs show
  higher mapping/snapshot p95 or p99, memory exhaustion, stale-world rejects,
  command exhaustion, worse speed/altitude/clearance, or no improvement in
  lateral waypoint completion. Replace with a measured sparse/chunked or
  route-oriented support design only after equivalent fail-closed evidence is
  demonstrated; do not shrink the map or lower visibility/acceptance gates to
  hide an unsupported route.
- **Verification:** `make build`; source the clean install and run planner,
  mapping, runtime, mission, and PX4 contract tests. Repeat the structured
  three-column route and the 0/45/90 degree matrix, comparing map memory,
  mapping update/callback/export p50/p95/p99, planner/EXP tails, target grid
  states, waypoint order, speed recovery, altitude, clearance, backup,
  command age, and fail-closed outcome. Run dataset shadow planning only
  after the runtime geometry comparison is complete.

### 2026-08-28 - Stabilize free-yaw continuity and PX4 hold heading

- **Owner:** Navigation trajectory-yaw and PX4 external-mode maintainers.
- **Scope:** Derive free yaw only from finite horizontal motion, retain the
  incoming heading for stationary or pure-vertical trajectory tails, and make
  yaw normalization finite and bounded. A hot replan now compares measured yaw
  with the committed yaw anchor and rebases when the configured
  `yaw_tracking_error_budget_rad` is exceeded. PX4 stationary, position-hold,
  and planner-recovery setpoints explicitly carry the latest valid odometry
  heading and zero yaw-rate.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This closes arbitrary
  `atan2(0,0)` heading selection, non-finite normalization, and stale committed
  yaw reuse. It does not relax waypoint, obstacle, UNKNOWN/OUT_OF_MAP, backup,
  dynamic, freshness, command-anchor, lease, or deadline gates. If the measured
  heading is unavailable, the setpoint omits yaw and the existing freshness
  contract remains authoritative.
- **Derivation and cost:** The horizontal direction threshold is the existing
  free-yaw/waypoint direction scale; vertical displacement no longer counts as
  heading evidence. The initial `0.35 rad` yaw rebase value is a continuity
  recovery envelope only, bounded below pi and deliberately not a certification
  threshold. Rebase reuses the existing measured-state connector and adds no
  optimizer pass. Hold setpoints add only quaternion normalization and one yaw
  calculation per 50 Hz setpoint.
- **Evidence:** The reviewed external-mode artifact showed cumulative yaw
  changes while the vehicle was moving through replans, while contiguous
  near-zero-speed intervals were mostly stable; it did not directly prove a
  stationary >180-degree spin because raw command yaw was not recorded. The
  artifact nevertheless exposed the missing yaw fields in stationary/hold
  publication and the planner's zero-horizontal terminal-direction risk.
  Focused pure yaw tests and rebuilt contract tests are required; SITL
  acceptance remains open until command yaw, measured yaw, yaw-rate, rebase
  count, and stationary intervals are logged together.
- **Removal/review condition:** Revisit if repeated yaw-versus-velocity cases
  show heading chatter, excessive yaw-rate, slower route progress, altitude or
  clearance regression, or increased solve tails. Do not increase the yaw
  envelope or suppress yaw response to make a single run pass; calibrate from a
  distribution of measured tracking error and route curvature.
- **Verification:** `make build`; run planner trajectory/config tests and PX4
  navigation contract tests; then repeat the 0/45/90-degree and
  yaw-different-from-velocity scenarios while inspecting yaw continuity,
  yaw-rate, position/velocity/altitude, waypoint order, handover reason,
  freshness, and p50/p95/p99 planner latency.

### 2026-08-28 - Restore canonical planner total in structured diagnostics

- **Owner:** Navigation runtime observability and report maintainers.
- **Scope:** Publish the measured wall-clock duration of each planner solve as
  `planning_total_us` in the structured planner decision trace. Keep
  `planning_latency_ms` for the rolling trace and keep stage timings
  (`exp_frontend_us`, `exp_opt_us`, `backup_frontend_us`, `backup_opt_us`)
  separate.
- **Safety impact:** `OBSERVABILITY_ONLY`. No planner behavior, deadline,
  command, fallback, or acceptance decision changes. Missing or invalid timing
  remains unavailable; it must not be converted to a zero or a safety pass.
- **Derivation and cost:** The value is `last_planner_us_`, measured around the
  complete `PlannerFacade` solve. The report must use this canonical total and
  must not sum stages, because stages can be skipped or overlap with other
  runtime work. One diagnostic key/value is added per solve.
- **Evidence:** Existing artifacts contained `planner_us` in runtime cycle
  metrics and stage values in the decision trace, but
  `planning_total_us` distributions were empty because the producer emitted
  only the millisecond rolling-trace field. The report parser and contract
  tests already named `planning_total_us`; the producer/parser mismatch is now
  covered by a trace fixture and percentile assertion.
- **Removal/review condition:** Keep this field while the report's p50/p95/p99
  planner total is used for budget decisions. Revisit only if the runtime
  timing clock or trace schema changes; never infer total latency by adding
  stage values or treat an empty distribution as zero.
- **Verification:** `make build`; run planner/PX4 CTest and
  `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -q`;
  inspect a fresh runtime artifact and require non-empty
  `planning_total_us` with units in microseconds and matching
  `planning_latency_ms * 1000` within measurement rounding.

### 2026-08-28 - Reuse disjoint world certificates during mapping publication

- **Owner:** Navigation world-snapshot, planner certificate, and runtime
  mapping maintainers.
- **Scope:** Store the swept candidate protected AABB in the immutable command
  certificate. During a mapping publication, skip a repeated full trajectory
  resweep only when the new immutable snapshot's complete change provenance
  proves all revisions after that certificate are disjoint from the protected
  region and the command lease has not expired. Publish counters for the
  provenance fast path and full revalidation path.
- **Safety impact:** `SAFETY_INVARIANT` preservation. The fast path is the
  existing `commitIfCurrentOrUnaffected` proof moved to the command-retention
  boundary; it does not trust a changed map, skip an intersecting region, or
  relax UNKNOWN/OUT_OF_MAP, backup, swept, dynamic, freshness, epoch, lease,
  or command-anchor gates. Missing, malformed, truncated, or intersecting
  history falls back to full validation, which remains fail-closed.
- **Derivation and cost:** The protected region is produced by the existing
  swept validator, including curve and voxel shells. The fast path performs an
  expiry check and immutable history query instead of repeatedly sampling the
  whole trajectory at mapping frequency. One certificate AABB is retained per
  command; counters add no map copy.
- **Evidence:** The valid external-mode traces showed mapping callback time
  materially above the observation period while the callback performed map
  update, snapshot publication, and command recertification. The store already
  had a disjoint-change authorizer, but the mapping retention path always ran
  `validateCommittedTrajectory`, so the proof was not reused there. Focused
  planner/runtime tests and a fresh artifact must verify non-zero fast-path
  counts, zero fast-path use for intersecting changes, and lower mapping
  callback p95 without loss of command/world identity ordering.
- **Removal/review condition:** Revisit if fast-path counts are zero, history
  queries dominate, disjoint updates are incorrectly rejected, or any repeated
  run shows stale/unsafe command retention, waypoint, speed, altitude,
  clearance, or handover regression. Never replace the provenance query with a
  timer, revision-only shortcut, or unconditional snapshot retention.
- **Verification:** `make build`; run mapping, world-snapshot, planner,
  runtime, and PX4 contract tests; then repeat the 3-column scenario and
  compare mapping callback/export/revalidation p50/p95/p99, world identity,
  command generation, freshness, strict backup, and mission outcome.

### 2026-08-28 - Bound immutable snapshot export cadence

- **Owner:** Navigation mapping actor and runtime world-snapshot publication
  maintainers.
- **Scope:** Keep every admitted mutable ROG update serialized and revisioned,
  but accumulate its conservative changed-region union and export only the
  newest immutable snapshot at `navigation_runtime.mapping_snapshot_publication_period_s`.
  The canonical profile uses 50 ms, no slower than the planner period and no
  slower than the world-freshness budget. New localization epochs and whole-map
  changes force immediate full export; ordinary coalesced updates use one
  bounded patch when its provenance and geometry remain valid.
- **Safety impact:** This changes publication latency, not map admission,
  occupancy semantics, UNKNOWN/OUT_OF_MAP policy, or certificate gates. The
  execution store continues to authorize only the last immutable snapshot and
  rejects stale snapshots through the existing freshness gate. Revision history
  retains every advancing update, and a patch spanning multiple revisions is
  accepted only with a newer identity and complete contiguous history. Missing
  history, malformed regions, epoch changes, patch-depth overflow, or export
  failure fall back to the existing full export/fail-stop path.
- **Evidence:** Prior traces showed mapping callback p95 around 42--72 ms and
  snapshot export p95 around 28--52 ms while the input stream was faster than
  the callback. The previous actor exported a patch or full grid for every
  advancing update. The new telemetry separates map-update completion from
  `world_snapshot_published_count` and `world_snapshot_deferred_count`; report
  timing excludes deferred zero exports.
- **Removal/review condition:** Revisit if published snapshot age exceeds the
  configured period, the freshness gate rejects due to publication delay,
  world-revision/history conservation fails, live snapshot memory grows,
  patch classification differs from an immediate full export, or any repeated
  scenario shows a clearance, waypoint, speed, altitude, handover, or mode
  regression. Do not turn this into timer-only retention or drop revision
  history.
- **Verification:** `make build`; run mapping, world-snapshot, runtime, planner,
  and PX4 contract tests; run the mapping actor coalescing test; execute a fresh
  3-column trace and compare callback/export p50/p95/p99, published/deferred
  counts, world age, identity ordering, memory, and mission outcome.

### 2026-08-28 - Distinguish certified pass-through look-ahead completeness

- **Owner:** Navigation planner route-window and runtime diagnostics maintainers.
- **Scope:** Record the route-window length required by the stopping/replan
  envelope separately from the prefix actually traversable and certified by
  the current world snapshot. A non-empty but short prefix remains eligible
  only as a bounded frontier; it is never labelled complete. Its terminal
  speed seed is capped by the certified distance and duration.
- **Safety impact:** `SAFETY_INVARIANT` preservation. No UNKNOWN/OUT_OF_MAP,
  dynamic, corridor, flatness, swept-world, backup, freshness, or command
  lease gate is relaxed. The completeness bit and speed cap are planner
  observability/seed constraints; the final polynomial and immutable world
  certificate remain authoritative. Invalid or missing measurements leave the
  completion state false and preserve the existing fail-closed path.
- **Evidence:** The exact `90d9148` audit found that any non-empty outgoing
  prefix was marked active even when shorter than the computed stopping and
  replan requirement, which can cause terminal-speed loss and repeated hot
  replans. The new fields are `required_lookahead_m`,
  `certified_lookahead_m`, and `lookahead_complete`; runtime traces expose all
  three for distribution-level review.
- **Removal/review condition:** Revisit if a repeated scenario shows terminal
  speed above the certified-distance bound, a completed prefix shorter than
  required, increased command exhaustion, or regression in waypoint order,
  clearance, altitude, handover, or fail-closed behavior. Do not replace the
  completeness check with a boolean-only bypass or a larger timeout.
- **Verification:** `make build`; run planner, runtime, and PX4 contract tests;
  run the planner trace tests; then inspect a fresh 3-column trace and compare
  required/certified look-ahead, terminal speed, planner latency, command gaps,
  waypoint outcome, and safety decisions.

### 2026-08-28 - Bound partial directional pass-through frontiers

- **Owner:** Navigation planner route-query and A* maintainers.
- **Scope:** Permit only the pass-through outgoing query to receive a bounded
  map-frontier result when the current anisotropic ENU AABB cannot provide the
  full route window. Ordinary active-goal searches still fail closed. The
  caller records the prefix as incomplete and applies the certified-distance
  terminal-speed cap; A* keeps its endpoint inside a two-cell margin derived
  from the active grid resolution instead of a fixed 2.5 m offset.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This does not make
  UNKNOWN or OUT_OF_MAP traversable and does not turn a clipped endpoint into
  a mission-goal acceptance. The partial prefix remains subject to continuous
  corridor, dynamic, flatness, swept-world, backup, freshness, and command
  identity gates. If the frontier has no useful positive support, the query
  fails closed. The ROG grid remains fixed ENU and is not rotated or resampled.
- **Evidence:** The exact `90d9148` audit identified 110 x 15 x 6 m geometry
  and the A* 2.5 m endpoint projection as causes of short lateral prefixes.
  Existing route-support validation correctly rejects unsupported ordinary
  goals, but pass-through needs an explicit incomplete-frontier path so F01's
  certified-length speed bound can operate without hiding the clipping.
- **Removal/review condition:** Revisit if partial-frontier runs produce a
  complete flag with insufficient certified length, terminal speed above its
  bound, endpoint outside the AABB margin, or any clearance, waypoint,
  altitude, handover, command-exhaustion, or fail-closed regression. Do not
  broaden the opt-in to ordinary goals or replace the support check with a
  larger map-margin constant.
- **Verification:** `make build`; run planner/world-model/mapping, runtime,
  mission, and PX4 contract tests; then repeat Y, diagonal, 0/45/90-degree,
  and yaw-versus-velocity route cases and inspect required/certified look-ahead,
  support, clipping, terminal speed, world identity, latency, and outcome.

### 2026-08-28 - Bound L-BFGS history for planner tail latency

- **Owner:** Navigation nominal/backup trajectory optimization maintainers.
- **Scope:** Replace the previous fixed 256-entry L-BFGS history with the
  explicit per-profile `lbfgs_memory_size` parameter, set to 32 in the
  canonical runtime profile. The same bound applies to nominal and backup
  solves and is validated in the inclusive range [3, 256].
- **Safety impact:** `SAFETY_INVARIANT` preservation. This changes only the
  numerical search workspace and may cause an optimizer to terminate or fail
  earlier; it cannot widen V/A/J, corridor, flatness, swept-world, UNKNOWN,
  backup, freshness, lease, or command-identity gates. Every resulting
  candidate still passes the existing independent certificates or is rejected.
- **Evidence:** Runtime artifact
  `.artifacts/runtime/external-mode-check-20260828T040104-673978` measured
  EXP optimization p95 137543 us, p99 170952 us and max 175900 us while
  nominal solves used the unbounded/default iteration count and 256 history
  entries. The history reduction targets per-iteration matrix work without
  changing the solve deadline; the next comparison must measure convergence,
  hard-gate rejection, and tail latency together.
- **Removal/review condition:** Revisit if repeated representative runs show
  worse corridor/dynamic/flatness/world acceptance, more command exhaustion,
  waypoint or altitude regressions, or no p95/p99 improvement. Do not raise
  the solve deadline or reduce a hard certificate to compensate for optimizer
  quality loss.
- **Verification:** `make build`; run planner, runtime, PX4, and optimizer
  contract tests; then repeat the same 3-column workload and compare L-BFGS
  iterations/return codes, EXP p50/p95/p99/max, total planning latency,
  certificate outcomes, command gaps, waypoint completion, altitude,
  clearance, and fail-closed decisions.

### 2026-08-28 - Align planner cadence with its absolute solve budget

- **Owner:** Navigation runtime scheduler and planner-backend maintainers.
- **Scope:** Set the canonical runtime planner cadence to 5 Hz while keeping
  command sampling at 50 Hz. At node construction, reject any configuration
  whose backend `solve_deadline_s` is non-finite, non-positive, or greater
  than or equal to one planner period. The planner callback remains
  mutually exclusive; this check prevents a nominal timer cadence that the
  callback cannot physically service.
- **Safety impact:** `PERFORMANCE_POLICY` and fail-closed startup validation.
  No solve deadline, dynamic limit, map policy, command lease, waypoint
  acceptance rule, fallback, or certificate is relaxed. A mismatched profile
  is rejected before the mapping worker starts instead of silently running an
  overloaded scheduler. The 50 Hz command sampler remains independent and
  continues to publish only the immutable validated command bundle.
- **Evidence:** The exact `90d9148` audit identified the canonical 10 Hz
  planner period (100 ms) as shorter than the 180 ms solve budget. The fresh
  post-L-BFGS artifact
  `.artifacts/runtime/external-mode-check-20260828T043244-698169` still
  reached a 180697 us total solve and stopped fail-closed before cruise,
  so increasing the deadline or retaining a 10 Hz nominal schedule would
  hide the measured tail rather than resolve the cadence mismatch.
- **Removal/review condition:** Revisit only after repeated representative
  SITL and recorded-data distributions demonstrate a bounded planner p99
  with explicit scheduling margin at a higher rate. Do not remove the guard,
  make the timer re-entrant, or compensate by increasing the solve deadline.
- **Verification:** `make build`; run runtime/planner/PX4 contract tests;
  verify that the canonical profile is 5 Hz and that a 10 Hz override with
  the 180 ms backend budget is rejected; then inspect planner total latency,
  callback gaps, command age, waypoint order, altitude, clearance, and
  fail-closed behavior in a fresh 3-column run.

### 2026-08-28 - Certify yaw rate on the composed command bundle

- **Owner:** Navigation planner command composition and execution-boundary
  maintainers.
- **Scope:** Before any candidate enters the immutable-world authorization
  gate, evaluate the maximum yaw rate of the composed yaw trajectory. This
  covers inherited prefix, newly optimized MAIN, BACKUP suffix, splice
  boundaries, and measured-state emergency-brake candidates. The optimizer's
  local yaw check remains in place; this is the final bundle-level check.
- **Safety impact:** `SAFETY_INVARIANT` strengthening. A non-finite or
  over-limit composed yaw trajectory is rejected and cannot be staged,
  world-certified, or sampled by the command timer. No yaw-rate limit is
  increased and no trajectory is clamped after certification. Position,
  V/A/J, flatness, corridor, swept-world, UNKNOWN/OUT_OF_MAP, freshness,
  backup, lease, and command-identity gates remain unchanged.
- **Evidence:** The exact 5 Hz artifact
  `.artifacts/runtime/external-mode-check-20260828T044208-708993` recorded a
  yaw-rate peak of 4.699 rad/s while the planner configuration limit was
  3.0 rad/s. `Planner::sampleCommand()` previously checked only finiteness;
  local `YawTrajOpt` validation did not protect a composed trajectory after
  concatenation/inheritance. The final certificate closes that boundary.
- **Removal/review condition:** Never remove the check while the 3.0 rad/s
  command contract is active. Revisit only if the vehicle contract changes
  with a separately owned dynamic/body-rate certificate and repeated evidence.
  Any increase in candidate rejection, safety hold, yaw discontinuity,
  waypoint loss, altitude loss, or command exhaustion must be traced to the
  composing segment and fixed there; do not bypass this certificate.
- **Verification:** `make build`; run the planner trajectory/config tests,
  runtime and PX4 contract tests; assert that composed MAIN+BACKUP and
  emergency yaw trajectories over the limit are rejected while finite
  in-limit trajectories pass; then repeat yaw-versus-velocity and 3-column
  traces and report maximum yaw rate, yaw continuity, role, mode, waypoint,
  altitude, clearance, and fail-closed outcome.

### 2026-08-28 - Expose EXP L-BFGS evaluation counts

- **Owner:** Navigation trajectory-optimization diagnostics maintainers.
- **Scope:** Publish total, first-attempt, and last-attempt L-BFGS objective
  evaluation counts for each EXP solve. The count is collected from the
  existing diagnostic `costFunctional` invocation counter and is propagated
  through the planner facade, runtime trace, and report parser. No solver
  parameter, stopping rule, objective, certificate, or fallback behavior is
  changed.
- **Safety impact:** `OBSERVABILITY_ONLY`. The fields are diagnostic-only and
  never participate in candidate selection, hard gates, cancellation,
  freshness, world authorization, or command publication. Missing or legacy
  fields remain absent/partial in the parser rather than being inferred.
- **Evidence:** The canonical trace
  `.artifacts/runtime/external-mode-check-20260828T044208-708993` exposes
  attempt and return-code counts but not the number of objective evaluations
  behind each 180 ms tail. Since L-BFGS returns `LBFGSERR_MAXIMUMITERATION`
  at -1008 and `LBFGSERR_MAXIMUMLINESEARCH` at -1009, evaluation counts are
  required to separate iteration-bound from line-search-heavy cases before
  tuning either parameter.
- **Removal/review condition:** Revisit if counters disagree with the
  objective invocation count, reset across retries incorrectly, or alter
  timing enough to invalidate comparisons. Do not use the counters as a
  substitute for certificates or silently convert an observation into a
  solver limit.
- **Verification:** `make build`; run optimizer/planner/runtime/PX4 tests and
  planner trace parser tests; verify total count equals the sum of recorded
  attempts, retry resets are accounted for, return-code interpretation is
  preserved, and fresh traces include the three evaluation fields.

### 2026-08-28 - Align immutable mapping snapshot publication with planner cadence

- **Owner:** Navigation mapping actor and runtime world-snapshot publication
  maintainers.
- **Scope:** Set the canonical runtime snapshot publication period to 200 ms
  while retaining per-observation mutable ROG-map updates and the existing
  bounded changed-region/history coalescing. The 200 ms period matches the
  200 ms planner period and remains below the 500 ms world-freshness window; it does
  change the actor API default used by isolated mapping tests.
- **Safety impact:** `PERFORMANCE_POLICY` with fail-closed freshness preserved.
  No occupancy, UNKNOWN/OUT_OF_MAP, route-support, swept-world, dynamic,
  backup, lease, or command-identity gate is relaxed. The last published
  immutable snapshot remains the only execution input; the existing freshness
  check rejects an aged snapshot. New epochs, whole-map changes, missing
  history, malformed regions, patch-depth overflow, and export failures retain
  the existing immediate/full-export or fail-stop behavior.
- **Evidence:** The canonical 5 Hz trace
  `.artifacts/runtime/external-mode-check-20260828T044208-708993` received
  mapping observations at about 18 Hz and published 509/509 snapshots with
  snapshot-export p95 27.4 ms and callback p95 43.6 ms. A 50 ms period
  therefore did not coalesce this workload. The fresh post-change trace
  `.artifacts/runtime/external-mode-check-20260828T050836-727416` measured an
  effective sensor cadence of about 10 Hz (p50 inter-observation period 100 ms),
  and also published 466/466 snapshots at 100 ms. The 200 ms runtime period is
  therefore the smallest canonical period expected to coalesce that workload
  while preserving one publication per planner period under the declared
  freshness contract.
- **Removal/review condition:** Revisit if published snapshot age exceeds the
  configured period, freshness rejects increase, world revision/history
  conservation fails, patch classification differs from an immediate full
  export, live snapshot memory grows, or any repeated run shows clearance,
  waypoint, speed, altitude, handover, or mode regression. Revert the period
  if the measured observation cadence no longer leaves a safe publication
  margin; do not drop map revisions or make retention timer-only.
- **Verification:** `make build`; run mapping/world-snapshot/runtime/planner
  and PX4 contract tests plus the mapping actor coalescing test; then repeat
  the same 3-column trace and compare callback/export p50/p95/p99,
  published/deferred counts, snapshot age, world identity, memory, command
  recertification, and mission outcome.

### 2026-08-28 - Reserve hard route-boundary cells for genuine turns

- **Owner:** Navigation planning backend / pass-through route-window policy.
- **Scope:** A pass-through look-ahead on a straight or shallow bend keeps the
  waypoint in the continuous guide but does not insert an exact route-boundary
  corridor cell. The hard route-boundary contract remains enabled for a
  genuine heading change, where corridor simplification could otherwise cut
  the turn. This is a policy distinction, not a waypoint or corridor bypass.
- **Safety impact:** No acceptance radius, unknown-space policy, dynamic limit,
  flatness gate, yaw gate, or swept-world certificate changes. Every exposed
  candidate still requires continuous corridor and world authorization; the
  mission controller still owns measured waypoint acceptance.
- **Evidence:** The reverted junction-unpin experiment
  `.artifacts/runtime/external-mode-check-20260828T053330-757562` showed that
  the initial shallow pass-through solve failed three times with EXP final
  normalized dynamic violation about 1.54-1.55 and no executable trajectory.
  The preceding pinned run also spent about 177.6 ms in EXP and advanced only
  three waypoints, motivating removal of unnecessary hard junctions without
  weakening the genuine-turn contract.
- **Removal/review condition:** Revert if a straight/shallow route skips a
  mission acceptance event, violates route order, or increases continuous
  corridor/world/dynamic/yaw failures in repeated 3-column runs. Revisit the
  genuine-turn geometry separately if sharp corners still stop or jitter.
- **Verification:** Rebuild with a clean manifest; run planner/backend/runtime
  tests and repeat `MAP_PROFILE=long_three_pillars_multiwaypoint
  make external-mode-check`, checking waypoint coverage, route order, speed
  recovery, junction distances, and all hard certificates.

### 2026-08-28 - Make sharp pass-through corners measured handoffs

- **Owner:** Navigation planning backend / pass-through trajectory maintainers.
- **Scope:** When the outgoing leg forms a genuine heading change, the current
  MINCO solve terminates at the active mission waypoint with an incoming
  tangent terminal-speed cap. It does not extend the same solve through the
  outgoing leg or use the acceptance-ball fillet endpoint. The following solve,
  initialized from measured state after waypoint acceptance, owns the outgoing
  leg. Straight and shallow pass-through legs retain the certified look-ahead
  continuity path.
- **Safety impact:** `SAFETY_INVARIANT` and route-progress correctness. This
  removes an overconstrained cross-boundary P/V request that was repeatedly
  committing a safe backup suffix beyond the still-unaccepted waypoint. It
  does not relax the acceptance radius, UNKNOWN/OUT_OF_MAP policy, corridor,
  swept-world, V/A/J, yaw, backup, freshness, lease, or immutable-world
  authorization gates. The transition may reduce speed at a sharp corner by
  design; the next measured handoff is responsible for speed recovery.
- **Evidence:** The clean-manifest artifact
  `.artifacts/runtime/external-mode-check-20260828T055010-782186` passed the
  widened map boundary through waypoint 3, then at waypoint 4 repeatedly
  failed the corner solve while the route look-ahead target moved to
  `y=0.38..0.97`. A committed bundle ended at `(84.99,0.83,3.00)` while the
  active waypoint remained `(85,-5,3)` (`endpoint_error=5.825`), followed by
  repeated EXP hard-gate/yaw and KNOWN_FREE backup failures and a safety stop.
- **Removal/review condition:** Revisit only after repeated representative
  three-column runs show that direct corner handoffs cause waypoint skips,
  corridor/world/dynamic/yaw failures, altitude loss, or unacceptable speed
  recovery. Do not restore cross-corner look-ahead merely to hide an EXP or
  backup rejection, enlarge the acceptance radius, or weaken a certificate.
- **Verification:** `make build`; run the planner/backend/runtime/PX4 contract
  tests; then repeat
  `MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check` and
  compare waypoint order/coverage, measured speed recovery, corner terminal
  speed, altitude, clearance, planner p50/p95/p99/max, EXP/backup reject
  stages, command continuity, and fail-closed mode outcome.

### 2026-08-28 - Accept recent measured pass-through crossings

- **Owner:** PX4 External Mode mission-controller and mission-contract
  maintainers.
- **Scope:** For a `PASS_THROUGH` waypoint, retain direct point acceptance and
  additionally accept a recent, forward route-ordered segment between two
  valid measured odometry samples when that segment intersects the configured
  acceptance ball. Crossing inference is limited to the 250 ms mission-update
  sample window; stop waypoints retain point-in-ball plus low-speed/hold
  confirmation.
- **Safety impact:** `MISSION_PROGRESS_INVARIANT` preservation. The planner
  cannot advance a checkpoint from look-ahead or command endpoint; both segment
  endpoints are measured, route projection must move forward across the active
  waypoint arc, and the segment-to-waypoint distance must remain within the
  configured acceptance radius. A stale or non-route-ordered gap is rejected.
  No acceptance radius, speed limit, corridor, UNKNOWN/OUT_OF_MAP, swept-world,
  freshness, backup, yaw, or PX4 mode gate is relaxed.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T060359-794884` sampled the
  active waypoint gate at 4.464 m/s with position error 2.037 m, then at the
  next throttled observation error was 2.652 m; the waypoint was never observed
  inside its 0.9 m ball and the mission stopped at waypoint 1. The node's
  mission timer is 50 ms, so recent segment intersection preserves a crossing
  that point sampling can miss at cruise speed.
- **Removal/review condition:** Revisit if repeated traces show acceptance on
  reverse motion, stale odometry gaps, lateral near-misses, wrong route order,
  duplicate request IDs, or any stop-waypoint regression. Do not compensate by
  enlarging acceptance radius or lowering the pass-through speed contract.
- **Verification:** `make build`; run mission/planner/runtime/PX4 tests,
  including direct and stale-gap crossing cases; then repeat
  `MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check` and
  compare waypoint order/coverage, acceptance errors/speeds, request/goal
  transitions, altitude, clearance, command continuity, and planner/mapping
  latency distributions.

### 2026-08-28 - Bound remote mission goals to a certified receding prefix

- **Owner:** Navigation planner route-search and trajectory-seed maintainers.
- **Scope:** When an A* route to the active mission waypoint exceeds the
  configured effective visibility horizon, retain the original waypoint as
  the mission-controller target but pass only a bounded, interpolated A* prefix
  to MINCO. The prefix is rechecked on the inflated map and the next planning
  cycle repeats from fresh measured state. Goals within the horizon retain the
  existing exact endpoint and pass-through/corner behavior.
- **Safety impact:** `SAFETY_INVARIANT` preservation. This prevents a remote
  endpoint beyond the current finite planning evidence from being coupled to
  one executable polynomial. It does not authorize UNKNOWN, clip a command
  past a waypoint, change acceptance radius, or alter V/A/J, corridor,
  swept-world, backup, freshness, lease, or PX4 mode gates. A malformed or
  uncertifiable prefix is rejected fail-closed.
- **Derivation and cost:** The prefix bound is the minimum of the remaining
  configured planning horizon and the existing effective visibility horizon;
  no new threshold is introduced. The bounded interpolation is linear in the
  A* path length and adds one segment traversability check before the existing
  guide allocation/corridor/optimizer stages. This is a receding-horizon
  ownership correction, not a reduction of the mission route or map footprint.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T061332-806845` accepted
  waypoint 1 after the measured-crossing fix, then repeatedly attempted the
  remote waypoint `(50,5,3)` from about 30 m away. `target_grid=1` and
  `planning_total` remained finite, but `main_minco` failed with repeated
  corridor/dynamic violations while the committed endpoint stayed at the
  previous waypoint. Focused prefix tests and a fresh clean-manifest runtime
  trace are required; the current artifact remains BLOCKED evidence.
- **Removal/review condition:** Revert or redesign only if repeated
  representative runs show worse waypoint order/coverage, speed recovery,
  altitude, clearance, command continuity, planner tails, or more fail-closed
  stops. Replace with a measured route-window/piecewise planner only when it
  preserves the same remote-goal and immutable-world contracts. Never increase
  the bound or relax a certificate to hide optimizer failure.
- **Verification:** `make build`; source `install/setup.bash` and run planner,
  trajectory, mission, runtime, and PX4 contract tests; rebuild the clean
  manifest; then repeat
  `MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check` and
  inspect `planning_target` versus mission target, waypoint coverage, speed,
  altitude, clearance, command identity/PVA continuity, and p50/p95/p99
  planning/mapping latency.

### 2026-08-28 - Bound backup refinement to the certified braking seed

- **Owner:** Planner backup-suffix generation and trajectory-handoff maintainers.
- **Scope:** When `BackupTrajOpt` returns a numerically successful refinement,
  compare its normalized spatial trace against the already certified
  minimum-snap braking piece. If the deviation exceeds
  `max(0.5 m, 3 * planner resolution)`, discard the refinement and retain the
  certified seed.
- **Safety impact:** This is a continuity/stability guard, not a relaxed safety
  gate. The seed still passes the existing PVAJ, V/A/J, flatness, SFC,
  KNOWN_FREE swept, and complete-bundle authorization checks. A refinement that
  remains within the trace bound still passes every existing check. This avoids
  repeated hot replans replacing a safe stop suffix with a geometrically
  different path that can reverse lateral or vertical motion; it does not
  authorize UNKNOWN or change PX4 handover behavior.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T062835-818576` repeatedly
  replaced BACKUP candidates while the mission remained on waypoint 1; the
  committed command showed lateral/vertical oscillation and PX4 stopped at
  `0.759/0.750 m` lateral tracking error. The guard must be reviewed against
  repeated multi-waypoint runtime distributions, not a single run.
- **Removal/review condition:** Remove or retune only after a piecewise
  route/trajectory handoff owns backup continuity and repeated representative
  runtime evidence shows no backup-induced oscillation. Verification requires
  build, targeted planner tests, the full test suite, and
  `MAP_PROFILE=long_three_pillars_multiwaypoint make external-mode-check`; inspect
  backup refinement acceptance/rejection counts, tracking envelope, waypoint
  coverage, and mission outcome.

### 2026-08-28 - Resolve delayed terminal commands without an active waypoint

- **Owner:** PX4 External Mode command adapter and mission-controller lifecycle.
- **Scope:** Keep accepting only the exact final mission/waypoint/request identity
  after mission completion, but resolve diagnostic geometry by the command's
  bounded waypoint index instead of dereferencing the controller's active index.
  The active index is intentionally one-past-the-end in `Complete` state.
- **Safety impact:** This removes an exception path after a valid delayed
  `STATUS_COMPLETED` sample without broadening command identity, freshness,
  monotonic sample, estimator-health, tracking-envelope, or handover gates.
  Wrong terminal identities and duplicate samples remain fail-closed.
- **Evidence:** Unit coverage requires the exact delayed terminal identity to
  match, wrong waypoint/request identities to fail, a duplicate sample ID to
  fail the monotonic gate, and terminal waypoint lookup at `size()` to return no
  value while the completed waypoint remains addressable.
- **Removal/review condition:** Remove only when mission lifecycle and command
  ingestion become one versioned state machine that owns a terminal checkpoint
  object directly. Do not restore unchecked `activeWaypoint()` access after
  `MissionControllerState::Complete`.
- **Verification:** Build `px4_navigation_external_mode`, run
  `test_navigation_command` and `test_mission`, then run the package CTest set.

### 2026-08-28 - Preserve certified command during atomic waypoint handoff

- **Owner:** PX4 External Mode command slot and mission-goal handoff.
- **Scope:** Publishing the next waypoint no longer clears the currently
  certified `NavigationCommand`. A rejected, malformed, stale-identity, or
  world-regressing replacement also leaves that exact command unchanged. Only
  a fully accepted candidate replaces it atomically; activation boundaries,
  estimator-epoch changes, terminal failure, and mode handover explicitly
  invalidate the slot.
- **Safety impact:** The retained command keeps its original mission/request,
  world, epoch, timestamp, and validity lease; it is never relabelled by the PX4
  adapter. Existing estimator-health, odometry, command-freshness,
  `valid_until`, tracking-envelope, and terminal-status gates remain
  authoritative. This removes the asynchronous zero-velocity pulse caused by
  clearing the slot before the next planner command arrives, without allowing
  indefinite execution of an old trajectory.
- **Evidence:** Unit tests require retain to preserve every identity field,
  commit to replace old identity in one transition, and lifecycle invalidation
  to empty the slot. Runtime metrics expose
  `waypoint_handoffs_retaining_command`. Focused PX4 package tests must pass
  before commit; SITL must later show no stationary command inserted solely at
  a `PublishGoal` transition.
- **Removal/review condition:** Replace only with a shared versioned
  mission/command handoff state machine that provides equal or stronger
  immutable identity and bounded-lease guarantees. Never restore unconditional
  cache clearing on waypoint publication.
- **Verification:** Build `px4_navigation_external_mode`; run
  `test_navigation_command`, `test_mission`, and package CTest. During repeated
  multi-waypoint SITL, correlate accepted-waypoint status, goal publication,
  command identity, velocity setpoint, and the retained-handoff counter.

### 2026-08-28 - Centralize measured route boundary and reversal progress

- **Owner:** `navigation_mission::RouteProgress`; MissionController remains the
  sole waypoint-state transition owner.
- **Scope:** RouteProgress now evaluates recent measured segments crossing a
  waypoint acceptance ball with a finite sample-gap bound, forward incoming-leg
  motion, and lateral miss rejection. MissionController delegates that geometry
  instead of maintaining a second projection policy. Monotonic progress also
  resolves geometrically tied branches using prior arc progress, including a
  route that reverses 180 degrees over the same line.
- **Safety impact:** Planner endpoints still cannot advance mission state; only
  current/recent measured positions can satisfy the boundary. STOP/PASS_THROUGH,
  measured velocity, hold, and braking policy remain in MissionController. The
  change does not enlarge an acceptance radius, permit a stale sample gap, or
  relax route backtracking diagnostics.
- **Evidence:** Route contract tests cover duplicate waypoints, altitude
  interpolation, monotonic backtracking, forward segment jump, lateral miss,
  backward crossing, and branch selection after a 180-degree reversal. Existing
  PX4 mission tests cover stale crossing gaps, exact-once active-index advance,
  STOP speed/hold gates, pass-through, and terminal backup/main behavior.
- **Removal/review condition:** Replace only when one immutable route snapshot
  is consumed directly by mission, planner, yaw, and diagnostics with equivalent
  measured-boundary semantics. This checkpoint does not yet claim that complete
  M1 integration; planner/yaw route consumption remains open.
- **Verification:** Build through `px4_navigation_external_mode`, run
  `navigation_mission/test_mission_contract`, and run all PX4 External Mode
  package tests before commit.

### 2026-08-28 - Certify an immutable nominal seed before optimizer fallback

- **Owner:** Nominal MINCO optimizer and planner trajectory-certificate
  maintainers.
- **Scope:** Introduce an independent certificate for a complete, immutable
  pre-L-BFGS trajectory. Certification requires exact piece-to-corridor
  provenance, continuous half-space containment at the configured tolerance,
  initial/terminal PVAJ equality, C3 junction continuity, route-boundary gates,
  V/A/J limits, and the quadrotor flatness envelope.
- **Safety impact:** This checkpoint is certificate infrastructure only and does
  not yet alter runtime candidate selection. It deliberately does not rebuild a
  seed from optimizer-mutated variables, sample corridor membership, widen the
  corridor tolerance, or bypass later yaw, immutable-world, backup, and atomic
  command authorization. A future fallback may select only the bit-identical
  trajectory that passed this certificate before L-BFGS began.
- **Evidence:** Unit coverage accepts a two-piece C3 minimum-snap trajectory
  with exact corridor provenance, rejects swapped piece/corridor ownership, and
  rejects a continuous corridor excess of `0.010001 m` against the unchanged
  `0.01 m` tolerance. The focused backend build and test executable must pass
  before this infrastructure is committed.
- **Removal/review condition:** Replace only with an equal or stronger typed
  trajectory certificate that preserves immutable candidate identity and every
  listed hard gate. Remove if runtime integration cannot prove that the selected
  fallback is the exact pre-optimizer object.
- **Verification:** Build `navigation_planning_backend`; run
  `test_exp_optimizer_seed` and the package CTest set. Runtime integration must
  be a separate commit followed by repeated recorded-data and three-pillar SITL
  evidence.

### 2026-08-28 - Retain the certified pre-L-BFGS trajectory on corridor regression

- **Owner:** Nominal MINCO optimizer candidate selection.
- **Scope:** Freeze and certify the complete MINCO initialization immediately
  before the first L-BFGS call. If an otherwise successful optimized iterate is
  rejected by the independent continuous corridor gate, select a direct copy
  of that frozen object only when its precomputed certificate is valid.
- **Safety impact:** This is a bounded same-solve fallback, not permission to
  execute an uncertified guide. Cancellation still fails immediately; an
  invalid seed still fails closed. The selected seed already satisfies exact
  PVAJ/C3, piece/SFC provenance, route-boundary, V/A/J, flatness, and corridor
  contracts, and remains subject to downstream yaw/flatness, backup,
  latest-world, deadline/goal identity, complete-bundle, and atomic command
  authorization. The `0.01 m` corridor tolerance is unchanged.
- **Evidence:** The three-pillar artifact
  `.artifacts/runtime/external-mode-check-20260828T074424-843700` reached
  waypoint 4 and then failed three PlanFromRest attempts because L-BFGS moved
  the candidate to corridor violations of `0.016697`, `0.016714`, `0.051317`,
  and `0.018787 m`. Unit coverage proves candidate selection copies the exact
  seed coefficients/duration and rejects fallback when its certificate is
  invalid. Runtime logs report both rejected-candidate and certified-seed
  violations when the path is exercised.
- **Removal/review condition:** Remove when the optimizer itself guarantees
  hard-feasible iterates or a replacement planner provides an equal or stronger
  certified-candidate ownership model. Never replace the immutable copy with
  `rebuildInitialCandidate()` or any reconstruction after optimizer mutation.
- **Verification:** Build and test `navigation_planning_backend`, rebuild the
  clean release manifest, and repeat
`MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make external-mode-check`.

### 2026-08-28 - Replan unsettled terminal holds from measured state

- **Owner:** PX4 External Mode mission-to-runtime recovery boundary. **Scope:**
  a completed MAIN or BACKUP endpoint counts as a settled terminal hold only
  when both command and measured vehicle are inside the waypoint acceptance
  ball (BACKUP also retains its command-anchor requirement). Otherwise retain
  the exact certified endpoint hold, start the existing bounded recovery timer,
  and re-publish the same mission checkpoint with a new request identity for a
  measured-state connector. Only one such retry is allowed per active waypoint;
  a second unsettled completion reaches the unchanged bounded fail-closed
  handover instead of creating a same-goal retry loop.
- **Safety impact:** waypoint radius, acceptance speed, command-anchor limit,
  collision certificates, and recovery timeout are unchanged. The old behavior
  could suppress replanning indefinitely at an acceptance-edge backup endpoint;
  the replacement either commits a normally certified connector or performs
  the existing fail-closed PX4 Hold handover when the bounded window expires.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260828T100206-949033` first stopped
  at command error `0.883 m` inside a `0.9 m` radius while measured error stayed
  around `0.99-1.11 m`. No planning cycles occurred for roughly 53 seconds;
  eventual recovery was incidental to a later world change. The change adds
  one same-waypoint publication per completed-outside BACKUP and no periodic
  planner work while the hold remains settled.
- **Evidence:** MissionController unit tests require one monotonic request and
  no publication loop. External Mode/backend/runtime tests must preserve the
  bounded timeout and command identity rules. Repeated long-three-pillars SITL
  must show immediate connector planning, measured waypoint completion, and no
  gate relaxation.
- **Removal/review condition:** replace when planner/runtime expose an atomic
  terminal-settling state machine that owns BRAKE, HOLD, REJOIN and measured
  acceptance explicitly. Do not restore command-only acceptance-edge hold.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run PX4 External Mode, runtime, and backend tests,
  `make build`, then repeat the 3 m/s three-column mission and inspect the
  terminal request/commit timeline.
  If blocked, re-review planner, mapping, PX4, mission, and report artifacts at
  the first causal boundary; do not tune the corridor gate from one run.

### 2026-08-28 - Transport one immutable measured route into planner ownership

- **Owner:** MissionController route state, NavigationGoal transport, runtime
  admission, and planner route context.
- **Scope:** MissionController publishes the complete versioned mission route,
  active waypoint, and measured monotonic arc progress in every goal. Runtime
  reconstructs and validates route geometry, requires identity/frame/index and
  compatibility mirrors to agree, and gives that same snapshot to the planner.
  The planner derives pass-through continuation from the snapshot instead of
  independently trusting `next_target`.
- **Safety impact:** Missing, malformed, duplicate-ID, non-finite, regressed, or
  mismatched route data is rejected fail-closed and cannot retain or create an
  executable command. Waypoint acceptance remains measured-state-only in
  MissionController. This change does not enlarge waypoint radius, relax map or
  trajectory certificates, rotate map evidence, or change yaw/speed limits.
- **Evidence:** Mission contract tests cover immutable identity and lookahead;
  ROS interface tests cover typed route transport; planner-facade tests reject
  an empty snapshot and accept reconstructed measured geometry. Focused Release
  builds and package CTest sets are required before commit.
- **Removal/review condition:** Remove compatibility `target/next_target`
  mirrors only in a versioned interface migration. Replace this snapshot only
  with an equal or stronger atomic mission-route object consumed by mission,
  planner, yaw, diagnostics, and acceptance logic.
- **Verification:** Build through `navigation_runtime` and
  `px4_navigation_external_mode`; run CTest for `navigation_mission`,
  `navigation_contracts`, `navigation_planning_backend`,
  `navigation_runtime`, and `px4_navigation_external_mode`. M2 must consume this
  route in yaw policy before claiming complete shared-route behavior.

### 2026-08-28 - Define semantic route-lookahead yaw independently of MINCO

- **Owner:** Planner route-yaw reference policy; runtime integration remains a
  separate M2 behavior change.
- **Scope:** Add a pure `FaceRouteLookahead` policy driven only by immutable
  mission-route geometry and measured P/V/yaw. Lookahead scales with horizontal
  speed, progress cannot jump beyond the active waypoint at crossing geometry,
  and reversal lookahead is capped at the STOP-TURN-GO boundary. Standstill,
  pure vertical motion, invalid route support, or a vanishing horizontal
  bearing holds measured yaw.
- **Safety impact:** This checkpoint does not yet select executable yaw and does
  not alter map, position trajectory, PX4 setpoint, waypoint acceptance, rate
  limit, or acceleration limit. It removes MINCO/path-shape ownership from the
  semantic target definition so a later integration cannot silently use local
  path loops as mission heading commands.
- **Evidence:** Unit tests cover eight horizontal bearings, shortest-angle
  unwrap, standstill, pure vertical motion, generation-independent straight
  flight, outgoing-corner lookahead, 180-degree reversal, and self-crossing
  geometry bounded by active waypoint identity.
- **Removal/review condition:** Replace only with an equal or stronger explicit
  mission yaw mode. Never restore implicit bearing from each optimized local
  trajectory as the default mission-yaw source.
- **Verification:** Build `navigation_planning_backend` in Release mode and run
  `test_route_yaw_reference` plus the complete backend CTest set. Runtime MAIN
  and BACKUP integration, PX4-limit capture, and open-map SITL are required in
  later M2 commits.

### 2026-08-28 - Execute MAIN route yaw and BACKUP heading hold with hard dynamics

- **Owner:** Planner MAIN/BACKUP yaw generation and final candidate admission.
- **Scope:** MAIN yaw now tracks the semantic route-lookahead target and no
  longer samples bearing from each optimized MINCO position trajectory. BACKUP
  decelerates its incoming yaw rate toward the incoming heading instead of
  following braking-path shape. Yaw trajectories preserve duration and initial
  yaw/rate, unwrap by shortest angle, and project only angular excursion when
  the requested turn cannot fit. The obsolete `goal_yaw_en` and path-derived
  free-yaw implementation are removed.
- **Safety impact:** Every generated and concatenated command now has final
  hard certificates for both yaw rate and yaw acceleration. Initial operating
  limits are 1.0 rad/s and 0.3 rad/s2, conservatively below PX4 upstream
  autonomous defaults of 60 deg/s and 20 deg/s2. Those upstream values are a
  design reference, not proof of the active airframe; M2 SITL acceptance stays
  open until the runner records `MPC_YAWRAUTO_MAX`, `MPC_YAWRAUTO_ACC`, and the
  lower-level `MC_YAWRATE_MAX` for the tested vehicle.
- **Evidence:** Semantic-yaw tests prove full feasible turns and bounded
  partial turns, shortest-angle behavior, rate/acceleration ceilings, heading
  hold, corner lookahead, and post-transition reversal rotation. All backend
  tests and all mission/contracts/runtime/PX4 package tests must pass before
  commit. PX4 source references are the official
  `multicopter_autonomous_params.yaml` and `mc_att_control_params.yaml` on the
  PX4-Autopilot main branch.
- **Removal/review condition:** Replace only with an equal or stronger explicit
  yaw state machine whose target is mission-owned and whose complete executable
  bundle remains rate/acceleration certified. Never restore local optimizer
  tangent as an implicit mission-heading source or weaken the hard dynamics to
  make a short-duration turn complete.
- **Verification:** Release-build through `navigation_runtime` and
  `px4_navigation_external_mode`; run their complete CTest dependency set.
  Then capture active PX4 limits and run open-map component SITL covering
  straight flight, 90-degree pass-through, standstill, and reversal before M2
  can be marked accepted.

### 2026-08-28 - Record route-yaw provenance and active PX4 yaw limits

- **Owner:** Planner diagnostics, structured runtime trace, and PX4 SITL startup
  evidence.
- **Scope:** Export route-yaw source, semantic target, lookahead, route progress,
  configured rate/acceleration limits, and measured candidate maxima into every
  planner solve trace. Print effective `MPC_YAWRAUTO_MAX`, `MPC_YAWRAUTO_ACC`,
  and `MC_YAWRATE_MAX` values into the PX4 artifact log at startup.
- **Safety impact:** Observability only. These fields do not select yaw, change
  a limit, admit a candidate, alter a PX4 parameter, or convert a missing value
  into a pass. Missing or unparsable active PX4 values leave M2 acceptance open.
- **Evidence:** C++ diagnostics tests verify configured limits; planner-trace
  tests verify lossless numeric normalization; runtime-contract tests require
  all three PX4 parameter queries in the startup wrapper.
- **Removal/review condition:** Replace only when equivalent typed PX4
  parameter telemetry and route-yaw provenance are included in each immutable
  run manifest. Never infer active airframe limits solely from source defaults.
- **Verification:** Release-build `navigation_planning_backend` and
  `navigation_runtime`; run their CTest sets plus
  `tools.runtime.tests.test_planner_trace` and
  `tools.runtime.tests.test_runtime_contract`. The next SITL report must retain
  the raw PX4 values and structured yaw trace before any M2 PASS decision.

### 2026-08-28 - Certify measured-state rebase yaw before bundle admission

- **Owner:** Planner measured-state rebase handoff.
- **Scope:** Apply the configured yaw-rate and yaw-acceleration certificates
  while searching the duration of the C3 rebase connector, in addition to the
  existing position PVAJ and flatness envelopes.
- **Safety impact:** This closes a fail-closed availability defect without
  changing either hard yaw limit. A connector that is position/flatness-valid
  but yaw-dynamically invalid is stretched and rechecked; if no common
  certified duration exists, the previous command is retained as before.
- **Evidence:** The first authoritative 3 m/s multi-waypoint SITL after route
  yaw integration reached waypoint 1, then repeatedly produced rebase yaw
  acceleration up to 1.42 rad/s2 (last attempt 0.5206 rad/s2) against the
  0.3 rad/s2 hard limit and entered `PAUSED_SAFETY_STOP`. A focused unit test
  rejects the short connector and accepts a longer connector under the same
  unchanged limits.
- **Removal/review condition:** Replace only with an equal or stronger unified
  position/yaw connector-duration solver. Do not remove the final bundle yaw
  certificate or increase its limit to recover planner availability.
- **Verification:** Release-build and run the complete
  `navigation_planning_backend` CTest set, then regenerate the authoritative
  manifest and repeat `long_three_pillars_multiwaypoint` at 3 m/s. M2 remains
  open until repeated representative yaw and mission evidence passes.

### 2026-08-28 - Keep sharp waypoint turns inside a live acceptance fillet

- **Owner:** Planner pass-through route geometry and terminal boundary state.
- **Scope:** When a sharp corner cannot use the long outgoing route lookahead,
  replace the exact-waypoint endpoint with the existing traversability-checked
  three-point fillet inside the mission acceptance ball. Cap its outgoing
  terminal speed from the actual fillet radius and unchanged acceleration
  limit.
- **Safety impact:** No waypoint radius, map policy, corridor tolerance, or
  V/A/J gate is relaxed. The prior behavior ended at the exact corner with a
  non-zero incoming velocity, leaving the next `PlanFromRest` solve an
  instantaneous direction change. The fillet keeps the old command executable
  across measured acceptance and gives MINCO certified geometry in which to
  rotate velocity. If any fillet point or segment is not traversable, the
  planner retains the exact-boundary fail-closed behavior.
- **Evidence:** Authoritative 3 m/s SITL at `c2a9cc0` cleared waypoint 2 at
  2.73 m/s, then repeated an eastbound terminal command at `(50,5)` while the
  next leg required southbound motion; three corridor/dynamic failures caused
  `PAUSED_SAFETY_STOP`. Unit evidence fixes the 90-degree fillet at 0.675 m
  inside a 0.9 m acceptance ball and derives a 1.162 m/s terminal cap from the
  unchanged 2.0 m/s2 acceleration limit.
- **Removal/review condition:** Replace only with a longer mission-route
  trajectory that spans the same corner under equal or stronger map and
  dynamic certificates. Never recover availability by enlarging waypoint
  acceptance or corridor violation limits.
- **Verification:** Release-build and run the complete backend CTest set, then
  regenerate the exact-HEAD manifest and repeat the 3 m/s three-pillar mission.
  Inspect waypoint transition velocity, fillet trace, yaw maxima, clearance,
  and mission coverage; one improved run is not final acceptance.

### 2026-08-28 - Expose immutable snapshot export mode and extent

- **Owner:** Mapping actor and runtime mapping telemetry.
- **Scope:** Record whether each publication used a bounded patch or full-grid
  export, the fail-closed reason for a full export, exported base/inflated cell
  counts, resulting patch depth, and cumulative full/patch publication counts.
- **Safety impact:** Observability only. Snapshot cadence, freshness, patch
  depth, map semantics, command recertification, and all failure gates remain
  unchanged. The fields close the evidence gap between a costly export and its
  architectural cause before any performance behavior is changed.
- **Evidence:** The third exact-HEAD 3 m/s run stopped before the sharp corner
  after a 502 ms LiDAR arrival gap. In the same run mapping callback p95 was
  72.8 ms and published snapshot export mean/max were 35.3/60.8 ms, but prior
  telemetry could not distinguish patch cost from periodic full flattening.
- **Removal/review condition:** Replace only with equivalent or stronger typed
  per-publication provenance. Do not increase the freshness window or reduce
  map evidence to conceal an export latency tail.
- **Verification:** Build mapping and runtime, run their complete CTest sets,
  run the runtime report contract tests, then capture a representative
  recorded-data or SITL distribution containing the new fields before choosing
  an optimization.

### 2026-08-28 - Export snapshot patches in row-linear circular-map order

- **Owner:** ROG-Map planning-grid export adapter.
- **Scope:** Preserve the exact patch bounds and X/Y/Z logical ordering while
  precomputing circular X/Y hashes and walking contiguous Z rows. Apply the
  same indexing strategy to evidence and inflated layers.
- **Safety impact:** Performance implementation only. No changed region,
  occupancy threshold, virtual-plane rule, UNKNOWN policy, map size, snapshot
  cadence, freshness window, or fail-closed gate changes. Patch output remains
  detached and is still validated by `MappingWorldSnapshot` before publication.
- **Evidence:** Exact-HEAD 3 m/s artifact
  `external-mode-check-20260828T091412-909377` recorded 465 patches with mean
  1.95M base and 2.34M inflated cells. All patches covered 52.9--88.6 percent
  of the base grid and averaged 45.96 ms, while 116 full exports averaged
  11.82 ms. Source review found patch export repeated circular index conversion
  per voxel, whereas full export amortized X/Y conversion per row.
- **Removal/review condition:** Revert if patch/full cell semantics differ at
  any index, including after signed map slides or at virtual ground/ceiling.
  Revisit the patch architecture if repeated representative distributions do
  not materially lower export p95/p99 or if snapshot memory/query tails grow.
- **Verification:** Compare every patch cell with the same region in a full
  export before and after signed-axis slides; build and run vendor, mapping and
  runtime tests; then repeat representative recorded-data/SITL and compare
  full/patch export and callback distributions without changing hard gates.

### 2026-08-28 - Do not cross a mission boundary from a visibility prefix

- **Owner:** Planner receding-horizon route geometry.
- **Scope:** Permit pass-through lookahead into the next mission leg only when
  the certified guide endpoint reaches the actual active mission waypoint.
  A visibility-truncated local prefix remains on the current leg and advances
  monotonically on later replans; it cannot masquerade as the waypoint.
- **Safety impact:** Correctness and fail-closed route ownership. No horizon,
  acceptance radius, corridor tolerance, map policy, or dynamic limit changes.
  All local prefixes and final candidates retain existing world and V/A/J
  certificates. The change removes an uncertified early route transition.
- **Evidence:** Exact-HEAD artifact
  `external-mode-check-20260828T092022-915192` bounded waypoint `(85,-5)` to
  local prefix `(70,-5)` from the vehicle near `(57,-5)`, then incorrectly
  extended toward next waypoint `(85,5)` and moved the guide endpoint to about
  `(77,-1)`. Repeated MINCO time stretches then violated the corridor and the
  third `PlanFromRest` failure caused a safety stop at waypoint 4.
- **Removal/review condition:** Replace only with a typed route-progress model
  that distinguishes internal horizon frontiers from measured mission
  boundaries. Never enable outgoing-leg geometry before the active waypoint is
  reached or enlarge corridor/acceptance thresholds to hide the diagonal cut.
- **Verification:** Unit-test a distant prefix and a true boundary endpoint;
  run the complete planner backend tests and exact build; repeat the 3 m/s
  multi-waypoint scenario, checking guide endpoints remain on the current leg
  until measured waypoint acceptance and cross only afterward.

### 2026-08-28 - Expose semantic-yaw infeasibility at MAIN and BACKUP

- **Owner:** Semantic yaw optimizer and planner solve diagnostics.
- **Scope:** Record input yaw position/rate/acceleration, available duration,
  normalized target/delta, full-turn maxima, zero-displacement hold maxima,
  active limits, and a typed failure reason whenever MAIN or BACKUP yaw
  construction fails.
- **Safety impact:** Observability only. The interpolation, partial-turn search,
  position trajectory, yaw limits, candidate admission, and fail-closed result
  are unchanged. No failed yaw candidate becomes executable.
- **Evidence:** Exact-HEAD run
  `external-mode-check-20260828T092448-919753` repeatedly reported only
  `YawTrajOpt failed` around the `(50,5)->(50,-5)` turn for both MAIN and
  BACKUP. Source review shows BACKUP asks a finite-rate yaw state to finish at
  the same angle with zero terminal rate/acceleration in the position backup's
  duration; the previous boolean API hid whether even that zero-displacement
  polynomial violated the 1.0/0.3 rad limits.
- **Removal/review condition:** Replace with equivalent typed per-candidate
  yaw evidence in the structured planner trace. Never widen yaw limits or
  accept a failed yaw suffix merely to recover availability.
- **Verification:** Unit-test an infeasible short same-heading stop; build and
  run all planner backend tests; repeat representative SITL and use the numeric
  failure state to derive a bounded yaw-stop or duration policy.

### 2026-08-28 - Remove roundoff-only degree from extrema polynomials

- **Owner:** Shared continuous polynomial root and trajectory certificates.
- **Scope:** Classify leading and trailing coefficients relative to the
  polynomial's own scale before root solving/counting, and normalize the retained
  coefficients before solving. The relative threshold is bounded by floating-
  point roundoff (`64 * epsilon * coefficient_count`); no dynamics limit or
  root interval is relaxed.
- **Safety impact:** Prevents LU roundoff from promoting an analytically lower-
  degree extrema equation into an ill-conditioned higher-degree solve that can
  miss an interior velocity/acceleration/jerk maximum. Candidate values remain
  evaluated from the original trajectory at all certified roots and endpoints.
- **Evidence:** A 0.2 s minimum-jerk stop should have coefficients
  `[0, 50, -20, 0, 0.8, 0]`, but interpolation produced a harmless leading
  `-6.66e-13`. The old root solve then reported 3.83 rad/s2—or nearly zero in a
  repeated construction—instead of the analytic 6 rad/s2 peak.
- **Removal/review condition:** Replace only with an equal or stronger scaled
  root-isolation or interval certificate. Do not use sampled maxima or increase
  dynamic limits to compensate for missed roots.
- **Verification:** Unit-test the perturbed analytic piece for physical V/A/J
  maxima and fail-closed limit checking; run all backend tests and repeat
  representative SITL after the dependent yaw behavior is separately committed.

### 2026-08-28 - Stop rotating yaw with a forward minimum-jerk displacement

- **Owner:** Semantic yaw optimizer for MAIN and BACKUP suffixes.
- **Scope:** If the requested turn and exact-angle hold are both infeasible,
  attempt the free-terminal-position minimum-jerk stop
  `delta_yaw = 0.5*w0*T + a0*T^2/12`. The suffix preserves initial yaw/rate/
  acceleration, advances in the physically consistent direction, and ends at
  zero yaw rate and acceleration. It remains subject to unchanged continuous
  1.0 rad/s and 0.3 rad/s2 certificates.
- **Safety impact:** Closes an availability defect without accepting a limit
  violation or extending an uncertified command. Exact-angle hold previously
  forced a rotating state to reverse back to its initial angle. If the analytic
  stop still exceeds either limit, optimization continues to fail closed.
- **Evidence:** Artifact `external-mode-check-20260828T093028-924584` recorded
  51 BACKUP failures with 2--3.5 s durations and exact-angle hold acceleration
  maxima up to 0.61 rad/s2. Offline evaluation of the analytic stop over all 51
  recorded states retained max rate <=0.417 rad/s and max acceleration <=0.290
  rad/s2; every sample was feasible under the unchanged active limits.
- **Removal/review condition:** Replace only with an equal or stronger bounded
  yaw state machine or a jointly optimized position/yaw safety suffix. Never
  retain non-zero terminal yaw rate at handover or widen active yaw limits to
  make an exact-angle reversal pass.
- **Verification:** Unit-test both an infeasible 0.2 s stop and the recorded
  3.0 s feasible state; run all backend tests and repeat representative SITL,
  requiring zero admitted yaw-limit violations and improved MAIN/BACKUP
  availability before acceptance.

### 2026-08-28 - Reject folded MAIN trajectories in active-route coordinates

- **Owner:** Planner final candidate admission and immutable mission route.
- **Scope:** Before world authorization, analytically evaluate every position
  extremum of the unexecuted MAIN intervals along the active incoming route
  tangent. Reject a candidate whose internal high-water regression exceeds the
  existing route-progress backtrack tolerance (0.5 m). BACKUP is not weakened
  or reclassified, and no ENU-axis velocity sign is used.
- **Safety impact:** Prevents a nominal optimizer result from commanding a
  multi-metre reverse/U-turn while still targeting a waypoint ahead. The gate
  is role-aware and fail-closed for malformed applicable candidates. Explicit
  reverse missions still require a separate STOP-TURN-GO/recovery role rather
  than silently treating reverse translation as ordinary MAIN.
- **Evidence:** Exact-HEAD artifact
  `external-mode-check-20260828T094723-936773` commanded about +2.9 m/s then
  -2.0 m/s on the same incoming leg to waypoint 2; measured waypoint distance
  grew from about 14.7 m to 21.6 m before recovering. Artifact
  `external-mode-check-20260828T093028-924584` independently contained a MAIN
  fold of about 4.17 m on waypoint 5 to 6.
- **Removal/review condition:** Replace only with a full route-tube progress
  certificate plus explicit nominal/rejoin/reverse/recovery states. Do not ban
  negative ENU X/Y velocity, apply the gate to emergency braking, or enlarge
  waypoint acceptance to hide regression.
- **Verification:** Unit-test an analytic advance-then-fold polynomial and a
  forward lateral detour; run all backend tests and repeat the exact
  three-column mission, requiring rejected folds to retain a valid prior bundle
  or transition to certified BRAKE without high-speed yaw reversal.

### 2026-08-28 - Make MINCO optional after immutable nominal certification

- **Owner:** Nominal trajectory optimizer candidate selection.
- **Scope:** Freeze and independently certify the pre-LBFGS piecewise MINCO
  seed before optimization. If a non-cancelled MINCO refinement fails
  numerically or its final corridor/dynamics/flatness gates reject it, copy the
  exact frozen seed only when its precomputed corridor, PVAJ boundary,
  route-boundary, V/A/J and flatness certificate is valid. Otherwise fail
  closed as before. Record whether the seed was used and its reject stage.
- **Safety impact:** Removes MINCO convergence as a single point of command
  availability without weakening any hard gate or reconstructing a fallback
  from mutated optimizer state. Cancellation remains terminal; an uncertified
  seed never becomes executable. Final route-regression and latest-world
  authorization still run on the selected bundle.
- **Evidence:** Artifact `external-mode-check-20260828T095508-942471` repeatedly
  exhausted MAIN bundles because MINCO candidates missed hard V/A/J limits by
  small and large margins, even though the deterministic seed had already been
  frozen and certified in source but was selected only for corridor rejection.
  The run later failed tracking continuity at waypoint 5; no limit was tuned.
- **Removal/review condition:** Replace with an explicitly product-owned
  deterministic CONNECTOR/NOMINAL generator. MINCO may remain refinement, but
  must never regain sole ownership of command availability or mutate the
  certified fallback object.
- **Verification:** Unit-test exact-copy/reject selection, run all backend and
  runtime tests, then repeat the three-column run and require explicit seed-use
  evidence, unchanged hard limits, route-regression/world authorization, and
  no worse connector/tracking continuity before acceptance.

### 2026-08-28 - Keep mission-route yaw invariant during cross-track and terminal correction

- **Owner:** planner semantic-yaw reference. **Scope:** derive moving yaw from
  the immutable route chord rather than the vector from measured vehicle
  position to the lookahead point. At the terminal route arc, use the incoming
  chord so a bounded correction after overshoot does not reverse heading.
- **Safety impact:** position planning, collision certificates, waypoint
  acceptance, and yaw rate/acceleration limits are unchanged. The change
  removes cross-track and overshoot error from the heading reference; explicit
  mission reversals still use the existing stop-turn-go boundary and outgoing
  route chord. Invalid or purely vertical support continues to hold yaw.
- **Derivation and cost:** artifact
  `.artifacts/runtime/external-mode-check-20260828T100206-949033` completed all
  waypoints at 3 m/s, but the final correction from just beyond waypoint 8
  changed route yaw from the incoming-leg heading (about `0.245 rad`) toward
  `2.5 rad`. Source inspection showed the reference used
  `target_point - measured_position`, which necessarily points backward after
  overshoot. The replacement is constant-time and uses the already immutable
  route geometry.
- **Evidence:** focused route-yaw tests cover cross-track displacement,
  terminal overshoot, arbitrary bearings, corner lookahead, and explicit
  reversal. Then run backend/runtime tests, Release build, and repeated
  long-three-pillars SITL while checking yaw continuity and mission completion.
- **Removal/review condition:** replace only if route heading becomes an
  explicit versioned mission field with equivalent overshoot and reversal
  semantics; never restore raw waypoint-vector yaw for terminal recovery.
- **Verification command:** source `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`; run the focused `test_route_yaw_reference`, backend and
  runtime suites, `make build`, then
  `MAP_PROFILE=long_three_pillars_multiwaypoint SPEED_CAP_MPS=3 make external-mode-check`.

### 2026-08-28 - Preserve physics-derived lookahead through genuine corners

- **Owner:** Planner pass-through route geometry and nominal corridor gate.
- **Scope:** Apply the existing stopping/replan/receding-distance lookahead to
  genuine corners as well as shallow route boundaries. A genuine corner marks
  its mission waypoint as a hard route-boundary cell; the bounded
  acceptance-ball fillet remains the fallback when no long outgoing prefix is
  map-certified.
- **Safety impact:** No planner cadence, planning horizon, waypoint acceptance
  radius, map policy, corridor tolerance, dynamic limit, or execution lease is
  relaxed. The route-boundary cell fixes the optimizer junction at the mission
  waypoint, and the complete candidate still requires corridor, continuous
  V/A/J, flatness, latest-world, route-regression, and execution admission.
  Failure to search, certify, allocate, or optimize the longer route falls back
  to the existing local fillet and ultimately fails closed.
- **Derivation and cost:** At 3 m/s with the mission limits of 2 m/s2 and
  4 m/s3, the existing physics helper requires 7.2 m: jerk-limited stopping
  distance plus two 0.2 s forward-replan intervals and the configured 3 m
  receding distance. Artifact
  `.artifacts/runtime/external-mode-check-20260828T104135-974023` retained the
  prior finite suffix at waypoint 6, but the corner guide ended only 0.675 m
  into the outgoing leg and expired after repeated new-goal solve failures.
  Source inspection found the genuine-corner route-boundary branch unreachable
  behind an outer `!genuine_corner` condition.
- **Evidence:** Unit evidence requires a 90-degree corner to remain eligible
  for the 7.2 m outgoing lookahead and rejects a search window no larger than
  two map voxels. The existing route-boundary tests prove the corridor gate is
  preserved during simplification, fixes the junction at the waypoint, and
  assigns distinct incoming/outgoing time. Full backend build/tests and
  repeated SITL remain required.
- **Removal/review condition:** Replace only with an explicit multi-segment
  mission route generator that provides an equal or longer certified command
  and equivalent waypoint-boundary ownership. Do not lower planner frequency,
  enlarge acceptance radius, retain stale world certificates, or disable the
  boundary gate to conceal corner solve latency.
- **Verification:** Run the complete backend tests and exact Release build,
  then repeat the 3 m/s three-column mission. Inspect required/certified
  lookahead, command duration, route-boundary acceptance, optimizer and rebase
  failures, yaw continuity, route regression, clearance, and mission coverage.

### 2026-08-28 - Certify MAIN regression in mission-polyline coordinates

- **Owner:** Final MAIN candidate admission and immutable mission route.
- **Scope:** When a candidate contains an optimizer-pinned junction at the
  active waypoint and the mission has an outgoing segment, evaluate exact
  polynomial progress extrema on the incoming route arc before that junction
  and the outgoing route arc after it. Candidates without that exact boundary
  continue to use the conservative active-segment certificate.
- **Safety impact:** The existing 0.5 m backtrack tolerance is unchanged and is
  applied independently to both route phases. BACKUP remains outside this MAIN
  gate. The change removes false rejects caused by projecting an outgoing turn
  onto the old incoming tangent; it does not authorize reverse motion, corner
  cutting, or an acceptance-radius shortcut. The waypoint switch requires a
  candidate junction within 0.1 mm of the immutable mission waypoint, matching
  the hard optimizer pin rather than using measured acceptance radius.
- **Derivation and cost:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T104858-979310` repeatedly
  produced 5--7.2 m certified corner lookahead and dynamically feasible MINCO
  results, then rejected many with reported incoming-axis regressions from
  0.51 m to several metres. The old certificate used one tangent for the whole
  candidate even though the route-boundary optimizer fixes an exact waypoint
  junction and the immutable route already owns both adjacent segments. The
  runtime cost remains analytic polynomial root evaluation once per piece; no
  sampling interval is introduced.
- **Evidence:** Unit evidence accepts a two-piece 90-degree route whose
  outgoing leg bows laterally on the old axis while progressing monotonically
  on the new arc, and rejects a 4 m advance-then-fold after the same corner.
  Existing tests retain straight, negative-ENU, lateral-detour, folded MAIN,
  and BACKUP-role coverage. Full backend tests and repeated SITL remain open.
- **Removal/review condition:** Replace only with a full route-tube projection
  certificate that is at least as strict for reverse motion and resolves
  overlapping/180-degree route ownership explicitly. Never enlarge the
  backtrack tolerance or exempt corner candidates from regression checking.
- **Verification:** Run all backend tests and exact Release build, then repeat
  the three-column mission. Require no false incoming-axis rejection after a
  pinned boundary, zero admitted folds, bounded cross-track/yaw, and complete
  mission evidence across more than one run.

### 2026-08-28 - Size pass-through route windows at mission cruise speed

- **Owner:** Planner pass-through horizon geometry.
- **Scope:** Derive the required outgoing route window from the mission maximum
  velocity rather than the instantaneous measured/guide speed. Continue to
  bound it by the finite outgoing leg, remaining planning horizon, traversable
  A* prefix, and immutable-map segment certificate.
- **Safety impact:** No velocity/acceleration/jerk limit, planner cadence,
  search deadline, map policy, waypoint radius, or candidate gate is changed.
  This is a conservative increase in requested geometry at low speed; failure
  to find or certify it retains the acceptance-fillet/fail-closed path. The
  same complete corridor, continuous dynamics, flatness, route-regression,
  latest-world and execution checks authorize the resulting command.
- **Derivation and cost:** At the declared 3 m/s mission cap, 2 m/s2 and
  4 m/s3 limits, 0.2 s forward-replan time and 3 m receding distance, the
  existing physics envelope is 7.2 m. Artifact
  `.artifacts/runtime/external-mode-check-20260828T105516-984820` showed the
  requested pass-through window changing from about 3.0 m near rest to 7.2 m
  at cruise. Consecutive optimizer terminal routes therefore changed while the
  vehicle accelerated, and the old 3.7 m-short prefix expired before a stable
  replacement committed.
- **Evidence:** Unit evidence fixes the cruise window at 7.2 m and proves it
  does not collapse to the zero-speed 3 m receding-only value. The prior
  physics, finite-leg, map-certification and insufficient-search-window tests
  remain authoritative. Backend tests, repeated SITL, and recorded-data
  distributions remain required.
- **Removal/review condition:** Replace only with an adaptive window that has
  hysteresis and proves an equal command-availability envelope from measured
  latency distributions. Do not lower planner cadence or increase deadlines to
  hide terminal-route churn.
- **Verification:** Run all backend tests and Release build, then repeat the
  same three-column mission. Compare lookahead variance, command duration,
  optimizer/rebase failures, waypoint coverage, speed continuity, yaw and
  clearance across multiple runs.

### 2026-08-28 - Restart tracking divergence without a reverse connector

- **Owner:** Planner hot-replan continuity boundary and runtime restart FSM.
- **Scope:** When the measured position or yaw exceeds its existing tracking
  budget, stop hot stitching and return the existing `NEW_TRAJ` transition so
  the next solve starts from a fresh measured state. A measured pose that is
  not traversable still returns failure. Remove the polynomial that joined the
  current command state back to the historical measured state.
- **Safety impact:** This removes a nominal reverse/rejoin behavior; it does
  not ban negative ENU velocity or affect a legitimate mission whose bearing
  is west/south. The currently committed command remains immutable and
  world-certified while the runtime schedules the restart. No tracking,
  command-anchor, route-regression, V/A/J, flatness, world, or waypoint gate is
  enlarged or bypassed.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T105832-989061` logged
  measured-state connectors as long as `8.978 s` for a position discrepancy
  of about `0.340 m`; later final admission rejected these candidates for
  route regression up to several metres. The connector starts at the command
  ahead of the vehicle and ends at an already historical measured state, so
  increasing its duration makes the stale-target and reverse-motion problem
  worse rather than recovering tracking.
- **Removal/review condition:** Replace only with an explicit tracking
  recovery controller that targets a predicted future route state and proves
  command continuity, route monotonicity, dynamics and latest-world safety.
  Never restore a connector whose endpoint is the measured state captured at
  solve start, and never hide divergence by increasing the tracking budget.
- **Verification:** Unit-test continue/restart/fail-closed classification, run
  all backend tests and the exact Release build, then inject command lag and
  yaw error in repeated three-column SITL runs. Require zero reverse-rebase
  connector logs, bounded handover residuals, successful measured-state
  restart or certified stop, and no stale-state command commit.

### 2026-08-28 - Fillet pass-through corners inside mission acceptance

- **Owner:** Pass-through guide geometry and final route-regression
  certificate.
- **Scope:** Do not pin a nonzero-speed C3 trajectory to the exact centre of a
  genuine corner. Allow the optimizer to fillet through the already configured
  waypoint acceptance region. At final admission, change from incoming to
  outgoing route coordinates at the closest polynomial junction inside that
  region; stop waypoints remain single-phase.
- **Safety impact:** The acceptance radius is an existing measured mission
  contract, not a new tolerance. Every polynomial still requires continuous
  corridor, V/A/J, flatness, yaw, latest-world and execution authorization.
  Route regression remains analytically bounded on both legs and measured
  waypoint acceptance still owns mission advancement. No corner may be cut
  outside the configured ball.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T110635-996862` reached the
  first two mission waypoints and then produced a certified `7.2 m` outgoing
  window at the `(50,5)->(50,-5)` corner. The exact route-boundary pin made
  every seed and refined trajectory violate its corridor, commonly by hundreds
  or thousands of metres. A C3 trajectory cannot retain nonzero velocity while
  changing tangent instantaneously at one exact point shared by perpendicular
  corridor phases; it must either stop or use finite curvature inside the
  acceptance region.
- **Removal/review condition:** Replace only with a route generator that owns
  an explicit curvature-continuous fillet/tube and preserves the same measured
  acceptance, per-leg monotonic progress, dynamic and world certificates.
  Never restore an exact nonzero-speed corner pin or enlarge acceptance to hide
  infeasible geometry.
- **Verification:** Unit-test an in-ball phase junction, folded outgoing leg,
  negative-ENU mission and stop-waypoint exclusion; run all backend tests and
  the Release build. Then repeat 90-degree and arbitrary-bearing SITL missions,
  requiring measured entry into the acceptance ball, bounded corner speed and
  clearance, stable yaw, zero admitted folds and improved command renewal over
  more than one run.

### 2026-08-28 - Retain a certified command while restarting from measured state

- **Owner:** Runtime planner-result FSM and immutable command execution store.
- **Scope:** If a measured-state `PlanFromRest` replacement fails while the
  same goal still has a committed command, route the result through the
  existing retained-command validation path. Apply the consecutive
  rest-to-rest failure budget only when no executable command is available.
- **Safety impact:** A planner return code never renews a command by itself.
  Retention still requires finite trajectory metadata, exact goal/localization
  identity, latest-world swept validation, remaining duration and the existing
  command-anchor bound. If those checks fail, the runtime builds a
  measured-state emergency brake or clears command exposure and fails closed.
- **Evidence:** In artifact
  `.artifacts/runtime/external-mode-check-20260828T111157-1003608`, generation
  `88` was still active and certified to continue from about `(44.5,5.1)`
  through the waypoint toward `(50.2,0.43)`. Three fast PlanFromRest optimizer
  failures charged the no-command failure budget and handed control to PX4
  Hold while the UAV was still `3.82 m` before the waypoint. The same failures
  in hot-replan mode already use current-world/suffix validation instead.
- **Removal/review condition:** Replace only with a unified supervisor whose
  states explicitly separate command availability from planner solve mode and
  preserve equivalent latest-world, anchor and expiry checks. Never suppress
  the failure budget when no valid command exists or retain a bundle based only
  on an old certificate.
- **Verification:** Unit-test failed PlanFromRest with and without a committed
  command; run runtime/backend tests and Release build. In repeated SITL,
  inject replacement failures before/after backup switch and after expiry;
  require no premature mode exit while validation passes and deterministic
  emergency-stop/fail-closed behavior once it does not.

### 2026-08-28 - Optimize pass-through junctions inside the acceptance ball

- **Owner:** Route-lookahead corridor generation and nominal MINCO optimizer.
- **Scope:** Restore one route-boundary corridor cell for every pass-through
  lookahead. Initialize its junction at the active waypoint, then allow MINCO
  to move that junction within the corridor and configured acceptance ball.
  Keep the existing outside-ball objective and independent post-solve hard
  check; do not overwrite the variable with the exact waypoint or clear its
  gradient during optimization.
- **Safety impact:** `MISSION_PROGRESS_INVARIANT` preservation. A long smooth
  command can no longer cut outside the region that the measured mission gate
  requires. This does not enlarge the radius or infer acceptance from a
  planner endpoint. Corridor containment, immutable-world sweep, route
  regression, V/A/J, flatness, backup and PX4 tracking gates are unchanged.
- **Evidence:** In artifact
  `.artifacts/runtime/external-mode-check-20260828T111725-1010293`, the active
  waypoint was `(20,5,3)` with radius `0.9 m`, while measured closest approach
  was about `1.35 m` and the reference passed near `y=3.9`. The planner then
  continued to a `7.2 m` outgoing lookahead, leaving the mission on the old
  waypoint until the certified stop expired. The previous exact-junction gate
  made nonzero-speed corner tangent changes infeasible; disabling the gate
  removed the mission-boundary guarantee entirely.
- **Removal/review condition:** Replace only with an equal or stronger
  continuous acceptance-region certificate shared by planner and mission.
  Never restore exact-centre pinning for pass-through corners, enlarge the
  mission radius to hide a miss, or accept a waypoint from command geometry.
- **Verification:** Build and run planner/backend trajectory tests, then repeat
  shallow, 90-degree and arbitrary-bearing SITL. Require ordered measured
  acceptance, an in-ball committed junction, bounded speed/clearance/yaw and
  no optimizer starvation or reverse route regression.

### 2026-08-28 - Encode waypoint acceptance as a convex corridor cell

- **Owner:** Route-boundary corridor generator and continuous corridor
  certificate.
- **Scope:** Intersect the generated waypoint boundary polytope with an
  axis-aligned cube centered at the waypoint with half extent
  `acceptance_radius/sqrt(3)`. The complete cube lies inside the 3-D spherical
  acceptance region, so every continuously certified boundary piece enters
  the mission region while its junctions remain optimizer variables.
- **Safety impact:** Conservative `MISSION_PROGRESS_INVARIANT`. This does not
  enlarge the configured acceptance radius, accept a waypoint from planner
  geometry, or relax corridor tolerance. It strengthens the planner-side
  prerequisite; MissionController still advances only from current/recent
  measured odometry. World, dynamics, flatness, route and backup gates remain
  unchanged.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T112401-1017565` showed that
  a broad point corridor plus a soft spherical penalty produced continuous
  corridor excess from about `3.98 m` to `79 m` and starved replacement solves
  near `x=14`. The previous ungated run could solve but missed the `0.9 m`
  measured acceptance sphere by about `0.45 m`. A bounded convex cell removes
  both ambiguous ownership cases without hard-pinning the waypoint centre.
  Follow-up artifact
  `.artifacts/runtime/external-mode-check-20260828T112759-1023232` confirmed
  that the convex geometry removed the former multi-metre corridor rejects,
  but exposed lost seed-line provenance after `Polytope::CrossWith`; every
  lookahead then failed the downstream vertical-envelope gate before MINCO.
- **Removal/review condition:** Replace only with an equal or stronger convex
  representation or analytic continuous ball-intersection certificate. Do
  not use a circumscribed box, a sampled-only check, a larger radius, or a
  soft objective as the authorization boundary.
- **Verification:** Unit-test that the boundary cell contains its centre,
  rejects an axis point at `0.8 * radius`, and preserves seed provenance
  through the vertical-envelope stage; run all backend tests/build and
  repeated three-column SITL with ordered measured acceptance and bounded
  planner/corridor latency.

### 2026-08-28 - Bound remote-goal A* by the executable visibility horizon

- **Owner:** Initial route search, receding-horizon guide construction and
  immutable-world route certification.
- **Scope:** Before A* expansion, cap its search horizon to the smaller of the
  remaining planning horizon and effective visibility horizon. Preserve the
  remote mission goal as the heuristic direction and controller-owned progress
  target; continue truncating and re-certifying the returned local prefix.
- **Safety impact:** This does not treat UNKNOWN as free under a stricter
  mission policy, extend map support, weaken collision checks, or accept a
  planner endpoint as mission progress. It prevents solver time being spent on
  geometry that cannot enter the current executable certificate. The same
  inflated-map, continuous-corridor, dynamic, backup and latest-world gates
  remain authoritative.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T113212-1029572` received the
  140 m goal from rest with a 14 m visibility floor and 45 m planning horizon.
  Four consecutive solves exhausted the A* stage at `84.483--89.139 ms`,
  committed no command, and failed closed. The old order asked A* to expand to
  45 m toward/around the first pillar and only afterwards truncated the result
  to 14 m, so work outside the executable prefix consumed the complete search
  budget.
  Follow-up artifact
  `.artifacts/runtime/external-mode-check-20260828T113759-1036939` confirmed
  that A* produced executable commands and advanced the measured UAV to about
  `(80.5,-7.6,3.0)` with speed up to `6.12 m/s`. It then exposed a stale
  postcondition: prefixes already bounded by the pre-search cap returned
  `truncated=false`, and the caller rejected them solely because it still
  required post-search truncation. Accepting an already-within-bound prefix is
  not a bypass; its finite length and every inflated-map segment remain
  re-certified before optimization.
- **Removal/review condition:** Replace only with a route-search hierarchy that
  separately represents global mission progress and a locally executable
  certified prefix with an equal or smaller search bound. Do not compensate by
  increasing A* timeout, reducing supervisor cadence, or enlarging the map from
  a single run.
- **Verification:** Unit-test finite/invalid and planning-vs-visibility bounds,
  run all backend tests and Release build, then repeat the 140 m three-column
  profile. Require A* to return a local prefix within budget, no no-command
  safety stop, continued measured mission progress, and latency distributions
  over repeated SITL plus representative recorded data.

### 2026-08-28 - Select immutable snapshot export before copying large patches

- **Owner:** ROG-Map planning-grid export and mapping snapshot publication.
- **Scope:** Estimate the exact clipped base/inflated patch cell counts without
  copying them. Use a patch only below 40 percent of the stable full-snapshot
  owned bytes; otherwise export one flat full snapshot directly. Publication
  cadence, world identity and accumulated changed-region ownership are
  unchanged.
- **Safety impact:** Representation/performance only. Both forms contain the
  same evidence and inflated layers at the same revision; no observation is
  dropped, no ray is shortened, no map cell is reclassified and no freshness,
  collision or revalidation gate changes. Invalid/overflowing estimates choose
  the already validated full export.
- **Evidence:** Complete three-column SITL artifact
  `.artifacts/runtime/external-mode-check-20260828T114150-1042798` measured 247
  patches spanning 50--81 percent of the 6,282,392-byte grid; they averaged
  `21.319 ms`, versus `12.577 ms` for 105 full exports. Recorded Mid-360 2x
  artifact `.artifacts/runtime/dataset-20260828T114615-1045031` PASSed all
  2756 observations and independently measured 679 patches: below 40 percent
  averaged `7.071 ms`, while 40--80 percent buckets averaged
  `10.103--15.551 ms`; 626 full exports averaged `8.261 ms`. The crossover is
  therefore distribution-derived from both SITL and real sensor data, not one
  latency sample.
- **Removal/review condition:** Replace with shared immutable chunk storage or
  an independently benchmarked export selector that preserves exact snapshot
  identity and query parity. Do not raise publication age or omit map layers to
  reduce callback time.
- **Verification:** Unit-test estimate/export byte parity after signed-axis
  slides, patch/full selection, malformed/empty bounds and arithmetic guards; run
  mapping/backend tests and Release build. Repeat recorded Mid-360 and
  three-column SITL, requiring identical accounting/mission safety with lower
  snapshot and callback p95/p99 plus bounded peak live bytes.

### 2026-08-28 - Anchor local route search to the active mission leg

- **Owner:** Mission-route geometry, local A* target selection and receding-
  horizon planner integration.
- **Scope:** Select a deterministic guidance target on the active route segment
  from the larger of measured progress and planning-start projection. Keep a
  remote guidance point beyond the executable horizon, bounded to two current
  visibility horizons and the active waypoint boundary. This distinction lets
  A* terminate at `REACH_HORIZON` after a local obstacle detour instead of
  treating the horizon-edge spine point as an exact endpoint. Continue to
  truncate output to one executable horizon and apply the existing inflated-
  map, corridor, dynamics, backup and latest-world certification gates.
- **Safety impact:** This does not create a remote executable trajectory through
  UNKNOWN space, rotate the ROG-Map voxel grid, skip A*, weaken collision
  semantics, enlarge waypoint acceptance or accept planned state as mission
  progress. It replaces a changing remote-goal heuristic with a stable local
  route-spine target while all existing fail-closed certificates remain
  authoritative. If route evidence is invalid or the active leg has no forward
  support, target selection fails closed to the existing bounded search path.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T114150-1042798` completed the
  140 m mission but produced 91 commits from 228 solves, 119 failed solves and
  repeated multi-second commit gaps aligned with seven-to-eight speed recovery
  waves. Guide prefixes were commonly 22--25 m, while each solve still used the
  remote waypoint as A* target and only bounded the returned path afterwards.
  Artifact `.artifacts/runtime/external-mode-check-20260828T120327-1058786`
  reproduced the ownership failure more severely: 42 commits, 53 failed solves
  and an External Mode exit near 57 m, despite complete mapping observation
  accounting. A stable route-spine target is therefore separated from local
  obstacle deformation instead of tuning planner frequency or safety limits.
  The first exact-horizon implementation at commit `1b64eff` was intentionally
  retained as a rollback/audit checkpoint. SITL artifact
  `.artifacts/runtime/external-mode-check-20260828T121836-1074658` reduced route
  cross-track p95 from `8.530 m` to `1.996 m`, but failed at the first pillar:
  the spine target was exactly 14 m away, so an obstacle detour longer than 14 m
  could neither reach that exact endpoint nor report horizon progress. Eighteen
  consecutive `PathSearch` failures expired the command lease near x=20.7 m.
  Keeping guidance beyond the executable radius corrects that semantic error;
  it does not increase the certified prefix or its safety horizon.
  Exact-HEAD follow-up artifact
  `.artifacts/runtime/external-mode-check-20260828T122307-1085850` completed both
  ordered waypoints and the 140 m mission with no collision, `4.429 m` minimum
  clearance and route path-length ratio `1.018`. Compared with artifact
  `...T114150-1042798`, commit ratio improved from `91/228` (39.9 percent) to
  `125/207` (60.4 percent), maximum inter-commit gap fell from `4.508 s` to
  `2.060 s`, and 3.0-to-4.2 m/s speed-recovery episodes fell from four to two.
  This is functional evidence, not final acceptance: 82 solves still did not
  commit and measured speed p95 reached only `4.579 m/s` against the requested
  `5.0 m/s`, so deterministic corridor-contained command ownership remains an
  open follow-on rather than a reason to relax the speed or dynamics contract.
- **Removal/review condition:** Replace only with an explicit global-route/local-
  deformation hierarchy that preserves monotonic measured progress, active
  waypoint boundaries and equal-or-stronger local certification. Do not replace
  it with one remote MINCO command through unobserved space or a planner endpoint
  as waypoint acceptance evidence.
- **Verification:** Unit-test straight, cross-track, active-boundary, high-water
  and invalid-route geometry; run all backend tests and Release build. Then run
  repeated three-column SITL and representative recorded data, requiring fewer
  solve/commit gaps and speed waves without degraded clearance, map accounting,
  ordered measured acceptance, altitude or External Mode retention.

### 2026-08-28 - Add a convex-hull-contained nominal baseline before MINCO

- **Owner:** Nominal trajectory construction, corridor containment and MINCO
  candidate selection.
- **Scope:** Build an immutable degree-seven piecewise Bezier candidate from
  the same corridor junctions and durations used to initialize MINCO. Require
  every Bernstein control point to lie in its assigned convex corridor, retain
  exact endpoint P/V/A/J, and share P/V/A/J at every junction. Reduce only
  internally selected junction velocity through a finite deterministic scale
  set when required for adjacent-corridor containment. MINCO remains the
  quality refinement; the Bezier baseline may replace it only after the same
  continuous corridor, route-boundary, velocity, acceleration, jerk and
  flatness certificate passes.
- **Safety impact:** Convex-hull containment gives a construction-time geometric
  guarantee in addition to, not instead of, continuous polynomial hard gates.
  Endpoint derivatives, mission progress, UNKNOWN policy, world freshness and
  command lease are unchanged. No failed boundary/dynamics/flatness candidate
  is published; invalid inputs or a boundary control point outside its corridor
  remain fail-closed. Internal velocity may be reduced to preserve containment,
  but acceptance and speed thresholds are not relaxed.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T122307-1085850` completed the
  route-backbone mission, yet 82 of 207 solves did not commit. Earlier artifact
  `.artifacts/runtime/external-mode-check-20260828T120327-1058786` exposed that
  92 of 102 immutable pre-LBFGS MINCO interpolation seeds failed at corridor
  stage 2, so the nominal fallback was structurally unavailable even before
  optimizer dynamics were considered. Unit geometry covers straight moving-
  boundary continuity, a corner requiring deterministic junction-speed
  reduction, full sampled containment and rejection of an immutable endpoint
  derivative pointing outside its corridor.
  The first exact-HEAD integration artifact
  `.artifacts/runtime/external-mode-check-20260828T123635-1104863` completed
  both ordered waypoints without collision and retained `4.505 m` minimum
  clearance, but selected no corridor-Bezier fallback. Candidate failures moved
  primarily from corridor stage 2 to dynamics stage 5, with observed baseline
  acceleration and jerk far above the unchanged `2 m/s^2` and `4 m/s^3`
  mission limits. The cause was a timing inconsistency: every internal junction
  was assigned up to `4.9 m/s` even when adjacent short pieces implied a much
  lower secant velocity. Internal junction velocity is therefore now the
  bounded average of adjacent position/time secants, with the existing finite
  containment reductions retained. A regression case proves that unequal
  straight pieces with equal `5 m/s` secants reproduce constant velocity with
  negligible acceleration and jerk rather than injecting a speed spike.
  Exact-HEAD follow-up artifact
  `.artifacts/runtime/external-mode-check-20260828T124328-1112739` again
  completed both waypoints without collision (`4.499 m` minimum clearance),
  and improved commit ratio from `133/245` (54.3 percent) to `131/220`
  (59.5 percent), but still selected the baseline zero times. Of 220 solves,
  169 baseline certificates failed at dynamics stage 5. A geometrically valid
  baseline may therefore make at most three deterministic rebuild attempts
  with a uniform duration multiplier derived from
  `max(v/v_max, sqrt(a/a_max), cbrt(j/j_max))`, 5 percent reserve and an
  unchanged upper bound of 4.0. Every rebuild repeats corridor, boundary,
  route-boundary, V/A/J and flatness certification; a finite retry cannot
  publish merely because its duration was increased. Unit evidence begins
  with a dynamically invalid but corridor-contained trajectory and verifies
  that only a fully recertified bounded-duration rebuild is accepted.
- **Removal/review condition:** Replace only with an equal-or-stronger
  corridor-contained baseline such as a formally bounded B-spline/Bezier or
  direct convex trajectory program that preserves endpoint and junction state
  continuity. Do not restore MINCO as the sole command-availability owner or
  use sampled collision checks as a substitute for continuous containment.
- **Verification:** Run focused baseline/certificate tests, all backend tests
  and Release build. Re-run repeated three-column and multi-waypoint SITL plus
  recorded-data shadow planning; require higher commit ratio and shorter commit
  gaps without increased clearance, route-regression, dynamics, altitude or
  External Mode failures. Treat a lower internal-velocity scale as diagnostic,
  not permission to weaken mission cruise acceptance.

### 2026-08-28 - Expose deterministic nominal fallback outcomes in reports

- **Owner:** Planner decision-trace normalization and runtime report evidence.
- **Scope:** Preserve the producer-owned `exp_used_certified_seed` and
  `exp_certified_seed_failure_stage` fields in every normalized rolling-bundle
  record. Summarize the explicit fallback-use count and failure-stage
  histogram without inferring either value from trajectory IDs, solve stage or
  commit counters.
- **Safety impact:** Observability only. This does not change planner selection,
  trajectory construction, PX4 handover, thresholds or acceptance. Missing or
  malformed fields remain absent rather than being synthesized.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T125006-1120692` was BLOCKED
  by a fail-closed PX4 Hold handover after `planner backend PVA command anchor
  is not near vehicle`; it did not complete the second waypoint. The raw trace
  contained 244 complete planner records, one explicit certified-seed use and
  failure-stage counts `{0: 36, 2: 7, 3: 6, 5: 195}`. Before this change those
  two fields were discarded by the report adapter, hiding that the bounded
  duration baseline was selected only once while dynamics remained the
  dominant rejection. Re-rendering the same immutable samples now reproduces
  those exact counts in `planning.rolling_bundle_trace`.
- **Removal/review condition:** Remove only if the producer and report migrate
  to a typed equivalent that preserves source and rejection stage explicitly.
  Never replace these fields with an inference from `solve_stage=backup` or a
  successful commit.
- **Verification:** Run `tools.runtime.tests.test_planner_trace`; re-render the
  cited session and require the report summary to equal the raw decision-trace
  counts. Run the complete runtime-contract suite before commit.

### 2026-08-28 - Match corridor baseline PVAJ to nonuniform guide timing

- **Owner:** Deterministic corridor-Bezier junction-state construction.
- **Scope:** Compute each internal velocity from adjacent position/time
  secants, acceleration from their centered finite difference and jerk from
  adjacent acceleration slopes. Apply the existing finite containment scale
  to all three internal derivatives together. Test only the four Bernstein
  controls owned by that junction while selecting its scale, then verify every
  control of every complete piece before coefficient conversion.
- **Safety impact:** Endpoint P/V/A/J remains immutable and every accepted
  polynomial still has all Bernstein controls inside its assigned convex
  corridor, C3 continuity, continuous route-boundary checks and complete
  V/A/J/flatness certification. This changes a deterministic fallback shape;
  it does not weaken MINCO, UNKNOWN, map freshness, backup or PX4 gates.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T125006-1120692` exposed 195
  dynamics-stage fallback failures in 244 solves. The previous builder forced
  acceleration and jerk to zero at every short corridor junction even when
  adjacent path secants were turning or accelerating, so each degree-seven
  piece had to recreate the derivative change internally. Exact polynomial
  regression now reproduces constant-velocity motion across unequal piece
  times and constant-acceleration motion across a junction with negligible
  spurious jerk. A corner case still proves deterministic derivative reduction
  when the full timed state would leave either adjacent convex corridor.
- **Removal/review condition:** Replace only with an equal-or-stronger
  nonuniform spline/QP state construction that keeps analytic convex-hull
  containment and complete post-construction certification. Do not return to
  zero derivatives at every subdivision or sampled-only containment.
- **Verification:** Run the six focused corridor baseline tests, optimizer
  fallback tests, all backend/workspace tests and Release build. Then compare
  explicit fallback use and failure-stage distributions on repeated
  three-column SITL and recorded data, requiring no clearance, route, altitude,
  anchor, dynamics or latency regression.

### 2026-08-28 - Prefer piece-local duration allocation for nominal fallback

- **Owner:** Deterministic fallback time allocation.
- **Scope:** Before uniform duration retry, derive one bounded multiplier per
  polynomial piece from its exact extrema using
  `max(v/v_max, sqrt(a/a_max), cbrt(j/j_max))` plus the existing 5 percent
  reserve. Rebuild shared junction states with those nonuniform times and
  require the complete trajectory certificate. Uniform retries remain finite
  secondary candidates if local allocation cannot be certified.
- **Safety impact:** No trajectory is retimed in place and no per-piece extrema
  authorizes publication. Retiming is only an input to a fresh convex-hull
  construction followed by corridor, boundary, route, V/A/J and flatness
  certification. Multipliers remain bounded by 4.0 and mission limits are
  unchanged.
- **Evidence:** Two exact-HEAD runs of `943d08a` completed the mission without
  collision but did not reproduce fallback availability: artifact
  `.artifacts/runtime/external-mode-check-20260828T125931-1129631` selected
  three fallbacks and committed `149/213` solves, while artifact
  `.artifacts/runtime/external-mode-check-20260828T130210-1131201` selected zero
  and committed `130/220`. The three selected uniform fallbacks lasted
  `15.08--21.13 s` and peaked at only `1.86--2.59 m/s`, showing that one local
  corner violation was slowing every straight piece. Unit evidence constructs
  an already-certified straight piece followed by a high-jerk piece and proves
  the allocator leaves the straight multiplier at 1.0 while increasing only
  the violating piece.
- **Removal/review condition:** Replace with a jointly optimized convex time
  allocation that provides equal-or-stronger per-piece dynamic and corridor
  guarantees. Do not restore global slowdown as the first response to one
  local derivative peak.
- **Verification:** Run focused baseline and optimizer tests, all workspace and
  runtime-contract tests, Release build, then repeated three-column SITL and
  recorded data. Require shorter certified fallback duration, repeatable
  fallback availability, lower dynamics-stage rejection and no latency,
  clearance, route, altitude, anchor or speed regression.

### 2026-08-28 - Trace deterministic corridor fallback retry boundaries

- **Owner:** Nominal-trajectory optimizer diagnostics and runtime report
  normalization.
- **Scope:** Publish the initial corridor-seed construction failure stage,
  retry attempt and construction-success counts, last retry certificate stage,
  selected construction mode, and selected maximum duration multiplier. Keep
  the fields source-owned through the backend facade, runtime decision trace
  and report summary; missing fields remain missing rather than inferred.
- **Safety impact:** Observability only. Retry ordering, trajectory
  construction, certification, selection, mission limits, UNKNOWN policy,
  backup handling, PX4 handover and acceptance gates are unchanged. The
  optimizer still publishes a corridor baseline only after the complete
  fail-closed certificate succeeds.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T130718-1138466` completed the
  three-column mission without collision and committed `160/232` solves, but
  selected zero deterministic fallbacks while 189 solves ended at dynamics
  stage 5. Existing fields cannot distinguish a retry that failed to construct
  a corridor-contained polynomial from one that constructed successfully and
  then failed the complete certificate. That ambiguity blocks an architectural
  decision between replacing the Bezier builder and replacing its time/dynamic
  allocation.
- **Removal/review condition:** Remove only when an equal-or-stronger typed
  planning-decision trace identifies every deterministic candidate's build and
  certificate boundary. Do not infer these outcomes from MINCO success,
  committed trajectory counts or the final fallback-use bit.
- **Verification:** Run `test_corridor_bezier_seed`,
  `test_exp_optimizer_seed`, `tools.runtime.tests.test_planner_trace`, all
  workspace tests and a Release build. Then run exact-HEAD three-column SITL
  and require report aggregates to equal the raw runtime fields before using
  the evidence for an algorithm decision.

### 2026-08-28 - Resume only an exactly recertified freshness-suspended command

- **Owner:** Runtime world-freshness and execution-command ownership boundary.
- **Scope:** When world evidence becomes stale, record the exact currently
  exposed bundle generation before suspending publication. A later immutable
  snapshot may restore publication without a replacement planner commit only
  after that same bundle is successfully recertified, remains inside its
  declared time interval, still matches the active localization and goal
  epochs, and the execution lease permits exposure. Preserve whether the
  suspended command was safety-suffix-owned and publish suspension/recovery
  counters.
- **Safety impact:** The freshness gate is unchanged and no command is emitted
  while world evidence is stale. Recovery cannot use a different generation,
  expired trajectory, changed goal/localization epoch, failed certificate or
  latched execution lease. A map-invalidated trajectory still schedules
  measured-state PlanFromRest and remains unavailable.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T131511-1148220` on `1df07fa`
  stopped at 37.443 s after the last command lease expired. At 36.672 s the
  world-freshness rejection count increased from 56 to 60; command publication
  stopped at cycle 227 even though fresh snapshots 312, 314 and 315 then
  produced successful full revalidations 74, 75 and 76 of the unchanged bundle
  generation 41. Replacement PlanFromRest solves continued failing in
  `main_minco`, so the old contract unnecessarily required a new commit after
  already proving the retained trajectory against the new world.
- **Removal/review condition:** Replace only with an equivalent typed lease FSM
  that explicitly distinguishes stale-world suspension, successful exact-
  bundle recertification, map invalidation, epoch transition and terminal
  handover. Never restore command availability merely because a fresh map
  arrived.
- **Verification:** Unit-test exact generation, epoch, validity interval,
  planner-failure and execution-latch rejection. Run all runtime/workspace
  tests and Release build, then repeated three-column SITL. Require observed
  suspend/recovery pairs, no command-validity expiry caused solely by a
  successfully recertified bundle, and no regression in clearance, collision,
  anchor, freshness or PX4 handover gates.

#### Exact-HEAD screening result

- Artifact `.artifacts/runtime/external-mode-check-20260828T132546-1159975`
  on `49f69ca` observed the intended recovery for bundle generation 21 and did
  not reproduce the prior command-validity expiry. It progressed to about
  82 m, committed `99/163` solves, emitted 1852 finite setpoints, and retained
  4.419 m minimum truth clearance with zero collisions.
- This is not mission acceptance. PX4 handed over to Hold when the unchanged
  generation 99 command exceeded the existing tracking envelope by 3 mm
  longitudinally (`0.753/0.750 m`) while the runtime was retaining its
  certified safety suffix after replacement-plan failures. The mission did not
  reach waypoint 1 and speed p95 was 4.408 m/s. Keep the tracking gate and
  mission threshold unchanged; review repeated tracking-error evidence and the
  main/backup trajectory contract before any threshold decision.

### 2026-08-28 - Use the complete certified local visibility budget

- **Owner:** Planner global-route/local-visibility horizon contract.
- **Scope:** Raise the ordinary-speed visibility floor from 14 m to the
  existing 23 m cap. The 45 m global route window remains the geometric
  backbone and the executable trajectory remains bounded by current map
  evidence, corridor generation and the existing main/backup certificate.
- **Safety impact:** Speed, acceleration, jerk, collision radius, UNKNOWN and
  OUT_OF_MAP policy, solve deadline, command freshness and PX4 gates are
  unchanged. This increases local search/corridor/optimization work and must be
  rejected if latency tails, feasibility or map containment regress.
- **Evidence:** Across exact-head three-column artifacts, successful local
  trajectories are repeatedly followed by bursts of replacement-plan failure;
  artifact `.artifacts/runtime/external-mode-check-20260828T132546-1159975`
  committed `99/163` solves and still entered a retained suffix before stopping
  at about 82 m. Its planner total p95 was 45.975 ms under the 180 ms deadline,
  leaving measured compute headroom for an A/B of the already configured 23 m
  cap. This change is not evidence that 23 m is accepted.
- **Removal/review condition:** Revert to 14 m if repeated SITL or recorded-data
  distributions show deadline, dynamics-stage, clearance or continuity
  regression. Replace fixed local visibility only with a bounded adaptive
  policy derived from map extent, braking horizon and measured latency tails.
- **Verification:** Run planner config/backend/workspace tests and Release
  build. Compare exact-head repeated three-column runs against the cited 14 m
  artifacts for guide length, commit ratio, planning p50/p95/p99, command
  suspension/recovery, speed continuity, tracking, clearance and mission
  completion. Do not change the 180 ms deadline or tracking envelope during
  this A/B.

### 2026-08-28 - Preserve command availability across certified bundle replacement

- **Owner:** Runtime command-store/mapping recertification transaction.
- **Scope:** When command publication loses the pointer-identity race to a
  mapping-recertified copy or a newer planner commit, skip that stale
  publication without clearing command availability if the store now owns a
  valid, non-older bundle for the same active localization and goal epochs.
- **Safety impact:** The stale sampled pointer is never published. Continued
  availability requires the current store bundle to remain structurally valid,
  inside its declared validity interval, on the active epochs, with no planner
  failure latch and a valid execution lease. Older generation, invalidation,
  expiry, epoch change or lease failure still clears command state fail-closed.
- **Evidence:** Exact-head artifact
  `.artifacts/runtime/external-mode-check-20260828T133615-1171708` recertified
  generation 102 as map revisions advanced. At simulation time 48.624 s the
  last valid command still sampled generation 102 at trajectory time 0.740 s,
  far before its 6.656 s declared end. A concurrent recertified pointer caused
  `publishIfCurrent()` to lose identity, clear availability, and the next
  ordinary MINCO replacement failure became terminal at cycle 301. The mission
  entered `PAUSED_SAFETY_STOP` near 83 m despite the current certified bundle.
- **Removal/review condition:** Replace only if command sampling and world
  recertification become one versioned transaction that returns an explicit
  retry/current-bundle result. Do not weaken pointer/world identity checks.
- **Verification:** Run `test_planner_fsm`, execution-store tests, full
  workspace tests and Release build. Repeat `MAP_PROFILE=long_three_pillars_speed
  make external-mode-check`; verify stale pointers are never exposed and map
  recertification races no longer produce a zero-generation emergency while a
  current valid bundle remains in the store.

#### Exact-HEAD screening result

- Commit `e33f4f8` passed 26/26 focused FSM tests, the full workspace test
  suite (including 184 runtime contract tests), and the 23-package Release
  build. Artifact
  `.artifacts/runtime/external-mode-check-20260828T134232-1181253` completed the
  140 m mission, accepted both waypoints, retained 4.318 m minimum truth
  clearance with zero collisions, and did not reproduce the generation-102
  recertification race.
- This remains screening, not mission acceptance. Speed p95 was 4.818 m/s
  against the 5.000 m/s profile requirement; all 142 committed trajectories
  selected BACKUP, while planner total p95/max were 79.226/180.272 ms. The
  command-store race is closed at unit and one-run screening level, but MAIN
  feasibility and latency tails remain open and require repeated evidence.

### 2026-08-28 - Rejected experiment: defer replans until the MAIN boundary

- **Owner:** Runtime planner scheduling experiment, reverted by `016caa0`.
- **Scope:** Commit `5158bd4` kept the 5 Hz timer but skipped replacement solves
  while a certified MAIN prefix had more than one solve deadline plus three
  timer ticks remaining. Mapping recertification remained active throughout.
- **Safety impact:** No gate was relaxed and the mission completed, but the
  reduced replacement cadence allowed several trajectories to enter their
  braking suffix. The experiment was therefore reverted in full; no scheduler
  deferral remains in product behavior.
- **Evidence:** Artifact
  `.artifacts/runtime/external-mode-check-20260828T135131-1191985` reduced
  planning records from 233 to 94 and observed role transitions from 76 to 39,
  but produced four near-stops below 1 m/s around route x=23, 28, 75 and 107 m,
  each lasting about 1.5--2.5 s. Speed p95 fell to 4.782 m/s. This reproduces
  the reported stop/reaccelerate behavior and proves that lowering effective
  planner frequency before improving replacement feasibility is the wrong
  ordering.
- **Removal/review condition:** Historical rejected-experiment record; retain
  until a replacement architecture demonstrates reliable MAIN continuation
  and can safely derive a lower solve cadence from measured success tails.
- **Verification:** `git show 5158bd4`, `git show 016caa0`, and the cited
  artifact preserve the implementation, rollback and runtime evidence.

### 2026-08-28 - Rejected experiment: add jerk cost only to feasibility retries

- **Owner:** Nominal MINCO bounded feasibility retry experiment, reverted by
  `7df9c6f`.
- **Scope:** Commit `e33c40d` kept the initial nominal jerk objective disabled
  but added a `5.0e5` jerk penalty to the existing maximum-two retries after a
  strict jerk-certificate failure. No retry, iteration or deadline budget was
  changed.
- **Safety impact:** No hard gate was relaxed and the 140 m mission completed
  without collision, but the altered search direction increased MAIN/BACKUP
  switching and made speed continuity materially worse. The experiment was
  therefore reverted in full; the product configuration has no retry-only jerk
  weight.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T135907-1203584` reduced
  planner total p95/max from 79.226/180.272 ms to 71.987/98.558 ms and reduced
  non-finite optimizer messages from five to one versus artifact
  `.artifacts/runtime/external-mode-check-20260828T134232-1181253`. However,
  dynamics-stage failures did not improve (60 to 61), MAIN/BACKUP command-role
  transitions rose from 75 to 99, route-region episodes below 2 m/s rose from
  one to three, and measured speed p95 fell from 4.818 to 4.574 m/s. The first
  5--55 m remained below 4 m/s for 20.6 s. This is a local numerical benefit,
  not a continuity improvement.
- **Removal/review condition:** Historical rejected-experiment record. Revisit
  jerk conditioning only together with a trajectory parameterization whose
  certified continuation horizon and terminal velocity contract prevent the
  optimizer from trading repeated deceleration for feasibility.
- **Verification:** `git show e33c40d`, `git show 7df9c6f`, and the cited
  artifacts preserve the implementation, rollback and A/B evidence.

### 2026-08-28 - Rejected experiment: exclude BACKUP geometry from recovery guide

- **Owner:** Hot-replan MAIN/BACKUP boundary experiment, reverted by
  `7a81a83`.
- **Scope:** Commit `0ccc993` preserved the exact sampled BACKUP PVAJ splice
  state but retained zero metres of the committed braking suffix as nominal
  spatial guidance. MAIN replans continued to retain the configured 3 m
  continuity prefix. A* rebuilt route geometry immediately from a sampled
  BACKUP state.
- **Safety impact:** No hard gate was relaxed and the mission completed without
  collision, but removing the certified geometric prefix made recovery solves
  less reliable and shortened the next accepted MAIN prefix. The experiment
  was reverted in full; product behavior again retains the configured prefix
  regardless of the sampled trajectory role.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T141445-1215801` is compared
  with baseline `.artifacts/runtime/external-mode-check-20260828T134232-1181253`.
  Committed bundles fell from 142 to 124, replan code `-6` rose from 62 to 88,
  MAIN/BACKUP command-role transitions rose from 75 to 89, and BACKUP commands
  exceeded MAIN commands (1640 versus 1398). Median accepted switch time fell
  from 1.512 s to 0.995 s, the longest central-route episode below 3 m/s grew
  from 3.468 s to 5.200 s, and measured speed p95 fell from 4.818 m/s to
  4.650 m/s. Planner p95 improved from 79.226 ms to 72.080 ms, proving that the
  lower latency did not repair behavioral continuity.
- **Removal/review condition:** Historical rejected-experiment record. A future
  recovery design must preserve dynamically and geometrically feasible
  continuity while preventing a braking branch from becoming the nominal route
  objective. It requires an explicit branched trajectory/recovery contract,
  not deletion of the only certified geometric prefix.
- **Verification:** `git show 0ccc993`, `git show 7a81a83`, focused/full tests,
  Release build, and the cited A/B artifacts preserve implementation, rollback
  and runtime evidence.

### 2026-08-28 - Publish nominal candidate and fallback-seed dynamic extrema

- **Owner:** EXP optimizer diagnostics and runtime rolling-bundle trace.
- **Scope:** Every optimizer invocation publishes the last independently
  evaluated MINCO candidate V/A/J extrema and the immutable deterministic
  seed V/A/J extrema. Existing generic `maximum_*` trace fields now carry the
  candidate values; explicit `exp_certified_seed_maximum_*` fields distinguish
  fallback quality. Rejected candidates remain observable.
- **Safety impact:** Observability only. Candidate generation, optimization,
  hard V/A/J and flatness gates, MAIN/BACKUP selection, command admission and
  execution are unchanged. Non-finite or unavailable measurements remain
  omitted by the existing trace serializer rather than synthesized.
- **Evidence:** Baseline artifact
  `.artifacts/runtime/external-mode-check-20260828T134232-1181253` contains 60
  EXP-stage failures and 165 deterministic seeds failing the dynamics stage,
  but its rolling records leave `maximum_velocity_mps`,
  `maximum_acceleration_mps2`, and `maximum_jerk_mps3` null. Text logs show
  materially different failure classes, from small optimized overshoots to
  fallback seeds with jerk in the tens or hundreds of m/s^3. Those classes
  require different remedies and must be machine-readable before changing
  trajectory architecture or limits.
- **Removal/review condition:** Keep while MINCO or a deterministic fallback
  participates in nominal planning. Replace only with equally explicit
  candidate-specific certificate evidence in a versioned diagnostics schema.
- **Verification:** Run `tools/runtime/tests/test_planner_trace.py`, backend and
  workspace tests, Release build, then one unchanged three-column screening
  run and verify successful and failed solve records expose finite extrema
  whenever the corresponding candidate was evaluated.

### 2026-08-28 - Rejected experiment: densify bounded temporal feasibility search

- **Owner:** Nominal temporal projection experiment, reverted by `9a30d69`.
- **Scope:** Commit `5fee806` inserted reserve proposals at 1.10 and 1.25 times
  the analytically required reserve between the prior required and 1.50x
  points. Every proposal retained exact endpoint PVAJ and required unchanged
  corridor, route-boundary, V/A/J and flatness certificates.
- **Safety impact:** No hard gate was relaxed and the mission completed without
  collision. However, certifying more temporally stretched MAIN candidates
  traded numerical feasibility for path quality and cruise continuity. The
  experiment was reverted in full; the product again uses the original
  three-point temporal proposal set.
- **Evidence:** Exact-HEAD artifact
  `.artifacts/runtime/external-mode-check-20260828T143013-1230484` selected 34
  intermediate scales. Against immediate predecessor artifact
  `.artifacts/runtime/external-mode-check-20260828T142323-1223796`, EXP failures
  fell only from 91 to 86 and role transitions from 92 to 78, while speed p95
  fell from 4.817 to 4.494 m/s, median speed fell from 2.407 to 2.032 m/s,
  cross-track p95 rose to 11.551 m and failed its 8 m gate, and planner p95
  increased from 88.405 to 98.898 ms. Clearance remained 4.160 m with zero
  collision, so the rejection is driven by operational quality and latency,
  not a weakened safety result.
- **Removal/review condition:** Historical rejected-experiment record. Do not
  add duration-only MAIN fallbacks without a joint progress, route-reference
  and continuity quality contract. Dynamic feasibility alone is insufficient.
- **Verification:** `git show 5fee806`, `git show 9a30d69`, focused tests and
  the cited A/B artifacts preserve implementation, rollback and runtime
  evidence.

### 2026-08-28 - Publish nominal seed piece-duration range

- **Owner:** EXP hot-initialization diagnostics.
- **Scope:** Publish the minimum and maximum piece duration of the exact
  pre-optimizer MINCO initialization alongside its total duration and dynamic
  extrema for every solve.
- **Safety impact:** Observability only. Guide construction, corridor timing,
  optimization, certificates, command admission and execution are unchanged.
- **Evidence:** `processCorridorWithGuideTraj()` permits consecutive corridor
  overlaps to select the same nearest guide sample and clamps the resulting
  non-positive piece duration to 0.01 s. The three-column screening artifact
  `.artifacts/runtime/external-mode-check-20260828T142323-1223796` shows median
  deterministic-seed jerk 553.6 m/s^3 but did not expose its piece-duration
  range, preventing direct causal correlation in the rolling trace.
- **Removal/review condition:** Keep while corridor-to-guide timing seeds
  MINCO or any deterministic fallback. Replace only with equivalent per-piece
  timing evidence in a versioned trajectory schema.
- **Verification:** Run trace parser, planner/runtime tests and Release build;
  then verify an unchanged three-column screening run publishes finite ranges
  and correlate minimum duration with seed dynamics and EXP outcome.
