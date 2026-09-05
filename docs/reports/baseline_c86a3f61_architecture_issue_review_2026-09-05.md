# Baseline architecture and issue review — `c86a3f61`

Date: 2026-09-05
Branch: `codex/runtime-evidence-for-analysis`
HEAD: `c86a3f61df7bc7124b180f5aa31b311ee38d8c08`
Comparison base: `14c3f78b`

## 1. Review rules and verdict

This report separates four evidence levels:

- **FIXED — CODE/COMPONENT:** the source defect is removed and focused evidence exists.
- **IMPLEMENTED — RUNTIME UNQUALIFIED:** the intended source path exists, but current end-to-end evidence is insufficient.
- **OPEN:** source or runtime evidence still shows unresolved behavior or architecture debt.
- **BLOCKER:** the current baseline must not be used for SITL qualification until resolved.

The verdict for this baseline is:

> **OWNERSHIP HARDENING IS MEANINGFUL, BUT THE AGGREGATE BASELINE IS NOT BUILD-CLEAN, THE CURRENT-BODY REPLACEMENT DOES NOT MATCH THE CURRENT CONTRACT, AND END-TO-END QUALIFICATION REMAINS OPEN.**

Tóm tắt phục vụ quyết định kiến trúc:

- Các fix timeline, identity, PASS continuation, watchdog, analyzer, timing và terminal handover có giá trị và nên giữ.
- `TraversedFreeSpace` đã được loại khỏi production state, nhưng current-body replacement tại HEAD vẫn theo contract initial-only, chứa policy trùng và debug I/O trong collision oracle.
- Baseline chưa build sạch nên chưa đủ điều kiện chạy SITL có giá trị đánh giá.
- Nên sửa bốn điểm có phạm vi hẹp trước: build graph, debug I/O, current-body ownership mỗi evaluation cycle và general worker cancellation.
- Dual execution authority, command/evidence transport và continuous STOP là nợ kiến trúc tiếp theo; không nên trộn chúng vào cùng patch current-body.

The branch is clean and pushed. Since `14c3f78b`, it contains 41 commits and changes 82 files (`+9004/-2357`). This is a large refactor rather than a bounded patch. `navigation_runtime_node.cpp` is now 6,704 lines, `planner.cpp` is 4,861 lines, and the safety ledger is 19,220 lines. Those numbers do not prove a defect, but they confirm that ownership and policy are still concentrated in a few very large surfaces.

No result in this report is a flight-acceptance claim. Historical map results are listed by exact map/profile and are not pooled.

## 2. Current verification state

| Check | Result | Interpretation |
|---|---|---|
| Git state | Clean, HEAD equals pushed branch | Baseline provenance is clear |
| `git diff --check` | Pass | No whitespace/patch corruption |
| Canonical release build | **FAIL** at `navigation_planning_backend` | Current baseline is not build-qualified |
| Focused Python evidence/contract tests | **209/209 pass** | Analyzer/runtime tooling subset is green |
| Terminal handover isolated focused tests | **2/2 pass**, handover repeated **3/3** | Component evidence only |
| Current-HEAD C++ aggregate tests | Not available | Blocked by build graph before backend/runtime completion |
| Current-HEAD SITL | Not run | Prohibited as qualification while build and body blockers remain |
| Build manifest | Missing | No authoritative product artifact for SITL |

Canonical build command:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/runtime/build.py --mode release build
```

It fails because [`navigation_planning_backend/CMakeLists.txt`](../../src/planning/navigation_planning_backend/CMakeLists.txt#L99) calls `find_package(navigation_mapping REQUIRED)` for a production-snapshot test, while [`package.xml`](../../src/planning/navigation_planning_backend/package.xml) does not declare that test dependency. The resulting build environment does not contain `navigation_mapping` in `CMAKE_PREFIX_PATH`. Exact retained log: [`navigation_planning_backend/stderr.log`](../../log/build_2026-09-05_20-16-03/navigation_planning_backend/stderr.log).

An earlier partial rebuild produced a shutdown-test crash after only `navigation_mapping` had been rebuilt. That run mixed incompatible workspace artifacts and is classified as invalid infrastructure evidence, not as a confirmed runtime regression.

## 3. Findings closed at code/component level

| Item | Status at `c86a3f61` | Evidence and remaining boundary |
|---|---|---|
| R15 stale predecessor admission | **FIXED — CODE/COMPONENT** | `05ac49c8` revalidates predecessor ownership inside `ExecutionTimelineStore`; `944f553a` binds stale callback cleanup to the captured timeline. Runtime interleaving qualification is still useful but the source defect is closed. |
| Unsafe legacy execution-store mutation APIs | **FIXED — CODE/COMPONENT** | `9f61c302`, `d8832f67` and related store tests move pending/active decisions through one timeline snapshot and remove unsafe legacy entry points. |
| Redundant planner lifecycle mirrors | **FIXED — CODE** | `ae76ecca` removes write-only planner flags and removes planning-worker activity from the physical execution episode. |
| Duplicate PlanFromRest terminal authority | **FIXED — CODE/COMPONENT** | `fe5f0e20` removes the consecutive failure budget; the stopped-recovery deadline is the sole terminal retry authority. |
| PASS route-boundary suppression with outgoing lookahead | **FIXED — CODE/COMPONENT** | `cac1af47` derives the boundary from actual trajectory entry rather than endpoint connectedness. |
| PASS continuation ownership | **IMPLEMENTED — COMPONENT** | `f45df3a7` transports an immutable continuation witness from candidate to command and requires measured ordered crossing. End-to-end pass-through smoothness and post-boundary MAIN horizon remain unqualified. |
| Self-crossing route acceptance ambiguity | **FIXED — COMPONENT** | `6883571c` requires measured position to project onto the ordered incoming/outgoing route arc rather than using only an acceptance sphere. |
| `NO_NEED` versus `FINISH` semantics | **FIXED — CODE/FOCUSED TEST** | `ddd56ceb` retains the existing command for `NO_NEED` and builds an EXP-only complete candidate for `FINISH`. A direct old-BACKUP successor runtime witness is still pending. |
| Coarse known-free planning failure | **PARTIALLY FIXED** | `d62b0830` adds a typed MAIN known-free insufficiency outcome. Other root causes still require structured evidence; R05 is improved rather than completely closed. |
| R13 frame/time analyzer defect | **FIXED — TOOLING** | `31d017aa` adds quaternion rotation, explicit frame conventions, source timestamps, bounded interpolation and clock-mapping requirements. Historical conclusions are not automatically requalified. |
| R14 hard-coded E5 verdict/scope | **FIXED — TOOLING** | `af666431` requires exact run identity, map/route/speed/clock/frame provenance and stops synthesizing a verdict from incomplete input. Historical H8 labels remain unqualified. |
| R06 per-observation synthetic body clearing | **FIXED — IMPLEMENTATION** | The production MappingActor/ROG path no longer writes repeated vehicle-neighborhood misses into the probability map. Runtime obstacle qualification remains open. |
| R07 base pose used as LiDAR ray origin | **FIXED — IMPLEMENTATION** | Canonical sensor-origin transport and epoch/timestamp checks are present (`f092b3aa`, earlier mapping fixes). Runtime lever-arm qualification remains open. |
| Canonical route authority inside planning | **FIXED — INTERNAL CONTRACT** | `8c0d9e70`, `687929d5` and `02a7cc37` make immutable `RouteSnapshot` authoritative and update the manual producer. ROS compatibility mirrors still remain. |
| World publication and pending-check snapshots | **FIXED — CODE/COMPONENT** | `4d45c1bf` removes non-token publication APIs; `d8832f67` uses one timeline snapshot for pending decisions. |
| Stale watchdog activity markers | **FIXED — CODE/COMPONENT** | `91dccc5d` and `14875333` bind markers and serialize activity ownership. No representative latency distribution is claimed. |
| Independent causal trace atomics | **FIXED — INTERNAL OWNERSHIP** | `0c5adfd0` replaces them with an immutable `ExecutionTraceSnapshot` store. Transport separation is still open because the trace is copied into `NavigationCommand`. |
| Fake product timing parameters | **MOSTLY FIXED** | `228bf605` moves fixed planner/command/snapshot/timeout values into `PlanningTimingContract` and removes their runtime parameter ownership. Tests still pass an obsolete `planner_rate_hz` override, and `deployment_profile=hardware` is advertised then rejected. |
| Execution phase ordinal gap | **FIXED** | `74c75625` makes internal phases contiguous `0..4` and preserves old diagnostic codes through `executionEpisodePhaseTelemetryCodeV1()`. The old missing ordinal is no longer part of the internal FSM. |
| Completed terminal STOP handover | **FIXED — CODE/COMPONENT** | `b438f77b` binds the predecessor completion witness and cancellation to exact execution identity. `0f691cf1` hardens the public-contract test; merged by `c86a3f61`. Repeated representative runtime/SITL evidence remains open. |
| Completion identity, foreign cancellation lock and mapping shutdown | **FIXED — CODE/COMPONENT** | `a51d97f0`, `fd08e490` and `e628cda2` close bounded identity/lock/lifecycle defects without relaxing gates. |

## 4. Blockers before a meaningful SITL baseline

### B01 — backend test dependency graph is broken

**Severity:** P0 integration blocker.
**Status:** OPEN.

The clean canonical build stops before planner-backend and runtime tests. The immediate source mismatch is:

- [`CMakeLists.txt:99`](../../src/planning/navigation_planning_backend/CMakeLists.txt#L99): `find_package(navigation_mapping REQUIRED)` under `BUILD_TESTING`;
- [`CMakeLists.txt:127`](../../src/planning/navigation_planning_backend/CMakeLists.txt#L127): production facade test links `navigation_mapping::navigation_mapping`;
- [`package.xml`](../../src/planning/navigation_planning_backend/package.xml): no `test_depend` on `navigation_mapping`.

Architectural decision is required. Adding a test dependency is the smallest build fix. Moving the cross-layer production-snapshot proof into an integration-test package keeps the backend dependency direction cleaner. Either way, the current state cannot produce an authoritative build manifest.

### B02 — unconditional debug I/O is committed in the collision oracle

**Severity:** P1 real-time/performance blocker.
**Status:** OPEN, source-confirmed.

[`mapping_world_snapshot.hpp`](../../src/mapping/navigation_mapping/include/navigation_mapping/mapping_world_snapshot.hpp#L258) contains unconditional `std::fprintf(stderr, ...)` calls at the segment entry, guard, every visited cell and every body-support interval. Two calls are inside voxel traversal. This makes planning/safety-query latency depend on stderr buffering and the consuming terminal/log pipe, outside the planner deadline model.

These statements are development instrumentation and must be removed before any timing or smoothness run. Their latency impact has not been measured, so this report does not invent a numeric penalty.

### B03 — current-body behavior does not implement the current requirement

**Severity:** P1 behavior/architecture blocker.
**Status:** replacement WIP failed architecture review.

The old `TraversedFreeSpace` production history has been removed, which is correct. The replacement at HEAD is not complete:

- Runtime creates `CurrentBodySupport` only for `kStoppedMeasuredState` at [`navigation_runtime_node.cpp:4041`](../../src/runtime/navigation_runtime/src/navigation_runtime_node.cpp#L4041).
- [`PlanningRequest::valid()`](../../src/planning/navigation_planning/include/navigation_planning/planning_request.hpp#L118) disallows it for committed-future starts.
- Staged/committed recertification at [`planner.cpp:1791`](../../src/planning/navigation_planning_backend/src/planner_core/planner.cpp#L1791) is sensor-only.
- The safety ledger still describes a one-shot stopped-state witness, while the current product requirement is a measured current-body context recreated for every planning/safety evaluation cycle.

Consequently, a candidate can be admitted from rest and later be revoked when UNKNOWN follows the moving UAV body. This does not satisfy the desired stable-climb behavior.

The implementation is also spread across mapping, public world-model API, planning request, A*, corridor, planner, trajectory validator and runtime. Planner owns a support pointer plus `matches_start` and `admission_pending` booleans; A* and corridor retain their own support pointers and setters. This is too much mutable policy for an ephemeral collision exception.

Correct target:

```text
each evaluation cycle
    freeze {immutable world snapshot, fresh measured pose, physical body}
    UNKNOWN inside body-at-measured-now may be admitted
    OCCUPIED / OUT_OF_MAP / UNDEFINED remain rejected
    do not mutate ROG
    do not retain prior body volumes
    do not place body support at a predicted/future anchor
```

The same call-local collision context must be used by planning and active-command safety revalidation. A body-dependent certificate must be reevaluated with the next measured context; the disjoint-world-update shortcut is sufficient only once the remaining certificate is sensor-only.

This rule prevents the system from rejecting the space the UAV currently occupies. It does not prove that an unobserved gap above the body is free. Extending the moving mask beyond the physical body would be a separate operational permission to fly through UNKNOWN and must not be described as physical-body evidence.

### B04 — current-body query policies disagree

**Severity:** P2 liveness/maintainability blocker for the body cutover.
**Status:** OPEN.

The mapping oracle at [`mapping_world_snapshot.hpp:315`](../../src/mapping/navigation_mapping/include/navigation_mapping/mapping_world_snapshot.hpp#L315) explicitly permits `UNKNOWN -> KNOWN_FREE -> UNKNOWN` while each UNKNOWN interval stays inside the OBB. A direct mapping test at [`test_mapping_world_model.cpp:274`](../../src/mapping/navigation_mapping/test/test_mapping_world_model.cpp#L274) codifies that behavior. The final trajectory certificate separately rejects re-entry by interval ordering at [`trajectory_world_validator.hpp:360`](../../src/planning/navigation_planning_backend/include/planner_core/trajectory_world_validator.hpp#L360).

The final certificate prevents an end-to-end bypass, but A*/corridor can optimize a path that the final validator deterministically rejects. The snapshot also implements a separate complete DDA/supercover traversal for body support alongside the normal segment DDA. The body cutover needs one common traversal and one semantic owner.

## 5. Open correctness and liveness findings

### R01 — PX4/LIO alignment witness is incomplete

**Status:** OPEN, code-path finding; synchronized runtime proof missing.

The alignment contract still needs localization epoch, PX4 reset generation, LIO source timestamp, PX4 estimator sample timestamp, receive timestamps, pair skew, velocity-valid flags and a stationary window. A stale stationary pair must not latch a transform. This report does not conclude that GNSS caused the historical failure.

### R02 — present-time safety versus state-source-time evidence

**Status:** OPEN policy/evidence boundary.

The new trace records raw and time-aligned samples, which improves diagnosis. Current safety still needs a clear statement of whether it compares the command at evaluation time against a propagated estimate valid at evaluation time. Historical aligned residuals are attribution evidence and cannot silently replace the present-time safety predicate.

### R03/R04 — nominal envelope and effective replacement cadence

**Status:** IMPLEMENTED/OBSERVABLE, RUNTIME UNQUALIFIED.

The provisional P0 closed-loop MAIN envelope remains a deliberate behavior restriction and is not a certified vehicle capability. A 5 Hz planning timer is not evidence of five trajectory activations per second. Scheduled, started, staged and activated distributions must be measured separately on each scenario.

### R05 — structured planner failure provenance

**Status:** PARTIAL.

Known-free insufficiency is now typed. Nominal dynamics, corridor construction, backup feasibility, world recertification, deadline and transaction-stage failures still need retained low-level stage/reason evidence before causal conclusions.

### R08 — reset versus sliding UNKNOWN prior

**Status:** OPEN, component-reproduced historical behavior.

Full reset and sliding reset initialize probability cells differently, so identical miss evidence may reach KNOWN_FREE at different rates. This inherited ROG behavior can affect map-boundary and reset cases. It requires a product policy decision and deterministic reset/slide tests; it must not be tuned from one map run.

### R09 — external-vision diagnostic timestamps

**Status:** OPEN code contract, runtime reproduction pending.

Validity-edge and periodic diagnostics may use timestamps with different meanings. A later-arriving recovery edge can carry an older source stamp and be rejected as stale. The health gate should remain; the producer/consumer timestamp semantics need a monotonic typed contract.

### R10 — publication attempt is not delivery evidence

**Status:** OPEN observability limitation.

Worker publication counters may advance even when the odometry publisher returns early. Separate counters are still required for attempted publication, actual publish and downstream DDS receipt.

### R11 — planning budget does not cover the complete transaction

**Status:** OPEN.

The request and optimizer contain deadlines, but authorization, certification, recertification and staging do not all consume one absolute transaction deadline. Tail latency is therefore not bounded by one request-to-stage budget.

### R12 — positional and yaw terminal durations can disagree

**Status:** OPEN, component-reproduced historical behavior.

A short positional stop may end before yaw rate reaches zero, after which HOLD clamps yaw rate. The whole command end should be `max(position_stop, yaw_stop)` with a certified zero-PVAJ positional hold, or the candidate should be rebuilt/rejected.

### R16 — general PlanningWorker cancellation turnover race

**Severity:** P1/P2 liveness risk.
**Status:** OPEN, source-confirmed interleaving.

[`PlanningWorker::submit()`](../../src/runtime/navigation_runtime/include/navigation_runtime/planning_worker.hpp#L114) and bare [`cancelActive()`](../../src/runtime/navigation_runtime/include/navigation_runtime/planning_worker.hpp#L130) release the worker mutex before `planner_->cancelActiveSolve()`. The old job can finish, the worker can start the new pending job, and the old caller can then interrupt the new solve. The terminal identity-specific API correctly holds the mutex through the interrupt; the general paths do not.

The fix should reuse the terminal method's serialization pattern or attach cancellation to a solve generation. It does not require another FSM.

## 6. Architecture debt that still creates unpredictable behavior

### A01 — two mutable physical execution authorities

[`ExecutionEpisode`](../../src/runtime/navigation_runtime/include/navigation_runtime/execution_episode.hpp) stores phase, generation, command availability, failure latch, safety-suffix state and restart-from-rest. The node separately stores atomic [`ExecutionRecoveryState`](../../src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp#L382) with six states. Runtime admission reads both.

This is genuine duplicated physical policy state. A safe simplification is:

```text
ExecutionTimelineStore = command pointer/identity owner
ExecutionStateStore    = measured state owner
ExecutionPolicyRecord  = recovery/failure/restart owner
role/command presence  = derived from timeline + current sample
```

The desired and executing goals should remain distinct because desired identity can advance before physical successor activation. PX4 freshness, validity and Hold admission gates should also remain independent; their existence is not evidence of unnecessary FSM complexity.

### A02 — runtime orchestration remains latch-heavy

The node still retains active/executing/pending/deferred goals, foreign-mission cancellation epochs, hot-transition/skip flags, world-freshness suspension fields, completion witnesses, terminal generation, `ExecutionEpisode` and `ExecutionRecoveryState`. Some fields represent legitimate asynchronous identities; others mirror derived execution facts.

The next simplification should audit each field by owner, reader and derivability. Removing them wholesale would damage handover correctness. The strongest removal candidates are state that duplicates timeline role/presence or recovery policy already available in the intended single execution record.

### A03 — planner worker owns execution, but raw facade access remains

`PlanningWorker` owns the mutable planner instance, yet the runtime retains a raw `PlannerFacade*` and mapping callbacks call staged/committed validators directly. Internal planner mutexes reduce immediate race risk, but the ownership boundary is not clean. The desired end state is worker-exclusive mutable planner history plus a validator operating only on immutable candidate/world values.

### A04 — `NavigationCommand` is both control and evidence transport

[`NavigationCommand.msg`](../../src/contracts/navigation_contracts/msg/NavigationCommand.msg#L49) carries many diagnostic and temporal fields that the message itself says do not participate in admission or PX4 control. Runtime copies the immutable trace into every command sample. This couples control serialization/bandwidth to evidence schema growth.

Target:

- `NavigationCommand`: identity, lease, PVAJ, yaw, role, status and minimal certified transition fields;
- `NavigationExecutionTrace`: diagnostic sample keyed by complete command/solve identity.

### A05 — `NavigationGoal` still has geometry mirrors

[`NavigationGoal.msg`](../../src/contracts/navigation_contracts/msg/NavigationGoal.msg#L18) contains canonical `RouteSnapshot` plus target/acceptance/behavior/next-target compatibility fields. Internal authority is fixed, but transport simplification is incomplete. Remove mirrors only after all producers/consumers migrate and agreement is measured.

### A06 — compatibility alias survives in production

[`CommittedBundleStore`](../../src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp#L581) aliases `ExecutionTimelineStore`, and production `CommandSampler` still consumes the alias. Migrate production and tests to the canonical name, then remove the alias.

### A07 — terminal STOP remains a discrete approach policy

`plannerTerminalStopApproachDue()` still switches behavior using stopping distance. The long-term simplification remains a continuous speed law derived from cruise, curvature, visibility and stop constraints. This should follow a stable runtime baseline; it must not be combined with the body cutover or tracking-gate tuning.

### A08 — current documentation is too large and partly stale

The ledger is 19,220 lines and mixes current contracts with a long historical record. The current body entry still states an initial stopped-state contract that has been superseded by the every-evaluation requirement. The SUPER comparison is explicitly pinned to an older source snapshot, and the completed-handover report contains historical fixture wording.

Preserve history, but add a concise current decision index with active owner/status and move superseded entries into append-only historical sections. The report size currently makes it difficult to distinguish active policy from prior reasoning.

### A09 — stale test-only parameter input

Runtime tests still provide `navigation_runtime.planner_rate_hz=5.0` even though the product timing cleanup removed that parameter from the node. This does not re-enable runtime configurability, but it weakens the test's ability to detect stale callers and should be removed.

## 7. Historical runtime evidence, kept separate by map

All artifacts below predate HEAD `c86a3f61`; none qualifies the aggregate baseline.

### Open map — `sanity_open`

| Artifact | Waypoints observed | Mission | Cross-track p95 | Additional blocker |
|---|---:|---|---:|---|
| `external-mode-check-20260905T082033-554608` | 4/5 | `PAUSED_SAFETY_STOP` | 0.828 m | External Mode exited before completion |
| `external-mode-check-20260905T082244-557515` | 4/5 | `PAUSED_SAFETY_STOP` | 0.693 m | External Mode exited before completion |
| `external-mode-check-20260905T082431-559366` | 4/5 | `PAUSED_SAFETY_STOP` | 1.121 m | Simulation-clock validity violation |
| `external-mode-check-20260905T111005-688012` | 4/5 | `PAUSED_SAFETY_STOP` | 0.689 m | LiDAR timestamp/freshness violation |

These are four separate open-map runs. They show repeated non-completion and tracking/freshness problems, but they do not identify whether LIO, PX4 fusion, command transfer or planner dynamics is the initiating cause.

### Legacy occlusion map — not clutter C0

`external-mode-check-20260905T012101-305346` is the `occlusion_featured/legacy` scene. It produced no valid planner PVA, reported only goal index 0 with no accepted-waypoint coverage, ended `PAUSED_SAFETY_STOP`, and had cross-track p95 1.000 m. It must not be presented as clutter C0 evidence.

### Missing exact scenarios

- **C0 clutter near-obstacle:** no qualifying current artifact.
- **O0 manual/moving handover:** not automated and not qualified.
- **RESET:** no deterministic current qualification artifact.
- **Current HEAD:** no SITL artifact and no valid build manifest.

R13/R14 tooling fixes require re-analysis of retained raw artifacts before historical E5/H8 labels can be restored. PX4-versus-LIO attribution requires synchronized source-time comparison against Gazebo/flight ground truth. The user's working hypothesis that LIO is more accurate remains plausible but unproven for the exact failure.

## 8. Branch and work preservation status

| Branch | Head | Status relative to current baseline |
|---|---|---|
| `codex/runtime-evidence-for-analysis` | `c86a3f61` | Current aggregate baseline, clean and pushed |
| `codex/handover-regression-hardening-wip-20260905` | `0f691cf1` | Fully merged by `c86a3f61`; safe to retain temporarily for audit |
| `codex/completed-handover` | `b438f77b` | Production fix merged |
| `codex/current-body-production-proof-wip-20260905` | `2258b343` | Preserved, **not merged** because architecture remains WIP and earlier production-proof direction was rejected |
| `codex/pre-aggregate-body-wip-20260905` | `f801d860` | Preservation checkpoint only; superseded, do not merge |
| `codex/architecture-review-aggregate` | `be39176a` | Superseded by current branch after handover test merge |

Branches whose latest work predates today's accepted baseline are not treated as pending features. No finished non-body production fix from today's assigned agents remains unmerged.

## 9. Recommended order to reach a stable SITL diagnostic baseline

1. Remove unconditional body-oracle debug I/O.
2. Resolve the backend/mapping test dependency without introducing a production reverse dependency.
3. Replace one-shot current-body admission with a call-local current-body context recreated at every planning/safety evaluation; keep ROG sensor-only.
4. Unify body segment traversal/prefix semantics and remove mutable body policy from A*, corridor and planner fields.
5. Close the general PlanningWorker cancellation turnover race.
6. Run the clean canonical release build and all affected C++/Python tests; require a valid HEAD-bound build manifest.
7. Run one **diagnostic** open-map SITL baseline to verify command continuity, current-body revalidation and absence of debug-tail latency. Do not call this acceptance.
8. Re-analyze historical E5 and closed-loop artifacts with the corrected analyzers.
9. Run exact scenarios separately: O0 handover, C0 clutter, RESET, open/pass-through and structured obstacle maps. Report each map/run independently.
10. After a stable diagnostic baseline exists, collapse `ExecutionEpisode` and `ExecutionRecoveryState`, then split command transport from evidence and simplify STOP speed behavior in separate changes.

## 10. Acceptance gates that remain mandatory

Before any SITL qualification claim, require:

- clean canonical release build and valid build manifest tied to exact HEAD;
- all focused and aggregate tests green in the same dependency overlay;
- no test-only bypass or relaxed safety gate;
- mission and every waypoint completed;
- continuous P/V/A/J and bounded activation gaps;
- scenario-specific clearance, tracking and altitude evidence;
- PX4 mode/health/freshness and actual downstream command receipt;
- LIO/PX4/ground-truth timestamps aligned through an explicit clock mapping;
- repeated representative runs, with open and difficult maps reported separately;
- negative-path evidence showing OCCUPIED, stale identity, reset and unavailable BACKUP still fail closed.

Current status is therefore:

> **NOT READY FOR SITL QUALIFICATION. READY FOR A BOUNDED ARCHITECTURE CORRECTION PASS ON BUILD GRAPH, CURRENT-BODY OWNERSHIP, DEBUG I/O AND WORKER CANCELLATION.**
