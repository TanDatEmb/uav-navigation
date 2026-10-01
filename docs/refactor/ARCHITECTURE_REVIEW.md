# Architecture review & refactor program — uav-navigation

Owner: chief architect review (2026-09-28). Baseline: `main @ 7e0b850`.
Every other branch is **historical reference only**: never merge or cherry-pick
from it. Beta scope: **SITL only** (ADR-016).

This file is the reference every work package (WP) prompt points to. IDs used
in the WPs: V1–V7 (boundary violations), RC1–RC6 (root causes), R-01…R-11
(risk register, `risk_register_20260928.md`).

## 1. The current architecture (as observed, not as documented)

| Tier | Package | src LOC | Notes |
|---|---|---|---|
| L4 judge/governance | `tools/runtime/*.py` | 32k Py | PASS/FAIL judge; `reduce_lifecycle` CCN 288 |
| | `tools/check_*.py` | 744 | 11 regex architecture guards |
| L3 ROS shell | `fast_lio_ros` | 5.4k | LIO node |
| | `navigation_runtime` | 14.7k | node .cpp 9 910 lines, `runCycle` CCN 776 |
| | `px4_navigation_external_mode` | 5.6k | `updateSetpoint` CCN 119 |
| | `px4_odometry_bridge` | 3.0k | 2 executables |
| L2 domain impl | `fast_lio_core` | 8.2k | ROS-free |
| | `navigation_mapping` | 3.0k | `MappingActor` |
| | `navigation_planning_backend` | 36.7k | 44% of product code; `Planner` 69 fields / 5 mutexes |
| | `navigation_execution` | 1.8k | `ExecutionAuthority` |
| L1 contract | `navigation_planning` | 1.7k | request/outcome/`CandidateBundle` |
| | `navigation_mission` | 1.0k | depends on world_model (V2) |
| | `navigation_world_model` | 0.8k | no own tests |
| | `navigation_contracts` | 12 ROS msgs | |
| | `navigation_common` | 0.25k | time helpers |
| L0 vendor | ikfom, ikd_tree, rog_map | 13.4k | rog_map exports `navigation_math` |

Runtime processes (product): `fast_lio_node`, `navigation_runtime_node`,
`px4_navigation_external_mode_node`, `px4_odometry_bridge_external_node`.
SITL-only: `gz_visibility_bridge` (product planner depends on its topic),
`px4_odometry_bridge_node` (started by runner), Python scenario observer.

### Boundary violations on main

- **V1** Runtime shell includes L2 implementation directly (`navigation_runtime_node.cpp:26` → `planner_facade.hpp`; also `navigation_mapping`).
- **V2** `navigation_mission` (contract) depends on `navigation_world_model` (`route_progress.hpp:11`, `mission.hpp:10`) for goal tolerances and `UnknownPolicy`.
- **V3** Backend exports generic include roots `utils/ traj_opt/ data_structure/ planner_core/ path_search/`; product math `navigation_math` lives in `rog_map_vendor`; undeclared dependency on `navigation_common`.
- **V4** `CandidateBundle` (contract) carries `std::function` `world_validator`/`evaluator` captured by the backend (`planner.cpp:1137`). Runtime "validation" executes the generator's own code.
- **V5** `px4_odometry_bridge` gates PX4 EV publication on string `/lio/diagnostics` KV, although typed `/lio/health` exists and docs call diagnostics observability-only.
- **V6** Adapter cannot see `PlanningTimingContract` (no dependency) and pins 0.10/0.20/5.0 s with literals (`navigation_mode_node.cpp:255-258`).
- **V7** Python judge re-derives C++ semantics: enum ordinals, 4 waypoint-acceptance parsers, collision truth, vehicle radius.

### Root causes

- **RC1 Shell owns domain logic.** One node: 5 threads, 9 mutexes, 57 atomics, 158 members, 38 `planner_fsm` predicates, 12 fault-injection params, 270 diagnostic fields, wall timers.
- **RC2 Generator certifies itself.** World/corridor/dynamics validators live in the backend; bundle holds closures.
- **RC3 No single configuration/constant source.** 0.15 m/s ×6, 0.5 s ×5, 0.35 m ×4, pinned params, cross-process consistency checked only by the Python runner.
- **RC4 Evidence is stringly typed.** 11 `DiagnosticStatus.name` channels parsed by the judge.
- **RC5 Estimator→PX4 trust boundary is blurred.** Discontinuity is guessed downstream (jump latch reseeds after a gap, R-02).
- **RC6 Architecture is kept by process, not structure.** Regex guards, 22k-line ledger, doc drift (`ExecutionTimelineStore` does not exist), CI does not build C++.

### Keep (these are right)

ROS-free `fast_lio_core`; integer-ns time (ADR-004); immutable COW world
snapshots with a single `MappingActor` owner; `ExecutionAuthority` single owner
with pure `transitionExecutionRecovery`; immutable `PlanningRequest`;
fail-closed `NOT_EVALUABLE` semantics; the safety-ledger discipline.

## 2. Target architecture

### Tiers and modules (names are binding for WP mapping)

Tier A — **contracts, data only** (no `std::function`, no pointers into impl, serializable):

| Module | Content | Main source today |
|---|---|---|
| `nav_core_types` | epochs, goal/request ids, bundle generation, clock-domain-tagged `TimestampNs`, canonical frame ids, finite checks, small math | `navigation_common`, `frame_ids.hpp`, `navigation_math` |
| `nav_safety_profile` | every threshold, generated from one YAML into C++ header + Python module + ROS param schema, with a content hash | scattered (see WP-A6) |
| `nav_world_contract` | `WorldModelView` interface, `WorldSnapshotIdentity`, `UnknownPolicy`, goal tolerances, `CurrentBodySupport` data | `navigation_world_model` |
| `nav_plan_contract` | `PlanningRequest`, `PlanningOutcome`, `CandidateBundle` as pure data (polynomial pieces, roles, identities, certificate refs), `KinematicState`, `ExecutionAnchor` | `navigation_planning` |
| `nav_mission_contract` | mission schema, `ImmutableRouteSnapshot`, route geometry; **no world dependency** | `navigation_mission` |
| `navigation_contracts` | product ROS msgs (existing) | — |
| `nav_evidence_msgs` | typed evidence ROS msgs, explicit enum values, per-producer sequence (ADR-015) | new |

Tier B — **pure domain cores** (ROS-free, deterministic given inputs, replay-testable):

| Module | Role |
|---|---|
| `lio_core` | estimator (`fast_lio_core`) + declares public-frame discontinuities |
| `nav_world` | `MappingActor`, snapshot store, ROG adapter |
| `nav_planner` | trajectory **generator**; untrusted; request-scoped `SolveContext` |
| `nav_certifier` | **new**; world / corridor-plane / dynamics+flatness / swept-world / stop certificates; depends only on tier A |
| `nav_execution` | execution reducer: sole command owner, lifecycle, anchors, command sampling |
| `nav_mission` | mission-progress reducer (`MissionProgress`) |
| `nav_planning_policy` | renewal/scheduling reducer (desired intent, `planner_fsm` predicates, supervisor, baseline refinement) |
| `px4_setpoint_core` | adapter logic: admission guard, tracking adapter, velocity-only continuity, setpoint pipeline, mode reducer |
| `odom_bridge_core` | ENU→NED conversion, timestamp policy, continuity check |

Tier C — **thin ROS shells** (I/O, ROS-time timers, parameter loading, no decisions):
`lio_node`, `nav_core_node`, `px4_adapter_node`, `odom_bridge_node`.

Tier D — **qualification, outside product binaries**:
`sitl_harness` (runner, scenario, Gazebo truth, fault plugins),
`nav_judge` (pure reducers + table-driven verdict policy),
`evidence_decoders` (generated from `nav_evidence_msgs`).

### Hard rules (enforced by package/CMake structure and types, not by regex)

1. Dependencies point downward only; shells see cores through interfaces/factories; implementation headers are not installed.
2. Contracts are data only.
3. The planner is untrusted: a bundle executes only after `nav_certifier` (which must not link `nav_planner`) accepts it.
4. Each concept has one owning reducer: `State × Event → (State, Effects[])`.
5. One decision thread per process; compute lanes take immutable inputs and return immutable results through queues; timers on ROS time; steady time only for receive/latency.
6. All thresholds come from `nav_safety_profile`; each process publishes the profile hash; mismatch ⇒ do not arm.
7. Evidence is typed msgs decoded by generated decoders; the judge defines no semantics of its own.
8. Fault injection and probes exist only in the SITL build; product binaries do not link them.

## 3. Phases

| Phase | Scope | Behavior change | Exit gate |
|---|---|---|---|
| P0 | CI build (ROS Jazzy), SITL baseline corpus (mcap bags + distributions), golden planner snapshots, dead-code removal, judge hotfix R-01 | none (R-01 is judge-only) | CI green on a clean clone; baseline frozen with manifest |
| P1 | tier-A contracts + `nav_safety_profile` codegen, hashes, remove pinned literals | none (values unchanged) | hash equal in 4 processes; 0 threshold literals outside the profile |
| P2 | `nav_evidence_msgs`, dual emit, new judge on typed evidence | none (product) | new judge = old judge on corpus except explained R-xx |
| P3 | bundle as data + `nav_certifier`, shadow mode | only at the boundary, shadow first | 0 unexplained disagreements; certify tail within the 80 ms budget |
| P4 | `nav_core_node` decomposition into reducers + lanes | none (decision order preserved) | deterministic event-log replay; package-wide TSan pass; ≤2 mutexes in shell |
| P5 | planner `SolveContext`, stateless stages, namespaced includes | none | facade tests via `plan(request)`; p99 solve ≤ baseline |
| P6 | adapter pipeline, bridge on typed health, estimator discontinuity (fixes R-02) | yes (R-02) — separate commit + ledger | SITL matrix distribution not worse than baseline; jump-across-gap test |

Before P3 starts: WP-A1…A6 reviewed and the resulting design specs approved.
