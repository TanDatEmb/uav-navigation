# Target critique

## Single writer: value and counterexample

A single writer for **mission/execution decisions** can remove cross-object goal/Episode/Store transitions and let result admission use one context key. It also makes replay order explicit. It does not make mapping, polynomial solve, world sweep, PX4 state ingestion or diagnostics single-threaded. A serial reducer that handles map recertification or arbitrary callback bursts can miss the command heartbeat. A bounded queue must distinguish replaceable latest observations from non-droppable resets, stop/Hold and cutover events; overflow must fail closed with a source-stamped trace. Priority, CPU scheduling, exception containment and shutdown ordering are implementation gates. No WCET or burst distribution is available, so this architecture remains conditional.

## No mega-FSM

`Idle/Tracking/Braking/Stopped/Handover` loses independent staged bundle, world certificate, desired intent, measured gate, source/receive ages, PX4 API operation and status. A small execution sum type can replace mutually exclusive phase/recovery combinations only after reachability analysis. Physical/protocol state remains orthogonal and owned once.

## PX4 adapter autonomy

The receiver should reject replayed/reordered/expired/wrong-frame or old-epoch references without consulting planner state. It must retain command receive time, sample sequence, state/health source and receive times, PX4 reset counters, frame alignment, External Mode ownership and Hold lifecycle. Moving `planner_recovery_pending_` or suffix/request mirrors out is possible only if a typed protocol event expresses their remaining safety meaning and current command lease; otherwise `NavigationMode` would lose a local fail-closed decision.

`tracking_experiment` remains diagnostic-only under the current safety contract. TARGET loader still derives suppression from `use_sim_time` and a disabled tracking gate (`tracking_experiment.hpp:58-88`), so reducer migration must not silently promote a simulated diagnostic policy to product authority. A live profile is unverified.

## CandidateBundle verdict: REFACTOR LATER

`CandidateBundle` stores evaluator and world-validator `std::function` (`candidate_bundle.hpp:159-162`). The header claims product-owned immutable callables, but the type does not enforce capture immutability, serialization or stable hashing. Pure data plus pure validation functions would improve replay and provenance. It is **REQUIRED FOR TARGET** only if the target promises deterministic cross-process replay/hash or long-lived serialized candidates; that requirement has not been approved. Do not rewrite it as the first migration cut.

## Safety invariants

Target admission must preserve current contract: immutable world identity, strict BACKUP UNKNOWN/OOM policy, moving MAIN positive BACKUP, continuous corridor and V/A/J/flatness/swept-world gates, active/staged future cutover, no pre-admission mutation, latest-world recertification, measured acceptance, source/receive distinction, localization reset, independent PX4 veto, requested versus confirmed Hold, stale result rejection and active-command retention after failed solve. `runtime_safety_current.md:40-69,84-130` is the current authority. The proposed one-way moving safety transition conflicts with a TARGET adapter branch (`mission_controller.cpp:150-165`); label it `SPECIFICATION_GAP`, not a conserved current invariant.

## Quantitative budget (proposal, not measured gain)

| Metric | Scoped AS-IS evidence | Target gate |
|---|---:|---|
| behavioral candidate fields | 240 across sixteen selected types (60 bool-like, 26 optional-like, 31 atomic-like); 215 unresolved | every old fact mapped, no absolute count target |
| scoped owner types | 16 in AS-IS model; seven decision/protocol owners, other state/evidence/worker/trace owners | one mission/execution decision writer; independent world/PX4/worker owners retained |
| control-message fields | 73 non-constant fields in `CONTROL_PROTOCOL_FIELD_AUDIT.csv`: 6 control, 13 preliminary safety, 7 provenance, 47 diagnostic candidates | zero diagnostic fields read by admission; preserve all safety witnesses |
| cross-owner transitions | present in goal/episode/store and mission/adapter handoffs; exact count unverified | each decision has one commit owner, protocol events explicit |
| manual identity comparisons / control locks / nested depth | 52 selected identity-helper name occurrences, 16 mutex declarations in scanned classes; TARGET publication has three nested runtime locks plus Store lock; actual site totals unverified | lower counts proven by AST/static metrics and no deadline regression |

If measured owner/transition/lock/comparison counts do not improve and latency tails worsen, keep improving existing architecture instead of migrating.
