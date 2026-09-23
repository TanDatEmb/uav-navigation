# State and authority conservation audit

**AUDIT_DELIVERY: PARTIAL.** Target design is a plausible way to remove repeated cross-owner mission/execution decisions, but it is not admitted for implementation. The declaration scan found 239 candidate behavioral fields in sixteen selected owner types; 214 still lack a proved disposition, and further owners/packages have not been exhaustively scanned. The 100% completion gate is therefore unmet. No product behavior was changed.

## Pinned baseline

| Key | Value |
|---|---|
| TARGET_REF | `origin/codex/close-proven-findings` fetched 2026-09-23 |
| TARGET_SHA | `7da3e97cb399c2e39d62cfe60213a45e8a92300e` |
| TARGET_TREE | `e7d2b56dbe4353810f029c9c6f0fe8991ebd63eb` |
| MAIN_SHA | `287b7b84cf4311e31d52ca04796223cc4efc1bc5` |
| MERGE_BASE | `287b7b84cf4311e31d52ca04796223cc4efc1bc5` |
| Original checkout | clean porcelain-v2; branch `codex/close-proven-findings`, same HEAD as TARGET |
| Submodules | `px4_msgs` `86d8239e`; `px4_ros2_interface_lib` `4a3370f0` |
| Audit branch | `codex/audit-state-authority-conservation-20260923` in separate worktree |

The target differs from main in 49 tracked paths, including safety ledger migration, runtime/episode/store and planner corrections. Prior audit artifacts were read from their own refs, not merged. Prior results are hypotheses unless rechecked against TARGET source.

## Findings, prioritized

1. **P1 CONDITIONAL — physical crossing and continuation readiness are coupled at the mission callback.** `MissionController::update` overwrites the previous measured sample before checking the PASS_THROUGH continuation and accepts only when both crossing and witness are true. A crossing can cease to be reconstructible after a later valid witness. The prior local counterexample supports this mechanism, while producer-to-PX4 reachability remains unproved. `mission_controller.cpp:363-371,569-579` is pinned in `EVIDENCE_INDEX.md`.
2. **P1 SPECIFICATION_GAP — “one-way braking until measured stop” is not a current system-wide invariant.** `MissionController::onTrajectory` explicitly changes `Braking` to `ExecutingWaypoint` when a successful non-stop trajectory arrives, before measured stop. Any target one-way rule would change behavior and needs a safety decision, scenario contract, and evidence. `ExecutionEpisode` has separate sampled-role and recovery logic; the two must not be conflated. See `mission_controller.cpp:144-165` and `execution_episode.hpp:146-173`.
3. **P1 CONDITIONAL — world freshness suspension has a cross-process lease consequence.** Runtime keeps an exact bundle but stops publication on stale world; adapter may independently expire the last admitted sample. This is fail-closed code behavior, not a proven liveness defect or warrant to bypass latest-world checks. A redesigned lease policy needs a measured timing budget and explicit stop transition. See `navigation_runtime_node.cpp:2983-3004,7704-7719`.
4. **P1 CONDITIONAL / SPECIFICATION_GAP — Hold retry can end before locally observed confirmation.** `onPx4HoldHandoverCompleted(Success)` clears pending/in-flight even if `px4_hold_confirmed_` remains false; the timer then has no retry condition. The pinned PX4 library delivers this callback from `ModeCompleted`, separately from VehicleStatus. Runtime ordering and intended disposition are unverified. See `PX4_HOLD_ORDERING.md`.
5. **P2 CONFIRMED_WITH_SCOPE — control authority is distributed across semantically different owners.** MissionController accepts waypoints, runtime desired goal and ExecutionEpisode manage intent/recovery, ExecutionTimelineStore owns active/staged bundles, NavigationMode admits command/freshness/frame and owns local mission/recovery mirrors, ModeExecutor manages Hold scheduling/status. Some are necessary independent physical/protocol facts; some look like replicated policy. `STATE_CONSERVATION_MATRIX.csv` intentionally leaves uncertain fields unresolved.
6. **P2 CONDITIONAL — `CandidateBundle` is not pure data.** Its evaluator and world-validator are `std::function` fields. The header describes them as immutable product callables, but the type cannot exclude captured mutable state or provide deterministic serialization/hash. Migration to pure data is warranted only if deterministic replay or cross-thread serialization is a requirement; it is not an immediate safety fix.

## Direct answers

| Question | Audit answer |
|---|---|
| Root cause | Source supports repeated semantic decisions and transaction checks across mission/runtime/adapter. No evidence attributes the whole observed bug series to a single root cause. Asynchrony and independent safety checks are inherent; duplicated policy and mirror identities are candidate accidental costs. |
| Complexity split | No defensible percentage or count yet. Independent world, measurement, execution, and PX4 protocol facts must remain; 214 scoped field dispositions and unscanned owners prevent a numeric split. |
| Keep independent facts | Desired intent versus active execution; active versus staged trajectory; latest versus certified world; source versus receive time; measured cursor versus accepted waypoint; trajectory endpoint versus measured stop; mission completion versus Hold request/API completion/VehicleStatus; MAIN and BACKUP certificates; localization reset and PX4 reset/frame state. |
| Suspected mirrors | Episode active IDs/generation versus committed bundle, adapter planner-recovery identity, terminal/suffix identity flags, runtime command-availability and suffix bits. These are candidates for derivation or protocol replacement, not approved DELETEs. |
| Bugs target could remove | A single reducer with typed events could eliminate cross-owner mission/execution dual writes and repeated stale-result tuple checks, if bounded delivery and adapter safety autonomy are retained. This is a design inference, unverified in product. |
| Potential capability loss | A scalar route progress loses branch/crossing provenance; a single Idle/Tracking/Braking/Stopped/Handover FSM loses orthogonal mission/PX4/lease facts; immediate brake on every world refresh gap loses certified continuation. |
| PX4 boundary | Keep independently received command and freshness/sequence/epoch, LIO and PX4 state/health/frame/reset witnesses, External Mode lifecycle, Hold request, command ACK, scheduled-mode completion, VehicleStatus confirmation and operator takeover. Remove planner policy mirrors only after equivalent explicit protocol is demonstrated. |
| Single writer | Recommended as an experiment for **mission/execution decisions**, not a single computation thread. Strongest counterexample is a burst of mapping/odom/result events delaying a 50 Hz command deadline or deadlocking on ROS executor/worker shutdown. A bounded priority queue and WCET/overflow evidence are prerequisites. |
| Anti-proliferation | Require the 12-question State Admission Test, one owner table row, explicit invalidation, a transition invariant, static detection of newly added persistent booleans/epochs, and review of semantic copies. The current extractor is a baseline aid, not a sufficient enforcement gate. |
| Migration value | Conditional. Smallest safe cut is a read-only shadow reducer fed by immutable events with no product authority, paired replay and latency/overflow measurements. Proceed only if it demonstrates fewer cross-owner transitions without losing facts. |

**Implementation gate:** finish whole-product field/writer/reader inventory, resolve the braking policy and PASS_THROUGH measurement semantics, specify Hold callback/status ordering, prove command lease and latest-world timing in representative replay/SITL distributions, specify queue overflow/priority/WCET, and pass equivalence plus fault-injection tests. No threshold changes or product rewrite follow from this audit.

## Evidence levels

`FACT_FROM_TARGET_CODE`: source paths and blob IDs in `EVIDENCE_INDEX.md`. `FACT_FROM_EXISTING_TEST`: targeted test source; tests were not rerun here. `FACT_FROM_AUDIT_ARTIFACT`: prior A/H0-H7/delta results, explicitly lower authority. `INFERENCE`: proposed ownership and likely bug-class reduction. `SPECIFICATION_GAP`: braking policy, cross-process recovery/lease budget, exact acceptance-on-late-witness semantics. `RUNTIME_UNVERIFIED`: target PX4 status ordering, end-to-end timing and flight behavior.
