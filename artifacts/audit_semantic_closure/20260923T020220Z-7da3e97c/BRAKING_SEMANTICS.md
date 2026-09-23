# Blocker B — braking and stop commitment

**Verdict: PARTIAL. Decision D — insufficient evidence to choose target policy.** Two distinguishable behaviors exist in TARGET source. The recoverable `onTrajectory` path is **not called by the repository's product source**; its exercised call sites are tests. It must not be promoted to a current flight behavior or silently erased from the API/test contract.

## Current paths

| Trigger/path | Physical/certificate condition | Recoverable before stop? | Owner and exit |
|---|---|---|---|
| MAIN+BACKUP reaches sampled BACKUP | committed bundle schedule, sampled role | runtime: no | `ExecutionEpisode::observeSampledSafetyRole`, `ExecutionRecoveryState::kTrackBackup`; measured endpoint and speed ≤0.15 m/s → `kStoppedRecovery` (`navigation_runtime_node.cpp:3792-3865,8200-8235`) |
| emergency candidate committed | emergency certificate and execution transaction | runtime: no | `kEmergencyBrake` → measured stop → `kStoppedRecovery`; certification failure → `kPx4Hold` (`execution_recovery_state.hpp:44-88`, runtime `:7259,3792-3865`) |
| failed replacement solve while retained MAIN still certified | exact retained bundle, lease and world validation | not a new brake | remain `kTrackMain` until sampled BACKUP/other safety event; HG-023 forbids dropping a valid certified command solely for solve failure (`runtime_safety_current.md:114`, runtime `:4537-4539`) |
| failed replacement after retained MAIN enters BACKUP | sampled BACKUP and certified suffix | runtime: no | sampled role commits `kTrackBackup`; moving nominal result cannot replace it (`navigation_runtime_node.cpp:3792-3865,8180-8235`) |
| product adapter receives accepted BACKUP/emergency sample | native command role and exact accepted command | does not itself enter `Braking` | `onNativeSafetyTrajectoryObserved()` clears generic mission readiness but retains terminal hold; runtime remains safety policy owner (`navigation_mode_node.cpp:1272-1287`, `mission_controller.cpp:270-285`) |
| terminal MAIN STOP | completed certified terminal MAIN plus measured stop | runtime restart after stop | `kTrackMain`→`kStoppedRecovery` via `kTerminalStopCompleted`; distinct from safety endpoint (`execution_recovery_state.hpp:65-83`, runtime `:3950-4030`) |
| adapter `onTrajectory` API/test safety STOP | successful safety role/STOP kind, finite nonnegative duration | yes in this API | MissionController enters `Braking`; a later successful non-STOP trajectory returns `ExecutingWaypoint` before measured stop (`mission_controller.cpp:145-227`); no product call site found |
| adapter braking STOP replacement | new successful STOP | remains braking | positive duration restarts its own end/confirmation timers; zero-duration refresh preserves them (`:165-195`) |
| adapter STOP failure or bad duration | failed/invalid metadata | no local tracking recovery | `Paused` and request POSCTL (`:119-140,195-206`) |
| adapter measured stop | trajectory ended, measured speed/position and confirmation | completion path | `Braking` waits on 0.15 m/s and confirmation; STOP and PASS differ (`:374-445`) |
| map recovery / new free corridor | successful non-STOP callback while Braking | adapter: yes | returns `ExecutingWaypoint`, resets braking timers (`:150-165`); source comment explicitly calls this rolling recovery |
| world recertification invalidates active trajectory | latest-world validation rejects retained bundle | no uncertified recovery | runtime fails closed to PX4 Hold if no certified brake; a valid recertification may resume an exactly identified suspended command (`navigation_runtime_node.cpp:1237-1305`) |
| planner failure during adapter execution | failed `onTrajectory` result | no local continuation | `Paused`, checkpoint kept, POSCTL requested (`mission_controller.cpp:229-245`); this is distinct from a certified runtime BACKUP suffix |

`FACT_FROM_TARGET_CODE` `CandidateRole::{Main,Backup,Emergency}` is a *sample/candidate role* and `CandidateBundleKind::{MainWithBackup,TerminalStop,BackupOnly,EmergencyBrake}` is bundle form (`candidate_bundle.hpp:18-25,147-155`). Neither alone is a global physical lifecycle. `ExecutionEpisode` stores sampled phase and recovery policy separately; the comment in `execution_episode.hpp:53-59` says they may differ. `nominalPlanningAllowed` excludes moving BACKUP/emergency (`execution_recovery_state.hpp:28-39`). Repository-wide `rg` finds **no product call** to `MissionController::onTrajectory`; `navigation_mode_node.cpp:1272-1287` calls `onNativeSafetyTrajectoryObserved()` or `onNativeTrajectoryReady()` for admitted commands. These native methods set readiness without entering/exiting `Braking` (`mission_controller.cpp:247-285`). The `onTrajectory` Braking path is source-real but test/API-only within this repository. A cross-layer runtime defect is therefore **not established**.

`FACT_FROM_EXISTING_TEST` `test_mission.cpp:1489-1547` exercises stop timers/replacement/failure; `test_execution_episode.cpp` and runtime recovery tests exercise one-way policy. Existing tests validate their local contracts, not end-to-end agreement. `FACT_FROM_TARGET_CODE` current safety ledger `docs/safety/runtime_safety_current.md` describes committed BACKUP/emergency retention and certified measured stop; it does not authorize silently changing the adapter rolling-recovery clause.

## Candidate policy split, not approved

`Tracking → RecoverableBraking → Tracking` requires fresh measured P/V/A, bounded tracking error, a continuous splice from the *current sampled brake state*, a current-world certificate, valid execution/command lease, remaining stopping distance and safety margin, and no reverse connector or policy commitment that forbids restart. A mere successful trajectory callback is insufficient.

`RecoverableBraking → CommittedStopping → Stopped` becomes irreversible before measured stop only on an explicit certified safety endpoint/one-way policy decision with exact bundle and localization identity. In that state, late MAIN, planner success or map update cannot restore tracking because the admitted safety suffix/stop is already the authority and the recovery state machine rejects `MainCommitted`. If product policy allows that replacement, the state is not committed and requires a different certificate/continuity proof.

- **Current behavior:** production runtime safety roles are one-way until measured stop; adapter `onTrajectory` API/test can recover from `Braking` on non-STOP success, with no repository product call site.
- **Desired target behavior:** unresolved. Candidate two-stage semantics preserves both only if cross-layer callback mapping and safety proof support it.
- **Information preserved:** current sampled role, bundle kind/certificate, measured P/V/A, stop progress/deadline, recovery policy, and one-way commitment event as distinct facts.
- **Information derivable:** `braking_recoverable` booleans from a typed execution variant; sampled role is *not* derivable from lifecycle alone.
- **Behavior intentionally lost:** none approved.
- **New risk:** permitting late nominal after actual committed BACKUP may break safe stopping; forbidding all recovery may degrade rolling liveness.
- **Required product tests:** inject late MAIN/new world/planner success before and after explicit commitment; verify continuity from measured brake P/V/A, margin, lease, map certificate, reverse connector rejection, measured stop and restart.

`SPECIFICATION_GAP`: whether public `onTrajectory` is an intended external integration contract and whether a future pre-commit recoverable state is required; no target product producer for it was found. `RUNTIME_UNVERIFIED`: liveness/performance and tail latency effect of requiring one-way stop for a future pre-commit uncertainty response.
