# Execution state reachability before mutation

Source-derived local transition table from `ExecutionEpisode` (`execution_episode.hpp:65-251`) and recovery reducer (`execution_recovery_state.hpp:8-94`). Runtime reachability still requires the caller gates cited in the target owner design.

| Event/path | Phase | Recovery | Safety owner | Restart | Exposure | Evidence/condition |
|---|---|---|---|---|---|---|
| reset/new non-retained goal | InitialHold | InitialHold | no | no | unavailable | `reset`, `beginGoal` |
| MAIN commit | TrackingMain | TrackMain or prior one-way state | no | preserved until caller clears | available | `commandCommitted`; caller must forbid nominal commit from one-way state |
| BACKUP commit | TrackingBackup | TrackBackup | yes | possible | available | `commandCommitted` |
| Emergency commit | TrackingBackup | EmergencyBrake | yes | possible | available | `commandCommitted` |
| retained safety suffix before BACKUP switch | TrackingMain | TrackMain or safety recovery | yes | possible | available | `observeRetainedCommand(..., true)`; sample role is still MAIN |
| sampled BACKUP/emergency | TrackingBackup | TrackBackup/EmergencyBrake | yes | possible | available | `observeSampledSafetyRole` |
| analytic terminal hold, measured stop pending | StoppedHold | TrackMain/TrackBackup/EmergencyBrake | no | possible | available | `stoppedHold` changes phase, not recovery |
| measured certified stop | phase may still be StoppedHold or tracking | StoppedRecovery | independent | possible | active may remain | `applyRecoveryEvent(CertifiedStopObserved)` |
| PlanFromRest retry in STOPPED_HOLD | StoppedHold | StoppedRecovery | no | yes | available | `requestRestartFromRest`; `stoppedHold` preserves request |
| world freshness suspension | prior phase | prior recovery | prior safety owner | prior request | suspended | `suspendCommand`, active bundle remains stored |
| PX4 Hold fail closed | Px4Hold | Px4Hold | no | no | failed | `failClosed`; observations cannot resurrect |

`phase` and `recovery_state` are not reducible to each other: a terminal MAIN may be sampled as STOPPED_HOLD before measured stop, and a safety suffix may be owned while the sampled role is MAIN. `restart_from_rest` is also independent of STOPPED_HOLD. `command_available` and `failure_latched` encode three observed exposure dispositions: available `(true,false)`, suspended/unavailable `(false,false)`, failed `(false,true)`; `(true,true)` is not produced by Episode transitions. Preserve an explicit initial/no-command distinction if a product predicate requires it. No boolean-only replacement is proposed.
