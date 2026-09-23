# Authority diff

| Semantic fact/action | Baseline product writer | As-built writer | Boundary observation |
|---|---|---|---|
| Mission definition/policy | PX4 `NavigationMode` + `MissionController` | Core `MissionProgress` constructor | Adapter receives no mission YAML. |
| Measured route cursor/crossing | PX4 `MissionController` | Core `MissionProgress::observeMeasured` | Propagated state source stamp, sequence, epoch. |
| Accepted waypoint/request advancement | PX4 `MissionController::update/advanceRequestId` | Core `MissionProgress::accept` | Adapter may only cache command identity for local replay checks. |
| Initial/successor goal | PX4 goal publisher | Core `applyMissionDecisionLocked` → common `applyValidatedGoalLocked` | GOAL receipt is presentation only. |
| Mission completion | PX4 `MissionController` and adapter bool publication | Core accepted final gate and `/navigation/mission_complete` | Adapter consumes immutable COMPLETE receipt to request mode completion. |
| Candidate/command authorization | Core execution store | Core execution store, unchanged | Adapter separately admits or rejects received command. |
| Local command/setpoint/PX4 Hold | PX4 adapter | PX4 adapter, unchanged | Core observes lifecycle status/command admission. |
| Latest world | WorldModel | WorldModel, unchanged | Active execution owns its certificate. |

Source guard: `tools/check_mission_authority_cut.py` fails if the product adapter mentions `MissionController`, keeps a goal publisher or mission-file parameter, or links the legacy contract library. `rg` of in-repository non-test source found `MissionController::onTrajectory()` definitions/overloads but no external product call. Library source remains for compatibility; external consumers are not proven absent.

Command identity is producer-owned `{mission, localization epoch, goal epoch, mode activation, waypoint, request, bundle, sample, world identity, finite lease}`. Adapter can reject old activation or nonmonotonic tuples, but cannot mint a request or accept a waypoint. Admission receipt carries only exact identity, not mission policy. A false or stale receipt cannot match Core's current issued-command queue and gate.
