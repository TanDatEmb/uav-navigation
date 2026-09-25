# Planner warm-start authority audit

Inventory scope: all textual references in `planner.hpp` and `planner.cpp` (29 references including one comment and the owner declaration). Executable references fall into these groups:

| Sites | Use | Classification | C2 effect |
|---|---|---|---|
| `planner.hpp:221` | `CmdTraj` owner declaration | backend state | Existing optimizer/committed-command backend state. |
| `planner.hpp:333-357` | backup availability/start, generation, certificate, snapshot, generation and metadata accessors | `BACKEND_SYNCHRONIZATION` / execution snapshot API | These report or gate current backend candidate/command state. None authorizes emergency route correction. |
| `planner.cpp:848,878,910` | candidate commit check, generation snapshot and commit | `BACKEND_SYNCHRONIZATION` | Retained transactional backend commit. |
| `planner.cpp:1394,1545,2607,2643,2972,4428` | committed snapshots / generation checks in candidate/refinement flows | `BACKEND_SYNCHRONIZATION` | Retained; checks current backend-owned committed candidate. |
| `planner.cpp:1697,1701` | committed snapshot plus can-commit revalidation | `BACKEND_SYNCHRONIZATION` | Retained; prevents stale candidate commit. |
| `planner.cpp:2044,2075,2293,2322,2351` | committed trajectory visualization | `DIAGNOSTIC` | Retained, non-authoritative. |
| `planner.cpp:2247,2250,2257` | current command wall-time and backup-suffix state | `BACKEND_SYNCHRONIZATION`; suffix fact feeds current-command handling | Retained as current executable-command lifecycle information. It does not grant predecessor route exception. |
| `planner.cpp:2620,2624` | current committed position/yaw trajectories | command sampling/backend query | Retained for current backend command access. |
| `planner.cpp:3076,3077,3082` | committed guide position/yaw and duration during hot replanning | `OPTIMIZATION_HINT` / executable continuity input | Retained; the committed trajectory is a solve guide, not historical emergency authorization. |
| `planner.cpp:3103` | determine whether successor activation boundary enters BACKUP | `SAFETY_AUTHORIZATION` for active-command continuation | Retained as distinct policy: do not stage a successor after its anchor enters current BACKUP. It is not the PlanFromRest route-regression exception addressed by C2. |
| `planner.cpp:3122` | comment contrasting planner history endpoint with executable endpoint | comment only | Updated explanation, no state read. |

Result: mutable warm-start state no longer supplies the prior bundle kind/role/declared emergency endpoint for `emergency_bounded_correction`. Request-independent warm-start safety use for that specific authorization is zero. Other current-command and BACKUP boundary consumers remain by design and are not mislabeled as eliminated.
