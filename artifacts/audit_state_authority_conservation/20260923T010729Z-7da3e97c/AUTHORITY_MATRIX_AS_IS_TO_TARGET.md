# Authority migration matrix

| AS-IS owner | Decision/fact | Candidate target owner | Migration condition |
|---|---|---|---|
| MissionController | accepted waypoint and mission completion | MissionProgress reducer | preserve measured cursor versus accepted gate; PX4 receives explicit completion event |
| NavigationRuntimeNode goal fields | desired mission intent and epoch | MissionIntent in NavigationCore | serialize with accepted mission event; retain older executing identity |
| ExecutionEpisode | execution phase/recovery/availability | ExecutionAuthority reducer | prove reachability and sum-type equivalence; no silent latch loss |
| ExecutionTimelineStore | active/staged bundle and future activation | ExecutionAuthority store, sole commit API | retain immutable pointer, generation, predecessor and world checks |
| PendingGoalHandoffOwner | deferred newer goal | MissionIntent queue/slot | exact consume after replacement commit; no second pending owner |
| WorldSnapshotStore | latest world evidence | WorldModel owner | independent immutable publication and provenance |
| PlanningWorker | pending/active solve, cancellation | PlanningWorker | result only; never execution commit |
| NavigationMode | local command/state/frame admission | PX4Boundary | independently reject invalid producer, no planner recovery policy mirror |
| NavigationModeExecutor | Hold request/command ACK/ModeCompleted/VehicleStatus | PX4Boundary protocol owner | preserve all four observations and deactivation ordering |

The table is a **candidate** mapping. `NavigationCore` would be sole writer for mission/execution decisions while workers and PX4 retain distinct independent facts. It must not become sole thread for computations or erase PX4 veto authority.
