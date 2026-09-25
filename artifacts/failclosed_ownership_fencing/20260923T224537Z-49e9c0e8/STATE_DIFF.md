# State and authority diff

| Item | Before | After |
|---|---|---|
| Persistent behavioral state | Existing owners/latches | Unchanged |
| Ephemeral async provenance | Solve generation/start only | One in-flight `PlannerSolveFailureWitness` in the existing activity slot; exact `ExecutionAuthoritySnapshot` carried by result context |
| State lease identity | Immutable lease pointer/ingress sequence | Reused as exact causal comparison |
| Pending goal identity | Immutable goal pointer | Reused by `clearIfCurrent` |
| New persistent boolean | 0 | 0 |
| New manager or authority owner | 0 | 0 |

Mission progress remains owned by `MissionProgress`; desired planning by `DesiredPlanningIntent`; active/staged execution by `ExecutionAuthority`; latest world by `WorldModel`/its immutable publication store; downstream command admission, receive lease and Hold by PX4 Adapter. A solve witness is callback provenance and cannot publish or own a command.
