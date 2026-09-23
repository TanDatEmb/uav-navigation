# Authority Transition Width V3

`W_t` is the number of independently owned mutable objects that must change to complete a stated transition. Lower bounds below come from V2 source writer proof; this branch's traces clarify protocol scope but do not reconstruct a whole-product writer graph. Target counts are design constraints, **not achieved metrics**. PX4 local/external width is inherent distributed protocol and must not be forced to one object.

| Completion boundary | AS-IS `W_t` | V3 expected `W_t` if admitted | New evidence/remaining gap |
|---|---|---|---|
| new goal desired intent accepted | ≥3, upper unknown | 2 candidate (mission decision + execution admission) | No new writer proof; V2 lower bound remains. |
| PASS mission acceptance | ≥2, upper unknown | 1 mission-owned measured/accepted transaction, plus read-only execution witness | One delayed-readiness episode proves independent facts; exact callback linearization missing. |
| successor cutover | ≥2, upper unknown | 1 execution transaction | No active cutover trace with full identity. |
| safety brake commitment | ≥2, upper unknown | 1 execution transaction + adapter received observation | Native one-way source proof; pre-commit recovery still policy D. |
| measured stop/restart | ≥2, upper unknown | 1 measured witness + 1 execution decision | No new synchronized stop trace. |
| world recertification | ≥2, upper unknown | 2 necessary owners (WorldModel + Execution) | Diagnostic revalidation duration exists; event-paired commit absent. |
| PX4 Hold request | ≥2 (local adapter + pinned ScheduledMode), PX4 command adds external actor | local protocol owner + PX4 external = 2 necessary domains | Command/ACK not recorded in TARGET runs. |
| PX4 Hold confirmation | ≥2, upper unknown (adapter status cache + PX4) | local received witness + PX4 external = 2 necessary domains | `AUTO_LOITER,executor=1` is valid run-specific mode occupancy; no charge-loss step is necessary. |

Highest proven AS-IS lower bound is **≥3** at new goal; no global max is established. A target `W=1` for distributed Hold would falsely erase PX4 authority, while accidental local mirrors still require future reduction and proof.
