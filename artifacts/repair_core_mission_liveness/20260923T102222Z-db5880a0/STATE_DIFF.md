# Incremental state/authority diff from `db5880a0`

| Category | Result |
|---|---|
| Persistent state added | None. |
| Persistent state removed | None. |
| Derived state | New pure `sameExecutionPublicationIdentity()` derives whether an in-flight sample still belongs to the current executing goal, command epoch and localization epoch. It stores nothing. |
| Publication authority | Final transaction and stale fallback now use executing identity plus the existing exact bundle pointer/world/state/certificate/lease checks. Desired MissionProgress gate can advance without revoking predecessor execution. |
| Mode boundary | Existing `mission_activation_applied_` and `mode_mission_boundary_` derive established session identity; no new latch. A 200 ms status freshness check is used only before initial activation, while explicit non-ACTIVE status still resets the session. |
| Mission authority | Still solely `MissionProgress` in Core. Static guard forbids adapter `MissionController`, goal publisher, mission YAML, waypoint/request writer and terminal policy latch. |
| Adapter authority | Local command/state/health/tracking/receive-lease and Hold protocol unchanged. |
| Threshold/config | No numeric threshold, lease, planner deadline, world policy, MAIN/BACKUP certificate or UNKNOWN-policy change. |

This branch does not merge `ExecutionEpisode` with `ExecutionTimelineStore` and does not redesign Hold callbacks. H4–H8 logs and retained-result diagnostics are observation only; they create no state.
