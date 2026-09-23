# Scoped state diff and admission

Counting scope: the 17 old private `MissionController` storage slots excluding its mutex, plus ten adapter mission-specific slots (`mission_`, `mission_controller_`, `mission_terminal_`, three suffix-handoff tuple members, two completed tuple members, `mission_complete_published_`, `last_goal_publish_ns_`). This is **27 scoped slots**, not the previous audit's 239-field whole-product denominator. The target scope has 12 `MissionProgress` members, five Core PX4-boundary/issued-command members, and three adapter boundary members: **20 slots**. Counts include immutable definitions and typed bundles as one slot each; they are not a safety metric.

| Disposition | AS-IS | AS-BUILT reason |
|---|---|---|
| DELETE from product path | `MissionController` object and adapter `mission_` | Core alone loads mission definition; legacy library retained, unlinked. |
| DELETE | adapter `mission_terminal_`, `mission_complete_published_` | Completion comes from Core accepted gate; adapter holds immutable COMPLETE receipt. |
| DELETE | adapter `last_completed_waypoint_index_`, `last_completed_request_id_` | Command replay checked by received goal/request/sample/session identity. |
| DELETE | adapter `safety_suffix_handoff_pending_`, waypoint/request tuple | Predecessor command remains Core execution authority until bounded successor admission or finite lease expiry. |
| DELETE | adapter `last_goal_publish_ns_`, goal publisher and mission timer | Adapter no longer publishes mission goals; local airborne first-command timer replaces timing need. |
| MOVE | mission definition, measured route progress, current/accepted gate, request writer | `MissionProgress` in Core, internal validated-goal transition. |
| DERIVE | `checkpoint_valid_`, `trajectory_ready_`, `terminal_hold_pending_` | Typed crossing, admitted continuation and measured stop windows. They are not one generic ready bool. |
| DERIVE | `MissionControllerState` mission advancement subset | Gate, accepted gate, active permission and completion predicate; execution/health/Hold stay separate. |
| NEW legitimate | crossing witness with before/after source samples and projections | Physical crossing survives later continuation; route/localization/gate identity fences it. |
| NEW legitimate | exact command admission receipt and bounded issued-command queue | Distinguishes a published Core command from one accepted by PX4 boundary. |
| NEW legitimate | PX4 mode activation ID on command and lifecycle status | Prevents old-session command replay after mode reentry. |
| NEW legitimate | terminal activation fence in Core | Delayed ACTIVE heartbeat cannot restart a terminal activation. |
| RETAIN PX4-local | accepted command, health/frame/reset/lease, planner recovery and Hold state | These are control-boundary facts, not waypoint policy. |

Target MissionProgress slots: immutable `mission_`; ordered `measured_route_`; typed `identity_`, `gate_`, `accepted_`; source sample `previous_` and `latest_`; optional `crossing_` and `continuation_`; STOP confirmation and hold timestamps; `active_` permission. Core boundary slots: fresh `mode_mission_boundary_`, terminal activation fence, last seen mode activation ID, applied activation ID, and issued-command queue. Adapter boundary slots: read-only completion receipt, mode activation counter, airborne first-command start time. Every listed target slot has one writer at its owner; no adapter slot increments a mission request.

`identity_.mission_id` is a stable typed copy of immutable `mission_.id`, not a second mutable authority. The other identity members (route revision and localization epoch) preserve independent invalidation information. `previous_` and `latest_` differ: previous creates ordered crossing pairs; latest supports the STOP measured speed/position gate when a delayed admission arrives. `active_` records permission inside the Core reducer; applied activation ID records which external lifecycle event was already consumed. Removing either loses an event-order distinction. STOP timestamps cannot be reconstructed from the current sample. The issued-command queue is bounded by each command's existing lease and one finite 16-entry cap; it is not a planner policy mirror.

Unchanged Core execution identity and planner fields are outside this scoped count. This cut does not claim a whole-product state reduction.
