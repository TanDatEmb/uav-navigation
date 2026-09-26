# PX4 authority handover: as-is source audit

Base: `7e0b8508781f68ecbd18d15d40129e019108f3e7`; pinned interface library `4a3370f084ac6f1ef001a4afa2b007845ffd0837`; external PX4 checkout `deaff86ee335dd697677bcfc2415a23878e1b895` (dirty as recorded in `BASE_PROVENANCE.md`). Source review only; no PX4 runtime conclusion is claimed here.

## Product path

- `NavigationMode::safetyStopNavigation()` and `failNavigation()` snapshot the latest finite odometry position when available, latch `failure_reported_` and `handover_requested_`, invalidate the cached `NavigationCommand`, publish PAUSED/FAILED status, then call the executor's handover callback. See `navigation_mode_node.cpp:2013-2060`.
- `NavigationMode::updateSetpoint()` gives the terminal failure branch priority and publishes a zero-velocity stationary setpoint at `safety_hold_position_`; if absent, the `handover_requested_` path falls back to the latest odometry position or zero velocity. It returns without sampling the old command. See `navigation_mode_node.cpp:2148-2177,2252-2291`.
- `NavigationMode::onActivate()` increments `mode_activation_id_`, invalidates `navigation_command_`, resets prior completion/safety-hold/recovery state and reports ACTIVE. `onDeactivate()` marks the activation terminal, invalidates the command, clears recovery/anchor state, and reports operator takeover unless a terminal status already exists. See `navigation_mode_node.cpp:821-905`.
- `NavigationModeExecutor` owns the Hold request/retry flags and subscribes to `VehicleStatus`; the handover timer checks every 50 ms. Current Hold retries are separated by 250 ms after a callback failure, but there is no attempt ceiling or total deadline. See `navigation_mode_node.cpp:2585-2700`.
- `onVehicleStatus()` sets `px4_hold_confirmed_` only when `nav_state == NAVIGATION_STATE_AUTO_LOITER`, and clears pending/in-flight then. Repeated AUTO_LOITER messages are idempotent for those booleans. However, any non-AUTO_LOITER status overwrites the confirmation bit, without recording whether the transition is a new navigation activation, operator mode change, or failsafe. See `navigation_mode_node.cpp:2603-2612`.
- `onActivate()` resets handover state and schedules the owned mode. `onDeactivate(reason)` clears handover state; it logs `FailsafeActivated` versus `mode_exit` but does not retain or publish typed takeover evidence. See `navigation_mode_node.cpp:2623-2634,2688-2700`.

## Callback semantics defect

Current `onPx4HoldHandoverCompleted()` treats both `Result::Success` and `Result::Deactivated` as completed Hold and clears pending/in-flight, despite `px4_hold_confirmed_` being separately derived from VehicleStatus. This conflates callback disposition with actual PX4 authority state. It also treats Deactivated as a successful outcome although the pinned library defines it as cancellation of the scheduled operation.

## Stationary-stream limits

The old moving command is invalidated before handover. The stationary setpoint is then emitted synchronously from the still-active NavigationMode update loop. A finite stored measured position is preferred; if unavailable, the current code can emit zero-velocity without position. The stream stops when NavigationMode deactivates. There is no code-level maximum duration; Hold scheduling retries indefinitely at a 250 ms minimum retry interval following failures. Freshness of the source used for the captured hold position is not rechecked in this terminal branch.

This audit identifies an unbounded retry/stream interval but does not invent a retry ceiling or stale-position policy. Those require evidence/policy analysis before changing behavior.

## Initial classification

- Hold request, callback, and AUTO_LOITER status are three distinct facts.
- `scheduleMode()` success is a VehicleCommand ACK accepted result, not an AUTO_LOITER witness.
- `ModeCompleted` is a scheduled-mode lifecycle callback, not a direct PX4 current-mode report.
- `DeactivateReason::FailsafeActivated` identifies executor deactivation while `VehicleStatus.failsafe` is true; `Other` alone does not prove operator takeover.
- A stale callback can presently mutate flags for whichever handover is current because the callback captures `this` and the mutable failure flag, not an episode token.
- No new authority owner is indicated; the executor remains the existing handover orchestrator.
