# Stationary safety stream

`NavigationMode::safetyStopNavigation()` / `failNavigation()` capture the latest finite measured position when available, invalidate the accepted moving `NavigationCommand`, latch the handover request, publish the existing terminal mode status, and request executor handover. `updateSetpoint()` then emits the existing stationary position/zero-velocity setpoint from the latched position (or the existing fallback when no position is available) while the navigation mode remains active.

The moving command cannot be restored by a later planner command after `failure_reported_`/`handover_requested_` is set; command callbacks and boundary logic check those terminal flags. The stream ends when PX4 deactivates the navigation mode. This branch did not change its state freshness, position-hold semantics, or command lease.

The stream has no explicit maximum duration. That is safer than silently dropping the only setpoint while PX4 has not confirmed Hold, but it means the stream can remain an alternate flight-control source if the downstream transition never occurs. A maximum-duration response requires an approved product policy and is an open finding; this branch does not invent one.
