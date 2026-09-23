# Core mission-progress authority cut (2026-09-23)

Implementation status: `IMPLEMENTED`, focused component tests passed;
runtime qualification remains pending. This branch moves the sole writer of measured route
progress, waypoint acceptance, request progression, successor goal intent and
mission completion into `navigation_runtime::MissionProgress`. Core loads the
immutable mission definition and enters its internal validated-goal transition;
the PX4 External Mode no longer loads a mission or publishes successor goals.
The legacy `MissionController` library/API remains compiled for compatibility
but has no adapter product call path.

Core retains an ordered, route/localization/gate-bound crossing witness across
callbacks. Crossing is discarded on route or localization identity change,
reverse movement, non-ordered samples or a source-sample gap above the existing
250 ms crossing limit. Acceptance still needs measured position and velocity.
STOP additionally requires continuous measured speed confirmation and waypoint
hold; a completed command alone is not mission completion. The route helper
now keeps the current segment through a near-parallel, nonadjacent projection
ambiguity within its existing backtrack tolerance. This prevents a closer
future leg from manufacturing route progress before the connecting turn.

Core-issued command publication is not execution readiness. The adapter emits
an exact `NavigationCommandAdmission` receipt only after its existing local
health, frame, lease, monotonic identity and tracking checks commit the
command. Core joins that receipt to the issued command, current mission gate,
localization epoch and mode activation before accepting a continuation
witness. A missing or late receipt cannot grant mission progress. The adapter
also rejects any command from an earlier External Mode activation, including
one still inside its 100 ms lease after reentry. The adapter
continues to own PX4 status, measured local state, command receive lease,
tracking response and Hold/takeover. Core's terminal progress receipt is
read-only presentation at the adapter boundary.

Core requires a fresh ACTIVE/airborne status source and receive observation
within the 200 ms mode-boundary lease before starting a mission gate or
consuming an admission receipt. This is a new cross-process liveness bound;
its scheduling tails require focused SITL evidence and are not flight-qualified.

The existing 100 ms adapter command lease, 500 ms runtime state freshness,
planner budgets, world/UNKNOWN rules, MAIN/BACKUP certificates and PX4 Hold
protocol remain unchanged. Unit and component evidence for this cut is recorded
under `artifacts/migrate_core_mission_progress/`; it is not SITL or flight
qualification.
