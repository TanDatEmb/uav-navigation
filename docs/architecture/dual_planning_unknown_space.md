# Unknown-space and backup planning status

This document records the current planner backend configuration. It is not a proposal
for a missing mapping/planning package.

## Current behavior

`navigation_runtime` constructs one planner backend planner that always produces
the nominal MAIN path together with a certified BACKUP suffix when the geometry
and evidence permit it. The runtime keeps both trajectory roles inside the
planner backend and performs the final command/safety checks before publishing.
There is no runtime switch that disables the backup contract. Visualization is
independently disabled in the canonical runtime configuration.

Mission files may declare an `unknown_policy`; it is translated to the typed
MAIN policy while BACKUP remains known-free. Neither setting by itself proves
that a trajectory is safe. Acceptance must show a valid command, a usable backup or emergency
brake when required, fresh propagated odometry, and no collision/failsafe.

## Safety interpretation

- A geometric route is not mission success.
- A safety stop is not waypoint acceptance.
- Unknown cells must not be described as known free in reports.
- A negative/no-path scenario is expected to fail closed.
- Real-flight collision geometry remains a configuration and validation
  prerequisite, not an inference from simulator truth.

Main/backup selection, verifier failures, and fail-closed behavior are part of
every runtime execution. They are not a separate flight permission mode.

## Required evidence for future changes

Any change that permits nominal execution through unknown space must add
tests and runtime evidence for swept-volume clearance, backup availability,
map/input freshness, stop distance, planner latency, and trajectory
continuity. The evidence belongs in `report.json`/`REPORT.html`; no second
Markdown report or visualization topic should be introduced.
