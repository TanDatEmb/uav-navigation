# Unknown-space and backup planning status

This document records the current SUPER configuration. It is not a proposal
for a missing mapping/planning package.

## Current behavior

`navigation_runtime` constructs one SUPER planner with backup trajectory
generation enabled (`backup_traj_en: true`). The runtime keeps the main and
backup trajectory ownership inside SUPER and performs the final command/safety
checks before publishing. `frontend_in_known_free`, map visualization, and
debug visualization are disabled in the canonical runtime configuration.

Mission files may declare an `unknown_policy`, and `DUAL_PLANNING=1` enables a
simulation experiment. Neither setting by itself proves that a trajectory is
safe. Acceptance must show a valid command, a usable backup or emergency
brake when required, fresh propagated odometry, and no collision/failsafe.

## Safety interpretation

- A geometric route is not mission success.
- A safety stop is not waypoint acceptance.
- Unknown cells must not be described as known free in reports.
- A negative/no-path scenario is expected to fail closed.
- Real-flight collision geometry remains a configuration and validation
  prerequisite, not an inference from simulator truth.

The simulation-only dual-planning experiment is started explicitly:

```bash
DUAL_PLANNING=1 make external-mode-check
```

This flag is for measuring main/backup selection, verifier failures, and
fail-closed behavior. It is not a real-flight permission.

## Required evidence for future changes

Any change that permits nominal execution through unknown space must add
tests and runtime evidence for swept-volume clearance, backup availability,
map/input freshness, stop distance, planner latency, and trajectory
continuity. The evidence belongs in `report.json`/`REPORT.html`; no second
Markdown report or visualization topic should be introduced.
