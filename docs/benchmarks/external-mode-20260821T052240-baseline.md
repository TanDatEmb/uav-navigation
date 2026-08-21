# External Mode baseline: long_three_pillars

This benchmark is the current navigation baseline for subsequent optimization.

- Artifact: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260821T052240-272841`
- Run verdict: `PASS`
- Mission outcome: `COMPLETE`; all five waypoint acceptance events observed
- Simulated duration: `78.664 s`; wall time: `98.415 s`
- Minimum truth clearance: `0.4395 m` at `long_three_pillar_02`; collisions: `0`
- Cross-track p95: `1.9609 m`; maximum allowed by this benchmark: `3.0 m`
- Ground-truth speed: p50 `0.6460 m/s`, p95 `1.9912 m/s`, maximum `2.5581 m/s`
- Planned speed: p50 `1.6794 m/s`, p95 `2.4982 m/s`, maximum `2.9594 m/s`
- Setpoint speed: p50 `1.3979 m/s`, p95 `2.1794 m/s`, maximum `2.3630 m/s`
- Trajectory boundary velocity jump: p50 `0.0042 m/s`, p95 `0.0680 m/s`, maximum `0.2076 m/s`
- Planner roles: nominal `301`, safety route `20`, terminal hold `13`
- LIO/navigation validity: tracking coverage `1.0`, navigation valid, residual gate passed

## Baseline code contract

The run corresponds to direct position/velocity/acceleration trajectory output
(`prefer_velocity_output: false`) and nominal trajectory progress rejection.
The later experimental safety-route terminal-velocity continuation is not part
of this baseline because it regressed the same mission into repeated safety-stop
states.

## Known limitations retained for the next iteration

- The 48 m mission leg exceeds the observed KnownFree horizon, so the planner
  still replans frequently and does not yet demonstrate the 5 m/s target.
- The benchmark's straight mission-line cross-track metric is useful as a
  regression signal, but it must be complemented by an obstacle-aware feasible
  corridor metric for forced detours.
- The map revision reached `691` for `691` accepted observations; the next
  iteration must separate harmless accumulated-map updates from changes that
  invalidate the committed trajectory.
