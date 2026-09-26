# H2 — World-stale Hold

Run: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260926T005456-384877` (`long_featured`, seed 0, injected mapping-only 700 ms scan outage, tracking off).

This run was captured before the final request-timeout retry change. It is evidence for the observed World-stale → stationary setpoint → AUTO_LOITER path, but not a runtime validation of the final 250 ms retry-on-missing-callback behavior.

An attempted rerun at `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260926T011209-398644` was rejected during setup before startup: the authoritative build manifest no longer matched the worktree fingerprint after evidence documents had been edited, and the mapping-only gate therefore had no evidence. This is an invalid setup attempt, not a product-runtime failure; it is not counted as a fault run. The workspace was rebuilt before the next attempt.

## Final-source rerun

Run: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260926T011400-401857`, same map, seed, 700 ms mapping-only fault, C0-SW scope, and tracking off. The run started successfully with no runtime setup failures. The isolated gate passed: 7 mapping scans dropped from the scheduled 700 ms interval (`22.672 s`–`23.372 s`) while odometry/health/clock counters continued; forwarding resumed at source stamp `23.400 s`.

Observed sequence:

- Navigation status entered `PAUSED / SAFETY_STOP / RECOVERY_HOLD` at sim `23.204 s`.
- PX4 input trace continued to emit `velocity_hold`; observed traces had no active command identity.
- The scenario observer recorded external-mode exit at `23.228 s`; the same observation reported `nav_state=4` (`AUTO_LOITER`), `executor_in_charge=1`, `failsafe=false`.
- The scenario ended `PAUSED_SAFETY_STOP`, not `COMPLETE`. Runtime verdict was `BLOCKED` because the injected safety scenario had no mission completion/waypoint acceptance. C0-SW remained `NOT_EVALUABLE` because product outcome and the required producer-owned handover phase witness were absent. This is an expected fault-scenario report, not a nominal pass.

This validates the mapping-only fault → Core suspension → stationary PX4 input → observed AUTO_LOITER integration path against the final compiled source. It does not prove schedule callback ordering or loss-free C0-SW request/callback/status reduction: the mode-exit correlation is observer-derived, and callback phase remains unrecorded.

Raw manifest:

| File | Size | SHA-256 |
|---|---:|---|
| `scenario.jsonl` | 3,184,018 | `01d59eabd28c4655e2bfa6d5f2652915b3f894b54475fc67ad719f967dc6643d` |
| `world_observation_gate.jsonl` | 7,881 | `df8414279480fbc33b20e76a720c9197f598232f71164bf9f0b0167ca2b12ccb` |
| `execution_timeline.jsonl` | 11,320,093 | `137f063bdf4bb938c778fd38436289a6519b2293b8786ae486506f5b85e7abbd` |
| `report.json` | 2,595,043 | `16a5298a35053d0754482a59917a50ce3c936754b84a7f8a2095c2caa201dee7` |
| `metadata.json` | 248,884 | `c8d6735893a2752e29b36891c5244c72c16222fa2006bc22fec1a7c8c68fabb9` |
| `runtime.json` | 240,579 | `059f372766db265f19435f40c9d4cc94a52a006c8b22ebf7686f78ce37aae5af` |

Observed sequence:

- Fault interval: sim time `23.391999999 s`–`24.091999999 s`; gate dropped 7 RegisteredScan messages and restored forwarding at `24.12 s`.
- Core published `NavigationModeStatus=PAUSED`, reason `SAFETY_STOP` at `23.912 s` after the last ready command identity was request 1 / bundle 2 / sample 188.
- PX4 input traces 198–201 were `velocity_hold`; they did not carry the old moving command identity.
- Product log reports `planner backend PVA command stale; safety hold then handover to PX4 Hold`, followed by the Hold request.
- External mode exit was observed at sim `23.984 s`; `VehicleStatus.nav_state=4` (`AUTO_LOITER` in the pinned `px4_msgs`) was observed in the same record with `executor_in_charge=1`, `failsafe=false`.

This proves the injected map-source outage reached Core suspension, stationary setpoints, and observed PX4 Loiter/Hold in this run. It does not prove request/callback/VehicleStatus correlation in C0-SW: the request event in `scenario.jsonl` is observer-derived from external-mode exit, and the executor callback phase is not separately reduced. The overall mission report is `BLOCKED` because the expected mission COMPLETE and waypoint acceptance `[0,1,2,3,4]` were not observed; this was the intended safety-fault scenario, not a nominal completion run.
