# Focused desired-intent cleanup SITL results

## Provenance and interpretation

The behavior-bearing source for all runs was clean commit `473b2121817b2ba248c0c6eb40f9cfed89a3c2d3`, built as an authoritative Release with source fingerprint `8ffce50ad2bd3adefcc0e056e1f990b5589b7369eb23a271e25ce6d0fedfacef` and manifest SHA256 `22af33048922a94ecfcfa8149f17909a419b027616b31b260fbeb5c32e9f9b3b`. Product code was last changed at `c9006768`. Subsequent commits only record these results; they do not alter the executable source. The three nominal runs and faults all record that same clean source in `metadata.json`.

All runs used `external_mode`, `long_featured`, seed 0, `tracking=off`, dynamics override `off`, unchanged gates, PX4 checkout `deaff86ee335dd697677bcfc2415a23878e1b895` and PX4 binary SHA256 `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`. Session paths below are under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/`.

The pinned unified-owner reference source `ba9c15a1` has matched 3/3 nominal completion and admission gap `19.894/20.030/20.093 ms` over 12 transitions (prior `SITL_RESULTS.md` and `PERFORMANCE.md`). It is reference evidence only.

## Run matrix

| Run | Session suffix | Intervention | Mission acceptance | Boundary result |
|---|---|---|---|---|
| Nominal 1 | `151450-509084` | None | COMPLETE, `[0,1,2,3,4]` | No stale PVA, unexpected Hold request, identity or continuity rejection. One isolated pre-admission `valid=0 command_present=1` rejection; following commands admitted, no lease event. |
| Nominal 2 | `151832-512307` | None | COMPLETE, `[0,1,2,3,4]` | No stale PVA, unexpected Hold request or command rejection. |
| Nominal 3 | `152036-515456` | None | COMPLETE, `[0,1,2,3,4]` | No stale PVA, unexpected Hold request or command rejection. |
| BACKUP fault | `152307-518741` | Existing `inject_failed_replan_repeated=True` | COMPLETE, `[0,1,2,3,4]` | 26 structured injected planner failures; requests 2/3/4 sampled BACKUP then restarted MAIN after measured low-speed state. |
| Injection setup check | `152624-522113` | Attempted rosbag-count trigger, no fault injected | COMPLETE, `[0,1,2,3,4]` | Rosbag did not expose live admission count; Core exited before trigger. Excluded from fault evidence and 3-run nominal denominator. |
| Core-pause fault | `152851-525435` | SIGSTOP Core PID 527769 after 1,802 observed PVA commands; SIGCONT after 350.094 ms | Expected incomplete `[0,1,2]` | Adapter logged stale PVA and requested PX4 Hold; report observed Hold and `PAUSED_SAFETY_STOP`. |

The first three runs are **three consecutive matched mission completions**. `report.json` acceptance reasons are empty for each. The runner process reports overall `FAIL` because its versioned qualification gate is ineligible; this is distinct from the focused mission outcome `COMPLETE`. No flight qualification is claimed.

`px4_hold_observed=true` appears in the nominal reports after normal terminal completion. Their `px4_hold_handover_requested_sim_ns` and trigger are null, their outcome is `COMPLETE`, and adapter logs contain no unexpected Hold request. The Core-pause report instead records `px4_hold_handover_requested_sim_ns=48100000000`, trigger `unexpected_external_mode_exit`, `px4_hold_observed=true`, and outcome `PAUSED_SAFETY_STOP`; adapter log wall time `1790177400.733844690` says PVA command stale and `1790177400.733887396` requests Hold. This distinguishes planned terminal handover from the fault response.

Nominal adapter final metrics report `trajectory_rejected=1/0/0`, `stale_state_failures=0/0/0`, and maximum recorded setpoint update gap `20/20/16 ms`. The single rejection is the same class as the prior cut's unresolved pre-validation sample; this run does not prove its exact cause. There is no nominal lease expiration, command identity reject, or candidate continuity reject in the retained logs/trace. The exact 12 handoff measurements are in `HANDOFF_MEASUREMENTS.csv`; min/median/max adapter predecessor-to-successor admission gap is `19.958/20.052/20.169 ms`, versus the 100 ms unchanged receive lease. These are recorder/admission observations, not PX4 firmware consumption timestamps.

Terminal STOP is directly witnessed in nominal 1: `/navigation/mission_progress` accepted terminal waypoint 4 with event 1, ROS stamp 75.860 s, position error `0.2474 m`, and measured speed `0.0982 m/s`; `/navigation/mission_complete` and the report confirm completion. In the injected BACKUP run, scenario trace has 26 `injected_replan_failure=1` planner records, 311 BACKUP (`analytic_sample_role=1`) command samples, and MAIN restart after BACKUP on requests 2, 3 and 4. The nearest preceding propagated odometry samples were 8 ms old and measured speeds `0.0866`, `0.0447`, and `0.0979 m/s`. Terminal waypoint 4 was accepted at `0.0978 m/s`. This is focused BACKUP/restart evidence; it does not qualify the emergency path.

The isolated world-source stale case and emergency injection remain boundary evidence debts. The Core-pause fault is a command-heartbeat fence, not a world-freshness test. No safety limit was relaxed.
