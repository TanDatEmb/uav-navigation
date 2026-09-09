# SITL matrix — 2026-09-09

The default rows below are `GPS+LIO/default` runs. They use
`long_three_pillars_multiwaypoint`, seed `0`, world
`long_three_pillars_speed`, MAIN `V/A/J=3/2/4`, planner `10 Hz`, and PX4
custom checkout `deaff86e` (dirty/project-customized provenance). Each row has
its own authoritative report and clean source manifest. Requested speed is
mission intent; effective MAIN cap is `3.0 m/s`, with observed terminal
setpoint cap around `2.940 m/s`.

| Source | Artifact | Requested | Mode | Accepted waypoints | Complete | Collision | PVA failure | Measured speed p95 | Planning p50/p95/max ms | Deadline flags | Qualification |
|---|---|---:|---|---|---|---:|---:|---:|---:|---:|---|
| `c0f9d372` | [`163042-350998`](../../runtime/external-mode-check-20260908T163042-350998/report.json) | 5 | off | 0–2 | no | 0 | 0 | 3.0207 | 39.675/41.113/46.216 | 0 | no |
| `8ea66105` | [`171428-387681`](../../runtime/external-mode-check-20260908T171428-387681/report.json) | 5 | off | 0 | no | 0 | 0 | 2.6749 | 40.125/44.466/44.466 | 0 | no |
| `bb609142` | [`184002-436911`](../../runtime/external-mode-check-20260908T184002-436911/report.json) | 5 | off | 0–1 | no | 0 | 0 | 3.0303 | 35.117/58.552/74.737 | 0 | no |
| `8ea66105` | [`223501-451718`](../../runtime/external-mode-check-20260908T223501-451718/report.json) | 3 | adaptive | 0–4 | no | 0 | 0 | 3.0673 | 40.142/41.778/43.905 | 0 | no |
| `8ea66105` | [`223719-453312`](../../runtime/external-mode-check-20260908T223719-453312/report.json) | 3 | relaxed | 0 | no | 0 | 0 | 2.9107 | 40.245/72.950/72.950 | 0 | no |
| `bb609142` | [`223834-454788`](../../runtime/external-mode-check-20260908T223834-454788/report.json) | 3 | adaptive | 0–1 | no | 0 | 0 | 3.0827 | 25.470/42.851/43.159 | 0 | no |
| `bb609142` | [`224002-456318`](../../runtime/external-mode-check-20260908T224002-456318/report.json) | 3 | relaxed | 0–7 | no | 0 | 0 | 3.0722 | 40.124/80.071/80.211 | 5 | no |

The paired tracking modes are diagnostic-only and therefore not
qualification evidence. The latest `bb609142` relaxed row has five hard
deadline flags and remains incomplete; it must not be converted into a PASS
by changing the deadline or the tracking gate. The historical relaxed
completion and repeat remain documented in `SITL_CAMPAIGN_REPORT.md` as
separate diagnostic evidence.

## Explicit GPS-off EV profile

These rows are not part of the default GPS+LIO matrix. They use the explicit
`gps_off_ev_12mps` profile (`GPS_CTRL=0`, `EV_CTRL=15`, `HGT_REF=3`) and are
diagnostic-only. Its control envelope is `V/A/J=12/2/4`; the requested and
effective mission speeds are 3 m/s for cap3 and 5 m/s for cap5. The profile's
unavailable PX4 Hold interval is an environment limitation, not a product
defect or a reason to relax Hold validation.

| Profile | Source | Artifact | Requested | Bundles | Accepted waypoints | Collision | PVA command | Executable trajectories | Measured speed p95/max | Planning p50/p95/max ms | Deadline flags | Outcome |
|---|---|---|---:|---:|---:|---:|---|---|---:|---:|---:|---|
| GPS-off EV | `d4e34e5f` | [`004629-509425`](../../runtime/external-mode-check-20260909T004629-509425/report.json) | 3 | 8 | 1/9 | 0 | 922 success / 0 failure | 922/922 | 3.0044 / 3.0800 | 23.705/72.901/82.118 | 1 | `BLOCKED` |
| GPS-off EV | `d4e34e5f` | [`005203-511728`](../../runtime/external-mode-check-20260909T005203-511728/report.json) | 5 | 4 | 1/9 | 0 | 369 success / 379 failure | 368/369 | 1.5382 / 2.4411 | 30.919 / — / — | 0 | `FAIL` |

The cap5 row has a lidar timestamp/freshness/validity violation. Neither row
is Q1 qualification evidence.
