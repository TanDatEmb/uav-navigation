# Baseline timeline (pinned `7da3e97c`)

Fresh seed-0 `long_featured` session: `external-mode-check-20260923T103615-258561`. Same `tracking=off` diagnostic configuration as the traced pre-fix comparison; validated Release manifest; PX4 checkout `deaff86e`. Five waypoints were accepted and mission COMPLETE was observed. Runner `FAIL` reflects versioned qualification ineligibility, not mission outcome. The earlier migration artifact retains two separate baseline `tracking=off` completions (`095057-201248`, `095812-211924`). The first three repair completions used `relaxed` tracking and are supplementary; matched `off` repair repetitions are reported separately.

Times below are simulation seconds. H2 is `/navigation/mission_progress`/`waypoint_accepted`; H16/H12 are last predecessor/first successor `/navigation/navigation_command` observed as `pva_command`; H14 is the adapter's first successor `px4_input_trace` tracking-setpoint publication. The trace does not prove PX4 firmware consumed that setpoint.

| New request | H2 mission acceptance | H16 last predecessor command | H12 first successor command | H14 first successor PX4 input | Adapter setpoint gap |
|---:|---:|---:|---:|---:|---:|
| 2 | 23.624 | 24.032 | 24.048 | 24.060 | 16 ms |
| 3 | 35.252 | 35.716 | 35.720 | 35.744 | 28 ms |
| 4 | 44.996 | 45.392 | 45.408 | 45.420 | 16 ms |
| 5 | 55.916 | 56.024 | 56.048 | 56.060 | 16 ms |

The baseline continued predecessor delivery after acceptance until successor cutover. Physical crossing observation and candidate commit timestamps are not emitted as separate events in this retained baseline trace; their exact instants are **not observed**.
