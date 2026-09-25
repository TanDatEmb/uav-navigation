# Representative exact OptimizationFailed timeline — I1

Raw session: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260924T013310-690969`; source `47a5c05e9b36feb558e5e2ecd3646cc36157613d`. Times below are recorder/rosbag observer nanoseconds and ROS simulation time; recorder time is not PX4 firmware execution time. Mission `external_mode_long_featured_low_altitude`, localization epoch `70309892499745`, desired revision 4/request 3, active request 2/generation 4, owner lineage 4, world generation 2/revision 347 at the fault, planner cycle 149.

| Event | Bag observer ns | ROS sim ns | Evidence |
| --- | ---: | ---: | --- |
| MissionProgress accepts waypoint index 1 / desired request 3 | 1790213654299970780 | 39960000000 | mission progress event 0, accepted=true |
| Last predecessor adapter admission before fault | 1790213654350059962 | 40000000000 | request 2, generation 4, sample 800 |
| `FAULT_INJECTION_ARMED` | 1790213654365206451 | 40012000000 | exact 2→3 semantic trigger; cycle 149 |
| `FAULT_INJECTION_APPLIED` | 1790213654365229968 | 40012000000 | original backend `kSuccess` 0 → exact `kOptimizationFailed` 6; status only |
| HG-023 retained-command decision | 1790213654366592734 | 40012000000 | purpose 0; exact owner/request current; world/state/anchor/suffix valid; disposition 8; command remains available |
| Actual planner classifier trace | 1790213654367152647 | 40012000000 | `planner_result_after_injection=6`, `planner_disposition=4` (`RetainCommittedCommand`) |
| First predecessor command after fault | 1790213654370030079 | 40015999999 | request 2, generation 4, sample 801 |
| First predecessor adapter admission after fault | 1790213654370232688 | 40016000000 | request 2, generation 4, sample 801 |
| Next normal planner result | next cycle 150 in planner trace | 40088000000 | backend success 0; successor staged, bundle generation 6 |
| Last predecessor adapter admission | 1790213654930208505 | 40464000000 | request 2, generation 4, sample 829 |
| First successor command | 1790213654950112686 | 40479999999 | request 3, generation 6, sample 830 |
| First successor adapter admission | 1790213654950274716 | 40480000000 | request 3, generation 6, sample 830 |

Fault→successor adapter admission: **585.045 ms**. There are **29 predecessor adapter admissions after the fault**. Maximum predecessor/Core publication interval is **20.200 ms**; maximum predecessor/adapter admission interval **20.174 ms**; exact last-predecessor→first-successor adapter gap **20.066 ms**. There is no late predecessor admission after successor cutover, no recorded Hold request, and final mission accepted `[0,1,2,3,4]` and completed. `/fmu/in/trajectory_setpoint` was recorded, but that observation does not prove firmware consumption.
