# Short World stale to recovery SITL

Session: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260925T091243-356218`. Profile `long_featured`, seed 0, C0_SW_V1, tracking requested/effective off. Fault gate result `PASS`; mission outcome `COMPLETE`, all five waypoint events were accepted.

| Event | ROS simulation time | Exact identity/evidence |
|---|---:|---|
| Last pre-fault committed world | 22.652 s publication | W generation 2, revision 187, source 22.600 s |
| Mapping-only fault scheduled | 22.616–23.046 s | 430 ms sim-time; input scans continue at gate |
| Exact active suspension | 23.104 s | E generation 2; timeline 194→195; W2/rev187/source22.600; reason `SOURCE_STALE`; computed source age 504 ms |
| Last command admission with old certificate | 23.088 s | sample 192, E generation 2, W2/rev187 |
| Mapping forwarding restored | gate observes 23.120 s ROS time | first forwarded source stamp 23.100 s |
| New world committed | 23.160 s | W2/rev188/source23.100 s; active validation path 2 (full immutable validation) |
| Exact execution recertified/resumed | 23.160 s | same E generation 2; timeline 195→196→197; disposition `RECERTIFIED_AND_RESUMED` |
| First post-recovery command admission | 23.168 s | sample 193, E generation 2, W2/rev188 |

No new command sample was emitted between the suspension at 23.104 s and the successful full recertification/resume at 23.160 s. The adapter admission gap from sample 192 at 23.088 s to sample 193 at 23.168 s was 80 ms; the post-suspension wait to the first new admission was 64 ms. The existing 100 ms adapter receive lease was not extended. The adapter stayed in External Mode and the mission completed.

This directly shows that forwarding restoration/fresh scan alone did not resume exposure: W2/rev188 had to publish and exact E2 had to pass full validation before `WORLD_COMMAND_RESUMED`. There was no replacement execution: active generation remained 2 across the entire sequence.

The source-stale interval narrowly crossed the existing 500 ms freshness boundary: 504 ms at first suspension. Fault duration is 430 ms, while the mapping source stamp gap was 500 ms due to the 100 ms producer cadence. No threshold was changed.

## Supplemental 420 ms run after evaluator correction

The earlier summary treated session `external-mode-check-20260925T090136-348595` as a no-stale boundary control. That label was based on its original `report.json`, whose world reducer had `required=false` because the report's nonempty base config masked the session-specific requirements. Re-running the current evaluator directly on the immutable session inputs yields `required=true`, `RESOLVED`, 0 required unresolved/conflicts/reference gaps, and 0 drops. The event stream contains three `WORLD_COMMAND_SUSPENDED` observations for E2/W2-rev191, then a committed/recertified W2-rev192 (source stamp 23.600 s) and `WORLD_COMMAND_RESUMED` for the same E2. The gate dropped only RegisteredScan source stamps 23.200–23.500 s and resumed forwarding at 23.600 s. This run is therefore supplemental stale/recovery evidence, not a boundary control. The exact selected witnesses and reducer output are in `EVALUATOR_RECHECK_420.json`; its captured report is not overwritten.

This supplemental session has a distinct build source-content digest (`c3bb8e…`) from the primary 430 ms recovery and final nominal/control cohort (`d87cd…`). It is not pooled into same-build repeatability counts. Both report the same navigation Git HEAD/tree and the same PX4 checkout/binary; the manifest records each build content digest separately.
