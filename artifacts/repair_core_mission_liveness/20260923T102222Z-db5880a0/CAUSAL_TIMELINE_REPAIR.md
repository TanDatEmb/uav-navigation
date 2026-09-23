# Repair timeline

Three consecutive validated Release, **tracking-matched `off`**, seed-0 `long_featured` sessions: `112040-309344`, `112251-312566`, `112456-315691`. Each accepted `[0,1,2,3,4]` and observed mission COMPLETE. No H4/H5 transition invoked `failClosedLocked()`. H4/H5 retained the exact predecessor execution epoch/generation until successor publication. Three earlier `relaxed` sessions (`105937-284324`, `110442-292258`, `110706-295798`) also completed but are supplementary because baseline/pre-fix used `off`.

Representative matched `off` session `112040-309344`, simulation seconds:

| New request | H2/H4 acceptance | H5 predecessor retained | H6 planner submit | H8 solve return | H16 last predecessor publication | H12 first successor publication | H14 first successor PX4 input |
|---:|---:|---|---:|---:|---:|---:|---:|
| 2 | 23.980 | generation 2 | 23.980 | 23.992 | 24.044 | 24.060 | 24.064 |
| 3 | 37.860 | generation 7 | 37.900 | 37.912 | 37.964 | 37.980 | 37.988 |
| 4 | 47.660 | generation 10 | 47.660 | 47.676 | 48.044 | 48.060 | 48.064 |
| 5 | 57.940 | generation 12 | 57.980 | 57.984 | 58.044 | 58.060 | 58.064 |

For the **12 matched `off`** successful handoffs, `HANDOFF_TIMING.csv` gives observed intervals. Min/median/max (ms): Core H4→planner submit `0/40/64`; submit→solve start `0/0/0` at 4 ms sim-clock resolution; solve start→return `4/12/16`; predecessor→successor sample `16/16/16`; Core publication gap `4/16/32`; recorder command→exact admission receipt `0.015/0.081/0.137`; first Core successor publication→PX4 input `4/12/28`; adapter tracking-setpoint publication gap `4/16/28`. These are diagnostic distributions from a small sample, not product deadlines. The setpoint gap is below the unchanged 100 ms receive lease. The first successor command and first adapter setpoint are tied by request/generation/sample identity. `NavigationCommandAdmission` is an exact adapter-local admission witness, not a PX4 firmware ACK or evidence of physical setpoint consumption. Recorder receive deltas are not physical transport latency.

H0 crossing observation and H1 continuation creation are proved by MissionProgress source/model and acceptance but lack a separate runtime timestamp. H3 is created within the H2/H4 Core decision call; no independent timestamp exists. H7 solve start is logged and equals H6 at the simulation-clock resolution in these runs. H9 candidate admission, H10 exact atomic store cutover and H11 sampler callback entry are bounded by H8 and first successor command, but do not have separate emitted timestamps. H13 adapter receive callback entry is bounded by H12 and H15/H14; H15 exact receipts are in the rosbag. H17 lease expiry and H18 Hold are **not applicable** in these nominal handoffs. These unobserved sub-events are not substituted with inferred point timestamps. A `scenario.jsonl` waypoint receipt can lag the same Core H4 event by 4 ms; timing uses H4 as the Core acceptance-side clock, not the later observer callback.
