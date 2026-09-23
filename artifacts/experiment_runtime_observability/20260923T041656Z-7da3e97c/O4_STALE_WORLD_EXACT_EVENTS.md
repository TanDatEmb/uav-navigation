# O4 registered-scan input fault and downstream lease

The fault gate runs **outside** runtime on the registered-scan input topic. It records source sequence, ROS time and host steady time for `FAULT_START`/`FAULT_END`, while odometry and External Mode continue. Product world freshness (500 ms) and adapter command receive lease (100 ms) were unchanged. `O4_STALE_WORLD_EVENTS.csv` links selected exact event records to normalized audit files; the full audit stream and gate JSONL remain in the evidence index. All W bags have a monotone, nonzero shared `/clock` stream (sample counts W1–W5: 17,642; 5,260; 6,783; 3,789; 7,079). These are individual diagnostic SITL observations, not qualification.

| Run | Gate ROS interval (s) | Scans dropped | First world stale / suspend (s) | Adapter receive lease expires (s) | First post-lease AUTO_LOITER status (s) | First fresh world after fault (s) | Runtime resume after Hold |
|---|---:|---:|---:|---:|---:|---:|---:|
| W1 control 200 ms | 19.916–20.116 | 2 | none / none | none | not applicable | continuous | not applicable |
| W2 800 ms | 20.420–21.220 | 8 | 20.808 / 20.808 | 20.896 | 20.920 | 21.252 | 0 observed |
| W3 550 ms | 24.616–25.180 | 6 | 24.912 / 24.912 | 25.004 | 25.188 | 25.248 | 0 observed |
| W4 3000 ms | 20.216–23.220 | 30 | 20.604 / 20.604 | 20.700 | 20.728 | 23.356 | 0 observed |
| W5 5000 ms | 25.620–30.620 | 49 | 26.016 / 26.016 | 26.116 | 26.144 | 30.652 | 0 observed |

`WORLD_TRANSITION` phase 3/4 counts were W1 **0/0**, W2 **28/28**, W3 **21/21**, W4 **172/172**, W5 **290/290**. Each W2–W5 bag has exactly one `LEASE_DISPOSITION` with product reason 1 (100 ms receive lease), followed by a local Hold request/attempt and a fresh `VehicleStatus` `AUTO_LOITER=4` after the lease event. The W2 scenario report stopped at the safety-block condition and its own `px4_hold_observed=false`; the longer independent raw/audit recorder subsequently captured AUTO_LOITER. That distinction must not be collapsed.

Fresh registered scans resumed in W2–W5. Each has `WORLD_SNAPSHOT_READY`, recert start/end and world commit after fault end. The recertified world callbacks reported **active bundle generation 0** in these post-Hold intervals; no `WORLD_TRANSITION` phase 8 `COMMAND_RESUME` and no later `COMMAND_PUBLISH` were observed. W3 is the narrow return: the first fresh world follows the observed AUTO_LOITER status by only **60 ms**. W5 is the long post-handover return. In these runs, the observed outcome for “can runtime resume an exact bundle after downstream handover?” is **NO, observed product fencing**. The source's resume predicate additionally requires retained bundle/goal identity, valid lease and no failure latch; this is source support, not a proof over every interleaving.

`RECERT_END.flags` distinguishes callback paths: bit 1 fast/unchanged certificate, bit 2 full swept certificate, and bit 0 pending retained. W2–W5's first recert callback after fault end has flags **0** and active bundle **0**; it is world processing without an active trajectory recertification. The normal C trace has phase-6 flags 4 (full path) on 339 callbacks and flags 5 (full path with pending retained) on 20; this prevents mislabeling post-Hold world updates as a 6 ms active-bundle certificate renewal.

The runtime still emits world recertification work after downstream Hold. That work must not be interpreted as regranting command authority. The recorder did not prove a general temporal safety bound or a future product policy. W2–W5 were `BLOCKED` by intentional safety stop, and W1's mission completed but evaluator `FAIL` remains unrelated to this input fault and is not flight qualification.
