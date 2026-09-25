# Focused SITL and matched baseline

All sessions are under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/`. They used `map_seed=0`, `tracking_experiment_mode=off`, separate validated Release manifests, and external PX4 checkout `deaff86ee335dd697677bcfc2415a23878e1b895`. Migration source was `ccb373422642f27a8740b32ea4ab6062e433e0be`; baseline source was `7da3e97cb399c2e39d62cfe60213a45e8a92300e`. The external PX4 checkout had pre-existing dirt recorded in each session provenance. These are focused observations, not qualification.

| Session suffix (`external-mode-check-20260923T...`) | Source / profile | Core/legacy accepted indices | Mission complete | PX4 Hold | Finding |
|---|---|---|---|---|---|
| `093516-173925` | migration / open | Raw `/navigation/mission_progress` bag: `[0,1,2,3]`; old scenario report: `[]` | No | Yes | First harness revision ignored the new Core receipt. Planner later failed full immutable candidate revalidation near coincident final STOP. Bag had 654 exact admission messages. |
| `094018-180594` | migration / long_featured | `[0,1]` | No | Yes | Goal receipts `[0,1,2]`; incumbent later unavailable and command lease expired. |
| `094157-183822` | migration / pillar | `[0,1,2]` | No | Yes | STOP 0 and 1 accepted from measured speeds 0.047/0.103 m/s; PASS 2 accepted. Planner later lost an executable command. |
| `094408-187059` | migration / open | `[0,1]` | No | Yes | Requested failed-replan-after-handoff injection was **not armed** because `inject_failed_replan_once` was false; not evidence of injected recovery. |
| `094536-190305` | migration / open | `[0,1]` | No | Yes | `inject_failed_replan_once=true` and `inject_failed_replan_when_safe=true`, but source log has no injection-fired event; MAIN-safe eligibility was not reached. Injection outcome `NOT_EVALUABLE`. Normal BACKUP/stop path did execute. |
| `094657-193493` | migration / open | `[0]` | No | Yes | After adapter accepted commands, Core process 195872 received SIGSTOP for 650.132 ms at wall time 1790156856734469089 ns, then SIGCONT in a `finally` block. Adapter logged stale PVA command, requested Hold, and report observed PX4 Hold. A mapping unconsumed-cloud warning also occurred; this fault run is solely a lease-fence observation. |
| `095317-204843` | migration / long_featured | `[0]` | No | Yes | Repeat stopped on stale/invalid PVA command timestamp. |
| `095556-208505` | migration / long_featured | `[0,1]` | No | Yes | Repeat lost an executable command and handed over. |
| `095057-201248` | baseline / long_featured | `[0,1,2,3,4]` | Yes | Observed mission handover | Matched baseline run completed. Runner verdict `FAIL` only because the versioned qualification assessment was not PASS. |
| `095812-211924` | baseline / long_featured | `[0,1,2,3,4]` | Yes | Observed mission handover | Matched baseline repeat completed with the same qualification limitation. |

The migration normal mission completed **0/3** `long_featured` runs against **2/2** matched baseline runs. Its safety stop is correct when command authority expires, but the mission did not retain liveness. The three migration failures were not identical, so the causal link to a single changed line is unproven. This is a reproducible differential regression signal and blocks a successful migration cut. No gate was changed to force parity.

The project tracking policy currently treats simulated time with zero adaptive coefficients as a diagnostic tracking bypass even when the runner records mode `off`; the evaluator reports `config_mismatch` / `INCONCLUSIVE`. This is visible in `tracking_experiment.hpp` and the session logs. Thus the sessions establish only the stated semantic event sequences and lease response. They cannot establish strict tracking or flight qualification. The fault injection did not create runtime recovery evidence. Existing static and component proofs remain separate from these observations.
