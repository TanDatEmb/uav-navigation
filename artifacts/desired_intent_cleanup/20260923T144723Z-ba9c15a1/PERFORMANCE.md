# Focused handoff and timing observations

All new-cut observations come from the three clean-manifest nominal sessions in `SITL_RESULTS.md`. `HANDOFF_MEASUREMENTS.csv` preserves all 12 exact request transitions. The method is the pinned `artifacts/execution_authority_unification/20260923T123424Z-8ffa14e5/analyze_sitl.py`: deserialize the versioned ROS bags, match exact adapter `NavigationCommandAdmission` request changes, and summarize periodic `DiagnosticArray` values. No timing threshold or planner deadline was changed.

| Boundary, 12 transitions | New cut min / median / max (ms) | Unified-owner reference (ms) |
|---|---|---|
| Adapter predecessor-to-successor admission | `19.958 / 20.052 / 20.169` | `19.894 / 20.030 / 20.093` |
| Core command publication | `19.952 / 20.035 / 20.128` | Prior CSV, same method |
| Nearest recorded PX4 setpoint | `19.961 / 20.002 / 20.095` | Prior CSV, same approximation |

The maximum new-cut admission gap is 0.076 ms above the prior 20.093 ms maximum and remains far below the unchanged 100 ms adapter receive lease. Three nominal adapter final metrics reported maximum setpoint update gaps of `20 / 20 / 16 ms`; no stale-PVA or unexpected Hold request was logged during handoff. The nearest setpoint timestamp is an approximation because PX4's setpoint message has no request identity; it does not prove PX4 firmware consumption time.

Periodic diagnostic observations below are `p50/p95/p99/max` in microseconds. Sample counts are `601 / 487 / 570` per nominal session for each named metric.

| Session | Command store publish | Transport publish | Planning scheduling gap | Command transition lock wait | Execution-owner publication lock wait |
|---|---|---|---|---|---|
| `151450-509084` | `40/55/75/106` | `35/48/62/72` | `0/62/399990/400130` | `0/0/0/12` | `0/0/0/0` |
| `151832-512307` | `42/65/72.28/283` | `37/58/63/280` | `1/113.7/400030.54/400147` | `0/0/0/8` | `0/0/0/0` |
| `152036-515456` | `40/56.55/65.31/135` | `36/50/55.31/62` | `0/100/399999.72/400240` | `0/0/0/9` | `0/0/0/0` |

The prior unified-owner cut measured transport-publish maxima `64 / 149 / 801 us`, with its 801 us tail unresolved. The new cut's maxima are `72 / 280 / 62 us`. These small, focused sets neither establish an improvement nor prove a worst-case deadline. The roughly 400 ms planning scheduling spikes were also present in the reference and reflect the existing scheduling pattern; no new planning budget is inferred. The owner lock-wait diagnostic records the latest publication attempt at each diagnostics tick and may miss transient peaks.
