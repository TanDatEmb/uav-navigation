# Desired intent cleanup performance

## Status

`PENDING_NEW_CUT_RUNS`

No timing value from a SITL session built from the desired-intent cleanup HEAD
is available yet. The reference below is pinned context, not a measurement of
this cut.

## Pinned reference

Reference product source: `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`, whose
product source is identical to `2e4f7a0f1e09f4d72afed23686e26dc173150dd3`.
The [prior performance report](../../execution_authority_unification/20260923T123424Z-8ffa14e5/PERFORMANCE.md)
records three matched nominal sessions and 12 exact adapter-admission request
transitions for that source. Its same-method min/median/max handoff gap is
`19.894/20.030/20.093 ms`. The associated
[handoff measurements](../../execution_authority_unification/20260923T123424Z-8ffa14e5/HANDOFF_MEASUREMENTS.csv)
and diagnostic timing observations are the comparison reference.

The prior report identifies a single `801 us` maximum transport-publish sample
in one nominal session; its cause was not established. It makes no latency gain
claim from reducing execution mutexes. Retain that interpretation with the
reference rather than treating its diagnostic samples as hard deadlines.

## New-cut measurements

| Metric | New-cut result |
|---|---|
| Nominal handoff count | `PENDING_NEW_CUT_RUNS` |
| Adapter predecessor-to-successor admission gap, min / median / max | `PENDING_NEW_CUT_RUNS` |
| Core command gap, min / median / max | `PENDING_NEW_CUT_RUNS` |
| Nearest recorded PX4 setpoint gap, min / median / max | `PENDING_NEW_CUT_RUNS` |
| Lease expiry / Hold during nominal runs | `PENDING_NEW_CUT_RUNS` |
| Identity / continuity rejects during nominal runs | `PENDING_NEW_CUT_RUNS` |
| Per-session command publish, transport publish, scheduling, and lock-wait percentiles | `PENDING_NEW_CUT_RUNS` |

## Measurement method

Use the same bag analyzer and adapter `NavigationCommandAdmission` boundary as
the reference. For each successful successor transition, measure from the last
admitted predecessor sample to the first admitted successor sample; calculate
min/median/max over the full nominal set and report the transition denominator.
Keep the core command gap and nearest recorded PX4 setpoint gap as separate
columns. A nearest setpoint timestamp is only an approximation when the PX4
message has no request identity.

Report periodic diagnostic percentiles by session for command-store publish,
transport publish, planning scheduling gap, command transition lock wait, and
owner publication lock wait. These are observations, not proof of worst-case
deadline behavior. Compare the maximum handoff gap with both the reference and
the unchanged 100 ms lease; flag a material worsening or a maximum approaching
that lease. Do not claim exact timing equality or an improvement from this
architecture refactor alone.

The performance evidence is pending until the three matched nominal runs have
completed from the final clean Release build and their provenance is recorded in
[`SITL_RESULTS.md`](SITL_RESULTS.md).
