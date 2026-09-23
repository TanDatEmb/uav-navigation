# Desired intent cleanup SITL results

## Status

`PENDING_NEW_CUT_RUNS`

No SITL run from the desired-intent cleanup HEAD is recorded yet. The run rows
below are placeholders and must be filled from the resulting artifacts; no
new-cut measurements are inferred from the reference.

## Pinned behavioral reference

The reference product source is pinned to
`ba9c15a1214ca8ba54ec8f301f1e4056a3322848`. Its product source is identical to
`2e4f7a0f1e09f4d72afed23686e26dc173150dd3`, whose matched SITL evidence is
recorded in the [execution authority unification SITL report](../../execution_authority_unification/20260923T123424Z-8ffa14e5/SITL_RESULTS.md).
That report records three `long_featured`, seed 0, `tracking=off` nominal runs
with accepted sequence `[0,1,2,3,4]` and mission completion in 3/3 runs. Those
are reference results only; they do not count as runs of this cleanup.

Reference boundary observations and provenance are retained in the linked
report. New-cut nominal parity requires three consecutive `COMPLETE` runs,
accepted sequence `[0,1,2,3,4]` in each, and no nominal lease expiry, Hold
request, identity rejection, or continuity rejection.

## New-cut run matrix

All runs must use the final cleanup HEAD and its clean authoritative Release
manifest, the same PX4 checkout and binary as the reference, `long_featured`,
seed 0, `tracking=off`, dynamics override `off`, and unchanged product gates.
Record source SHA, build manifest, PX4 provenance, session directory, accepted
sequence, completion status, and adapter boundary events for every run.

| Run | Purpose | Session / result |
|---|---|---|
| Nominal 1 | Matched `long_featured` handoff liveness | `PENDING_NEW_CUT_RUNS` |
| Nominal 2 | Matched `long_featured` handoff liveness | `PENDING_NEW_CUT_RUNS` |
| Nominal 3 | Matched `long_featured` handoff liveness | `PENDING_NEW_CUT_RUNS` |
| Fault 1 | Pause Core for more than 100 ms; stale command must request Hold | `PENDING_NEW_CUT_RUNS` |
| Fault 2 | Repeated replan failure; retain BACKUP through measured stop, then restart | `PENDING_NEW_CUT_RUNS` |
| Fault 3 | Terminal STOP and measured mission completion | `PENDING_NEW_CUT_RUNS` |

## Run method

1. Build the final cleanup HEAD using the authoritative Release workflow and
   preserve its manifest and pinned submodule SHAs.
2. Run three separate nominal `long_featured` sessions with the matched profile
   above. Capture the canonical runtime evaluator output and bag data for the
   entire mission.
3. Run the three fault cases independently. For Core pause, record pause length,
   stale-command detection, Hold request, and adapter response. For repeated
   replanning failure, record the injected failures, BACKUP role and ownership,
   measured stop witness, and restart. For terminal STOP, record the terminal
   acceptance event and measured speed.
4. Preserve each raw session and report path. Do not count blocked fault runs as
   nominal failures, and do not infer PX4 firmware consumption from recorder
   timestamps.

World isolated-source freshness and emergency SITL faults remain outside this
focused run matrix unless explicitly selected later; their existing deterministic
tests do not substitute for new-cut SITL evidence.
