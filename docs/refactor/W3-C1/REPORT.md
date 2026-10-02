# W3-C1 part 2 — baseline and golden evidence

## Verdict

**NOT_EVALUABLE / review request.** The clean build and the requested 18-run
M1 plus 30-run M2 matrix were executed on one validated binary cohort. The
session artifacts are complete and provenance-clean, but the acceptance
classification is not evaluable: 38/48 runtime rows are `BLOCKED`, all 48
classification rows remain `NOT_EVALUABLE`, six sessions have invalid
simulation-clock leases, and the golden replay sweep was not completed.

## Cohort

- Branch: `refactor/W3-C1-part2`
- Runtime cohort SHA: `c435a4f19da0455cd538cb7ae111c0ba00e2e644`
- Build manifest: `install/.uav_navigation_build_manifest.json`
- Matrix log: `/home/letandat/uavnav-w3-c1-part2-sitl-batch-20261002.log`
- Summary source: `/home/letandat/uavnav-w3-c1-part2-baseline-summary-20261002/`
- Committed run CSV: [`baseline_runs.csv`](baseline_runs.csv)
- Per-run-p50 distribution: [`baseline_distribution.csv`](baseline_distribution.csv)
- Pooled raw-sample distribution: [`baseline_pooled_distribution.csv`](baseline_pooled_distribution.csv)
- Per-cell outcome/cause table: [`baseline_by_cell.csv`](baseline_by_cell.csv)

The source tree was clean while the matrix ran. All 48 sessions report the
same navigation SHA, `provenance_dirty=false`, complete required artifacts,
and cleanup `PASS`.

## Verification

| Check | Result |
|---|---|
| Clean release build from `origin/main` | PASS — 23 packages; warnings only |
| `make test` | PASS — 94 package/C++ tests and 40 tools tests |
| M1 | COMPLETE as execution matrix — 18 sessions: 3 scenes × SAFE/FAST × 3 |
| M2 | COMPLETE as execution matrix — 30 sessions: 2 scenes × 1/3/5 m/s × 5 |
| Provenance | PASS — 48/48 on `c435a4f`, clean navigation source |
| Cleanup | PASS — 48/48 |
| Infrastructure | 42 valid; 6 `INFRASTRUCTURE_INVALID` from simulation-clock lease gaps |
| Runtime classification | 4 `PASS`, 6 `FAIL`, 38 `BLOCKED`; all classification rows remain `NOT_EVALUABLE` under the current acceptance policy |
| M1 terminal stop | 15/18 `PAUSED_SAFETY_STOP` (83.3%); FAST 6/9, SAFE 9/9 |
| M1 terminal outcomes | 15 `PAUSED_SAFETY_STOP`, 2 `COMPLETE`, 1 `FAILED_COMPONENT` |
| M2 terminal stop | 23/30 `PAUSED_SAFETY_STOP` (76.7%) |
| Golden capture | COMPLETE accounting — 43 snapshots (2WP), 11 (5WP), 16 (9WP); all sidecars `COMPLETE` |
| Golden replay | **NOT_EVALUABLE** — full 70-snapshot × 2 sweep was interrupted; partial manifest is not acceptance evidence |

The initial failure-only capture produced no snapshots for 5WP/9WP. A second
capture with `UAV_NAVIGATION_NOMINAL_SNAPSHOT_FAILURE_ONLY=0` produced the
complete three-scene snapshot cohort above, on the same binary. Replay output
and the partial manifest are under:

`/home/letandat/uavnav-w3-c1-part2-golden-20261002/`

## Per-cell M1/M2 evidence

`paused/total` is the exact terminal `PAUSED_SAFETY_STOP` count from
`scenario.json`, not the qualification classification. `cause` is the
read-only initiator from `triage_sessions.py`; `infra` counts sessions whose
report marks the simulation-clock lease invalid.

| Matrix | Scene | Speed | Policy | Paused/total | Runtime | Cause counts | Infra |
|---|---|---:|---|---:|---|---|---:|
| M1 | legacy | 5 | FAST | 6/9 (66.7%) | 6 BLOCKED, 2 PASS, 1 FAIL | NO_SOLVE_FAILURE 4; SOLVE_DYNAMICS 2; CERT_ROUTE_REGRESSION 2; SOLVE_DEADLINE 1 | 1 |
| M1 | legacy | 5 | SAFE | 9/9 (100.0%) | 9 BLOCKED | SOLVE_DYNAMICS 6; CERT_ROUTE_REGRESSION 1; NO_SOLVE_FAILURE 1; SOLVE_KNOWN_FREE 1 | 0 |
| M2 | sanity_open | 1 | SAFE | 1/5 (20.0%) | 1 BLOCKED, 4 FAIL | NO_SOLVE_FAILURE 4; SOLVE_KNOWN_FREE 1 | 2 |
| M2 | sanity_open | 3 | SAFE | 2/5 (40.0%) | 2 BLOCKED, 2 PASS, 1 FAIL | NO_SOLVE_FAILURE 4; SOLVE_DYNAMICS 1 | 2 |
| M2 | sanity_open | 5 | SAFE | 5/5 (100.0%) | 5 BLOCKED | SOLVE_DYNAMICS 3; NO_SOLVE_FAILURE 1; CERT_ROUTE_REGRESSION 1 | 0 |
| M2 | structured_obstacle | 1 | SAFE | 5/5 (100.0%) | 5 BLOCKED | SOLVE_KNOWN_FREE 5 | 0 |
| M2 | structured_obstacle | 3 | SAFE | 5/5 (100.0%) | 5 BLOCKED | NO_SOLVE_FAILURE 5 | 1 |
| M2 | structured_obstacle | 5 | SAFE | 5/5 (100.0%) | 5 BLOCKED | NO_SOLVE_FAILURE 1; SOLVE_KNOWN_FREE 2; COMMIT_GATE_WORLD_ADVANCED 2 | 0 |

The six invalid runs are recorded in `baseline_by_cell.csv` with their full
session paths and lease-gap reports. The artifacts prove the simulation-clock
lease gaps (1 M1 FAST and 5 M2 SAFE); whether another build/agent was running
concurrently is **NOT_MEASURED**, so it is not inferred here.

## Distribution interpretation

`baseline_distribution.csv` aggregates each session's p50 and therefore has
`n = number of sessions` per cell/metric. `baseline_pooled_distribution.csv`
aggregates the raw samples across those sessions and is the source for pooled
p50/p95/p99/max. For example, the pooled M1 FAST mapping-update distribution
is `n=21288`, p50 `32942 us`, p95 `54173 us`, p99 `65240 us`, max `341894 us`;
the corresponding pooled solve distribution is `n=459`, p50 `21040.708 us`,
p95 `56234.38 us`, p99 `75763.422 us`, max `80188.695 us`. These are
diagnostic distributions only and do not close a safety gate.

## Required follow-up

Run the replay binary on every `nominal_problem_snapshot_[0-9]*.json` under
the three `*-all-snapshots` directories twice, then require every pair to
have identical complete output and return code 0. Do not mark C1 golden PASS
until the resulting manifest covers all 70 snapshots.

No threshold, safety gate, or product behavior was tuned from these runs.
