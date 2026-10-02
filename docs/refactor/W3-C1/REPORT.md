# W3-C1 part 2 — baseline and golden evidence

## Verdict

**NOT_EVALUABLE / review request.** The clean build and the requested 18-run
M1 plus 30-run M2 matrix were executed on one validated binary cohort. The
session artifacts are complete and provenance-clean, but the acceptance
classification is not evaluable: the runtime reports are predominantly
`BLOCKED`/`NOT_EVALUABLE`, six sessions have invalid simulation-clock leases,
and the golden replay sweep was not completed.

## Cohort

- Branch: `refactor/W3-C1-part2`
- Runtime cohort SHA: `c435a4f19da0455cd538cb7ae111c0ba00e2e644`
- Build manifest: `install/.uav_navigation_build_manifest.json`
- Matrix log: `/home/letandat/uavnav-w3-c1-part2-sitl-batch-20261002.log`
- Summary: `/home/letandat/uavnav-w3-c1-part2-baseline-summary-20261002/`

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
| Golden capture | COMPLETE accounting — 43 snapshots (2WP), 11 (5WP), 16 (9WP); all sidecars `COMPLETE` |
| Golden replay | **NOT_EVALUABLE** — full 70-snapshot × 2 sweep was interrupted; partial manifest is not acceptance evidence |

The initial failure-only capture produced no snapshots for 5WP/9WP. A second
capture with `UAV_NAVIGATION_NOMINAL_SNAPSHOT_FAILURE_ONLY=0` produced the
complete three-scene snapshot cohort above, on the same binary. Replay output
and the partial manifest are under:

`/home/letandat/uavnav-w3-c1-part2-golden-20261002/`

## Required follow-up

Run the replay binary on every `nominal_problem_snapshot_[0-9]*.json` under
the three `*-all-snapshots` directories twice, then require every pair to
have identical complete output and return code 0. Do not mark C1 golden PASS
until the resulting manifest covers all 70 snapshots.

No threshold, safety gate, or product behavior was tuned from these runs.
