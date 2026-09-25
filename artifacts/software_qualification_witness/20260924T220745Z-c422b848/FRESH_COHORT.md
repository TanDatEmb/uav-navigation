# Fresh cohorts

Two sequential five-run cohorts were captured with `long_featured`, seed 0, tracking requested/effective off, no braking or health suppression, and the pinned PX4 binary. Each cohort used one clean source SHA, listed in `BASE_PROVENANCE.md`. `FRESH_COHORT.csv` gives every raw path and result; `RAW_EVIDENCE_MANIFEST.csv` gives sizes/hashes. All original runner verdicts remain visible.

| Cohort | Capture source | Mission COMPLETE | Safety stop | Other component failure | C0-SW eligible after offline reevaluation | Required lifecycle unresolved/conflicts | Required references missing/conflicts |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| Primary | `5a77f5e0` | 5/5 | 0 | 0 | 5/5 consecutive | 0/0 all runs | 0/0 all runs |
| Verification | `61ee1e50` | 2/5 | 1 | 2 | 2/5 | 0/0 all runs | 0/0 all runs |

Primary accepted `[0,1,2,3,4]` in every run and has 15,856 required references. Verification has 11,528; runs 2/3 hit adapter `RECEIVE_STALE` and Hold after an upstream receive gap; run 5 stopped safely but its specific safety-stop gate decision is not assessed, so it remains `NOT_EVALUABLE`. Neither outcome is hidden. Original runner verdict for primary is `FAIL` in all five because integrated flight-performance criteria still fail; C0-SW is an additional scoped assessment, not a rewrite of runner outcome.

The CSV is an offline reevaluation at evaluator `db43b608` of raw sessions captured at earlier clean source commits. The capture-time report files are retained unmodified and can carry earlier C0-SW statuses. This provenance difference is intentional and must accompany any use of the 5/5 result.
