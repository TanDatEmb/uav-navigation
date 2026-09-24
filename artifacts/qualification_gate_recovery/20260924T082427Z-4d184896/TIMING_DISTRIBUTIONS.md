# Timing distributions

The pinned A2 capture did not contain per-sequence producer publication or adapter callback/lock witnesses; `receive_age_ms=208.583` is a safety boundary observation, not a transport distribution. The natural pilot and all ten cohort sessions contain exact `(localization_epoch, sequence)` joins. Every accepted callback in these eleven sessions has a matching producer trace. The offline analyzer reads both; `NOMINAL_TIMING.csv` contains each run's sample count and p50/p95/p99/max. No WCET is inferred.

For the ten-run cohort, median of the **per-run** quantiles and largest per-run maximum (ms), with total samples across runs:

| Boundary | n | Median p50 | Median p95 | Median p99 | Largest max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Producer publish interval | 38,026 | 24.998 | 25.406 | 25.652 | 35.547 |
| Worker ready→publish call | 38,036 | 0.061 | 0.086 | 0.103 | 0.782 |
| Publish call→adapter callback | 36,608 | 0.111 | 0.186 | 0.248 | 3.029 |
| Adapter mutex wait | 36,608 | 0.0002 | 0.0005 | 0.0009 | 0.111 |
| Callback→accepted receive | 36,608 | 0.0011 | 0.0022 | 0.0033 | 0.112 |
| Accepted receive interval | 36,598 | 24.999 | 25.430 | 25.707 | 35.592 |

These columns are **not pooled quantiles**. The source's median p50/p95/p99 was 20/20/20 ms; maximum 28 ms. Pilot 3's p50/p95/p99/max accepted gap was 24.991/25.542/29.518/481.677 ms (n=3,163); producer max 481.825 ms, publish→callback max 1.584 ms and mutex max 0.00789 ms. The pilot's negative 281.677 ms margin to the unchanged 200 ms lease contrasts with the cohort's smallest positive margin of 164.408 ms.
