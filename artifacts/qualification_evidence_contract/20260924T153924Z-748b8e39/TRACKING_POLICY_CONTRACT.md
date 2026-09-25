# Tracking policy contract and candidate calculation

Three policies are independent: product tracking safety (Core/adapter current gates), the explicitly requested/effective tracking experiment (`off` in the ten-run cohort), and evaluator acceptance against independent truth. Disabling the experiment does not disable either safety or qualification measurement. Prior cohort metadata reports requested/Core/adapter experiment `off` and suppression flags false; this is configuration evidence, not an evaluator threshold.

The evaluator requires a coverage policy (`min_coverage_ratio`, `max_uncovered_interval_s`, `max_pairing_gap_s`) and acceptance policy (`position_error_p95_max_m`, `position_error_max_m`, `velocity_error_p95_max_mps`, `velocity_error_max_mps`). This branch additionally requires explicit nonempty `version` and `provenance` on both policy records before they can authorize qualification. No approved C0 values exist in product config or the current safety contract. Synthetic fixture values are test data, not policy.

Raw ten-run descriptive ranges (independent truth reference comparison) are:

| Per-run statistic | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Position p95, m | 0.218 | 0.276 | 0.496 |
| Position maximum, m | 0.272 | 0.446 | 0.769 |
| Velocity p95, m/s | 0.376 | 0.553 | 1.052 |
| Velocity maximum, m/s | 0.756 | 1.032 | 1.647 |

These data cannot be used to set thresholds and then qualify the same runs: successful and failed missions overlap in tracking error, and the two failed terminal runs are a closed-loop performance issue. The planner's 0.25 m tracking budget is a command-to-LIO planning margin, not a bound for command-to-independent-truth error. A defensible truth threshold needs an independently allocated estimator/truth error budget and frame/clock uncertainty bound; a velocity threshold needs a control-performance requirement. No such allocations were found. Candidate formula: `truth position allowance ≤ command-to-LIO allocation + LIO-to-truth allocation + alignment uncertainty`; only the first term is currently bounded. Therefore numeric evaluator policy remains `POLICY_DECISION_REQUIRED`, and integrated tracking stays `NOT_EVALUABLE`.
