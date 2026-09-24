# SITL results and denominator

The pinned predecessor cohort (`96ed8d089e576c505abdc15500746e9b5be10e50`, exact PX4 binary in `BASE_PROVENANCE.md`) ran ten `long_featured`, seed 0, true tracking-off attempts. It produced 8/10 mission COMPLETE, including eight consecutive COMPLETE on runs 3–10. Runs 1–2 failed in terminal recovery and remain in the denominator. `NOMINAL_COHORT.csv` and `TERMINAL_RESULTS.csv` retain every run and raw-session path.

Core and adapter effective tracking were off; braking and health suppression were false. The largest accepted-state gap was 35.592 ms, leaving 164.408 ms to the 200 ms adapter freshness boundary. No natural >150 ms accepted-state tail occurred in this ten-run cohort. The separate historical temporal pilot is indexed in `TEMPORAL_STALL_EVENTS.csv`; it is not an eleventh nominal run.

The current product branch has made no flight-behavior change. A new native/host diagnostic pilot is needed to validate temporal-layer capture. The predecessor 8/10 result is a **blocking reliability result**, not a green product gate. Every evaluator `NOT_EVALUABLE` outcome remains unchanged and is not used as this branch's product-behavior verdict.

At **first analytic completion** across the ten predecessor runs, measured endpoint error was p50 0.326, p95 0.401, p99 0.405, max 0.406 m; speed was p50 0.261, p95 0.506, p99 0.522, max 0.527 m/s. The eight eventual measured mission acceptances had endpoint error p50 0.235, p95 0.273, max 0.273 m and speed p50 0.093, p95 0.099, max 0.099 m/s. The p95/p99 figures are descriptive interpolation with n=10/8, not tail bounds. These are distinct lifecycle events; completion distributions cannot substitute for the failed recovery gates, which saw 0.762/0.759 m later.
