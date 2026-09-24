# Performance scope

The only runtime additions are in explicit diagnostic/test runs: one native IMU metadata subscription, a 50 ms observer-loop timestamp, and a 150 ms diagnostic gap recording budget. The native observer is a separate process and never publishes flight commands. Normal product path is unchanged.

The predecessor ten-run cohort predates these additions; its per-run producer/publish→callback/mutex/acceptance distributions are in `artifacts/qualification_gate_recovery/20260924T082427Z-4d184896/NOMINAL_TIMING.csv`. A new diagnostic run is required before any before/after overhead claim. No hard product deadline is inferred from the small sample.

