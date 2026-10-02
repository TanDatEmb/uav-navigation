# H3.1 hunk ledger

- `docs/safety/runtime_safety_current.md`: update only the H3.1 ledger row.
- `measurement_buffer.hpp`: add the typed existing-history-duration field.
- `measurement_buffer.cpp`: add scan-bracket/no-scan-window pruning and retain
  bounded capacity behavior.
- `fast_lio_pipeline.cpp`: consume the configured history duration.
- `parameter_loader.cpp`: wire the existing propagated-odometry parameter.

Safety scope: bounded sensor retention only; no synchronization authority,
threshold, UNKNOWN policy, command ownership, lease, or PX4 behavior change.
Verification: focused measurement-buffer/parameter tests, backend CTest,
`python3 tools/validate_runtime_safety_ledger.py`, and `git diff --check`.
