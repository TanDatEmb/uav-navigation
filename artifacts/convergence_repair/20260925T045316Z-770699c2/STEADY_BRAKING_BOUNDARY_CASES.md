# Steady braking boundary cases

Pinned deterministic vectors in `navigation_planning_backend/test/test_planner_config.cpp`:

| Case | Speed m/s | A limit m/s² | J limit m/s³ | Before correction | Result |
|---|---:|---:|---:|---|---|
| Acceleration rounded down | 0.08839407447163837 | 0.00012525139441244633 | 1e6 | Actual polynomial A is above limit while the old ULP helper accepts it | One upward `nextafter`; strict A/J pass; finite support |
| Jerk rounded down | 0.028138831782626911 | 1e6 | 4.2389422825644623 | Actual polynomial J is above limit while the old ULP helper accepts it | One upward `nextafter`; strict A/J pass; finite support |

Additional focused cases cover an interior duration with no correction, abort returning `kBudgetExhausted`, bounded correction/support, preservation of non-steady PVAJ behavior, and measured overspeed behavior. Maximum observed duration correction in the pinned round-down regressions: one ULP. Maximum configured duration loop: 32 corrections. Support-boundary integration correction: max 8 representable speed steps.
