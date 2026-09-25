# SITL results

Profile: `long_featured`, seed 0, tracking explicitly off, same PX4 checkout and binary, exact-base control at `770699c2271dbc029be183f2affa5916909e3fa1` versus repair source `68289532b93641973238329d0ec6c4b0626b43f9`.

| Cohort | Run | Result | Accepted waypoints | Max accepted-state gap |
|---|---|---|---|---:|
| Repair | `20260925T033816-82154` | `PAUSED_SAFETY_STOP` | [0,1,2,3] | 174.436932 ms |
| Repair | `20260925T034041-82154` | `COMPLETE` | [0,1,2,3,4] | 30.093645 ms |
| Repair | `20260925T034310-82154` | `PAUSED_SAFETY_STOP` | [0,1,2,3] | 35.378922 ms |
| Exact base | `20260925T042303-124583` | `COMPLETE` | [0,1,2,3,4] | see raw analysis |
| Exact base | `20260925T042524-124583` | `FAILED_COMPONENT` | [0,1,2,3] | 444.459667 ms |
| Exact base | `20260925T042734-124583` | `WALL_TIMEOUT` | [0,1,2,3] | 28.277283 ms |

Both cohorts reported requested tracking `off`; Core and adapter effective tracking were off, with braking and estimator-health suppression false. C0-SW/qualification eligibility was `NOT_EVALUABLE` (tracking/motion policy provenance unavailable; repair cohort also had a reference-lineage mismatch on the successful run). Observed completion count is 1/3 in both cohorts. Failure dispositions differ and the small cohorts do not prove statistical parity or a common cause. The two repair pauses were fail-safe responses associated with out-of-envelope PX4 velocity-setpoint / external-mode exit evidence; the exact baseline also recorded a velocity-setpoint limit finding in its completed run. No safety thresholds changed. This SITL evidence is not flight qualification and does not establish C0-SW eligibility.

Two attempts to exercise an exact injected `kOptimizationFailed` hot-handoff on the repair source explicitly reported `INJECTION_NOT_ARMED`; runs ended before the requested hot handoff. They are not counted as exact-status evidence. The C2 emergency-brake-to-bounded-correction scenario was not deliberately triggered in SITL. Historical exact-status proof from an older behavior SHA is not substituted for current-source evidence.

First three premature setup attempts lacked the authoritative build manifest and were `NOT_EVALUABLE`; they are retained separately and excluded from the matched scenario denominator.
