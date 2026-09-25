# Fresh nominal C0-SW cohort

Profile: `long_featured`, seed 0, tracking requested off, same pinned PX4 binary. The first nominal attempt `external-mode-check-20260925T143332-131296` failed before simulator launch because the build manifest no longer matched edited tooling. It is classified `ENVIRONMENT_INVALID`, retained in denominator disclosure, and replaced only after a clean Release build refreshed the manifest. It is not one of the five valid runtime trials.

| Session | Runtime / mission | Waypoints | C0-SW | C0-IFP | Notes |
|---|---|---|---|---|---|
| `external-mode-check-20260925T135537-112192` | BLOCKED / PAUSED_SAFETY_STOP | 0–3 | PASS, eligible | NOT_EVALUABLE | valid predecessor retained; final terminal bridge/tracking certificate expired its existing gate |
| `external-mode-check-20260925T140742-118663` | BLOCKED / PAUSED_SAFETY_STOP | 0–3 | PASS, eligible | NOT_EVALUABLE | same expected terminal performance class; no World suspension |
| `external-mode-check-20260925T141506-123208` | PASS / COMPLETE | 0–4 | PASS, eligible | NOT_EVALUABLE | normal completion |
| `external-mode-check-20260925T142616-127156` | FAIL / mission COMPLETE | 0–4 | PASS, eligible | NOT_EVALUABLE | scenario's max-setpoint check caught 1.669 m/s during `STATUS_BRAKING`; see below |
| `external-mode-check-20260925T143615-134487` | PASS / COMPLETE | 0–4 | PASS, eligible | NOT_EVALUABLE | normal completion |

Valid cohort totals: 5/5 C0-SW eligible; 3/5 mission COMPLETE; 2/5 safe terminal stops; 2/5 overall runtime PASS (one completed run is runtime FAIL due the scenario velocity assertion). All failures remain in the denominator. This is below the strong 4/5 completion target and is explicit pre-beta stability debt, but does not by itself negate software evidence eligibility.

All five requested/Core-effective/adapter-effective tracking states are OFF/false; `suppress_braking=false`, `suppress_health=false`, config mismatch false. No C0-IFP motion/tracking acceptance policy is claimed.

For the completed-but-runtime-FAIL session, exact PX4 input trace shows request 5, execution generation 21, command status BRAKING, speed 1.668744 m/s, while the configured cruise request/effective cruise is 1.5 m/s and control envelope is 5 m/s (physical model 12 m/s). The scenario checker currently applies its `expected_max_velocity_mps` assertion to every finite PX4 setpoint, including braking samples. The setpoint sample carries command status `BRAKING`; the product contract distinguishes measured overspeed and braking from prospective steady cruise. The runtime helper `_speed_contract_failures` nevertheless computes a max over all finite setpoints without a status filter. This explains the runtime FAIL as an over-broad scenario acceptance predicate over a braking sample; it is not evidence of an out-of-envelope or C0-SW failure. The raw runtime FAIL remains unchanged. Whether to make this scenario metric mode-aware is queued for a separately reviewed runtime/evidence contract change; no threshold or evaluation rule was altered here.
