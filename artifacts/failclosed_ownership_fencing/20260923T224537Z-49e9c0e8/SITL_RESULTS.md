# Focused SITL results

Pending build and deterministic tests. Reference is `49e9c0e8`, `long_featured` seed 0, tracking off: three prior complete runs, accepted `[0,1,2,3,4]`; predecessor→successor adapter admission gaps min/median/max 19.958/20.052/20.169 ms. These are reference observations, not new repair evidence.

Required repair runs: three consecutive matched nominal COMPLETE without nominal lease/identity/continuity rejection or unexpected Hold. Fault checks: injected failed replacement if exact `OptimizationFailed` can be forced without product modification, Core pause >100 ms, repeated replan failure→BACKUP→measured restart, and terminal STOP measured-speed acceptance. Record exact run IDs and evaluator-owned reports; a launch or single completion is not flight qualification.
