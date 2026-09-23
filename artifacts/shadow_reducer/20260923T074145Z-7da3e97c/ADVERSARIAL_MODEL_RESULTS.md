# Deterministic adversarial model results

`python3 -m unittest discover -s tests/shadow_reducer -q`: 39 tests, all pass. This is model proof of represented event orderings, not component/SITL or flight qualification.

| Case | Result |
|---|---|
| PT1 continuation→crossing | accepts exactly once. |
| PT2 crossing→continuation | measured witness survives and later acceptance succeeds under same identity. |
| PT3 crossing→leave ball→continuation | route-relative witness survives; acceptance succeeds. |
| PT4 localization reset / PT5 route revision | old crossing invalid. |
| PT6 reverse through segment boundary or lower `u` | old crossing invalid. |
| Skipped ball / sharp corner | segment witness accepts without demanding outgoing velocity alignment; current trace does not supply full 3-D geometry, so geometric correctness is source/test carried. |
| Coincident PASS→STOP | PASS gate and subsequent STOP confirmation remain separate; generic STOP model test requires measured stop. Product-specific coincident suffix exception remains source-derived. |
| Odometry gap / out-of-order source stamp | segment witness discarded at >250 ms or non-forward source order. |
| EX1 active + staged / EX2 stale context | predecessor owns until atomic cutover; stale goal/world/dynamics result discarded by one context key. |
| Recoverable brake / EX3 committed stop + late nominal / EX4 polynomial endpoint while moving / EX5 measured stop | Recovery requires a same-context unexpired trajectory plus event-level continuity certificate; committed stop forbids moving recovery; endpoint alone does not become Stopped; measured-stop event enables new session. |
| WL1–WL4 | certificate expiry blocks publication; release fences generation; fresh world alone cannot revive it. |
| Hold permutations | ACK/status/callback orders, missing status, deactivation, operator/failsafe, duplicate/stale status, no response and retry deadline preserved. |
| Trace integrity | gap/queue drop comparison produces `INSUFFICIENT_TRACE`; duplicate and observer inversion are tracked distinctly by replay. |

The tests use synthetic typed events. They prove the reducer can represent these orderings; they do not prove a PX4 callback result means AUTO_LOITER, nor quantify scheduler/transport delay.
