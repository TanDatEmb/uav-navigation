# Open findings

| ID | Finding | Current evidence and next owner |
| --- | --- | --- |
| C0SW-01 | Verification runs 2/3 ended `FAILED_COMPONENT` | Adapter rejected state as `RECEIVE_STALE` at 200.477/211.246 ms then Hold. Software fail-closed is visible; first upstream stalled layer is unresolved. Temporal/integration investigation, not an evaluator green override. |
| C0SW-02 | Safety-stop run 5 C0-SW outcome not assessable | 312 no-execution signals have exact Core rejected authorization and typed adapter rejection. The measured safety-stop gate decision is not yet joined; retain `NOT_EVALUABLE`, add exact gate witness before counting such runs eligible. |
| C0SW-03 | Same-sample setpoint trace coverage | 147 of 15,856 primary references have exact Core authorization and adapter admission but no same-sample setpoint trace (3 of 11,528 verification). Coalescing vs observer loss is unproven; do not claim firmware consumption. |
| C0SW-04 | Instrumentation latency delta | No matched A/B profile of diagnostic emission versus uninstrumented source; writer loss is zero but precise overhead remains unverified. |
| C0SW-05 | Historical evidence incompleteness | Original 140 unresolved lifecycle transactions and 5,957 missing exact references remain missing by design. |

Deferred integrated findings: Gazebo/native simulation fidelity and PX4 closed-loop tracking/stopping tuning. They are real limitations, not resolved by C0-SW witness closure.
