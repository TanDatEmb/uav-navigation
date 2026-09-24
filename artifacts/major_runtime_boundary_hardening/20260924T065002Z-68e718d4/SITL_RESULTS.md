# Three true tracking-off SITLs

All runs: `long_featured`, seed 0, external mode, requested tracking `off`,
clean source HEAD `1c75f3b4`, PX4 checkout `deaff86ee`, binary SHA256
`b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.
Core and adapter effective witnesses both said mode `off`, enabled false,
braking suppression false, health suppression false; report configuration
mismatch false and no experimental bypass in each run.

| Run | Experiment ID | Mission | Accepted | Adapter/flight observation | Runner/evaluator |
| --- | --- | --- | --- | --- | --- |
| A1 | `hardening-phase-a-off-1` | COMPLETE | `[0,1,2,3,4]` | No Hold request; periodic adapter metrics showed 2770 received/accepted, 0 rejected | FAIL / NOT_EVALUABLE |
| A2 | `hardening-phase-a-off-2` | FAILED_COMPONENT | `[0,1,2]` | `RECEIVE_STALE`, source age 8.000 ms, receive age 208.583 ms; command rejected, navigation failed closed and Hold requested | FAIL / NOT_EVALUABLE |
| A3 | `hardening-phase-a-off-3` | COMPLETE | `[0,1,2,3,4]` | No Hold request; periodic adapter metrics showed 2893 received/accepted, 0 rejected | FAIL / NOT_EVALUABLE |

A2's adapter log has the first explicit fail-close cause:
`Rejecting planner backend command because navigation odometry lease is stale`
followed by `navigation odometry stale at command acceptance; handing over to
PX4 Hold`. Source inspection of `navigation_mode_node.cpp` confirms the
unchanged `evaluateExecutionStateFreshness()` gate acts on the current cached
odometry receive timestamp before command acceptance and calls
`failNavigation()` for a stale lease. The trace does not prove why the callback
was late. The scenario later observed external mode exit and requested Hold
with trigger `unexpected_external_mode_exit`; it did not complete the mission.

All three versioned evaluator assessments were NOT_EVALUABLE for lifecycle
attribution, motion acceptance policy, reference lineage, and tracking
coverage policy. A2 and A3 also reported `SOURCE_TIMESTAMP_DUPLICATE`.
Mission outcome and evaluator result are deliberately kept separate. There
are only **2/3 mission completions**, not three consecutive completions, and
**0/3 evaluator passes**. The campaign stops before Phase B.
