# E4 — world, recertification, command and lease observations

**Verdict `E4_PARTIAL`; continuous-heartbeat proposal `INSUFFICIENT_EVIDENCE`.** `E4_LATENCY_SAMPLES.csv` holds per-run nearest-rank p50/p95/p99/p99.9 (only N≥1000), max and N. It never pools different profiles or clocks. Fourteen exact-TARGET run manifests are eligible for diagnostic inspection; none is mission qualification. Thirteen runs emitted command/planner diagnostics; one has no useful command samples. A run with infrastructure-invalid simulation pause is retained for fault classification, not real-time acceptance.

`FACT_FROM_PINNED_PRODUCT_SOURCE` Distinct contract values: mapping snapshot target 100 ms, planner period 100 ms, solve deadline 80 ms, command period 20 ms, world source freshness 500 ms, published command valid-until +100 ms, adapter receive lease 100 ms and state freshness 200 ms (`planning_timing.hpp:9-21`, `navigation_runtime_node.cpp:678-756,8500-8504`, `navigation_mode_node.cpp:113-168`). On stale world, runtime `suspendCommandForWorldFreshness()` retains bundle identity but clears command availability and stops publication (`navigation_runtime_node.cpp:2983-3004`); a later compatible world can recertify/resume. The adapter records `last_command_receive_ns_` internally (`navigation_mode_node.cpp:1141`) and checks its lease (`:1389-1393`), but current trace does not emit that receive timestamp.

`FACT_FROM_RUNTIME_TRACE` Example per-run metrics (ms; **diagnostic samples**, not paired stage latency):

| Profile/run | Metric | N | p99 | max |
|---|---|---:|---:|---:|
| structured_obstacle `...197705` | command recorder inter-arrival | 1,257 | 20.122 | 40.037 |
| structured_obstacle | mapping callback duration | 354 | 42.758 | 134.907 |
| structured_obstacle | active revalidation duration | 125 | 2.390 | 2.748 |
| long_three_pillars_multiwaypoint `...247206` | command recorder inter-arrival | 14,564 | 20.508 | 40.510 |
| multiwaypoint | mapping callback duration | 2,512 | 100.649 | 124.723 |
| multiwaypoint | snapshot export duration | 2,512 | 17.672 | 19.570 |
| multiwaypoint | active revalidation duration | 2,846 | 6.519 | 10.499 |
| multiwaypoint | command transport publish diagnostic duration | 2,846 | 0.343 | 0.528 |
| multiwaypoint | adapter tracking setpoint update duration | 14,568 | 0.219 | 2.527 |
| multiwaypoint | state source age at adapter tracking update | 14,568 | 16.000 | 20.000 |
| multiwaypoint | world source age at command source stamp | 14,565 | 284.000 | 500.000 |

Across these different profiles, the largest sampled mapping callback diagnostic is 229.260 ms (`...229248`), and largest active-revalidation diagnostic is 10.499 ms (`...247206`). Neither is a WCET. Some sampled planning scheduling gaps and recorder command gaps are seconds because run phases include mode exits/restarts and one explicit simulation-clock pause. For example, `...256944` has a 1.380 s recorder command gap from request 2→3 while status still reports external mode; execution trace shows **69 `velocity_hold` setpoint updates** during the gap. That is an adapter safety stream, not evidence that flight control went silent. `...212855` has a 28.700 s gap and is `INFRASTRUCTURE_INVALID` (simulation clock wall-arrival lease violation); fault class `SIMULATION_PAUSE`. Other long request-transition gaps are `UNRESOLVED` without adapter receive/authority events. No gap is labeled a measured 100 ms lease violation.

Observed planner counters `world_freshness_command_suspend_count`, `world_freshness_command_recovery_count` and `command_publication_deadline_miss_count` end at zero in the 13 runs that emit them. The stale-world silence→adapter-lease-expiry timeline requested in §26 was **not observed**; absence is not proof of unreachability. The current recorder lacks a direct adapter command-receive stamp, so lease violations and transport max/p99 are `UNOBSERVABLE_WITH_CURRENT_TARGET`.

| Required stage | Evidence | Status |
|---|---|---|
| world observation source | mapping `observation_stamp_ns` | recorded ROS source time |
| mapping callback start → snapshot publish | only `mapping_callback_total_us`, snapshot export and diagnostic stamp; diagnostic stamp is an observer/source proxy | exact pair `UNOBSERVABLE_WITH_CURRENT_TARGET` |
| snapshot → recert start/end/commit | `active_revalidation_us`/`pending_revalidation_us` are sampled durations, not paired start/end/commit | exact pair unobservable |
| command timer start → publish | scheduling and publish-duration diagnostics; no event-paired command timestamp | exact pair unobservable |
| runtime publish → adapter receive/accept | no adapter receipt trace; bag timestamp is recorder | **unobservable** |
| adapter receive → PX4 setpoint | `px4_input_trace` has local update start/end, not matching receive | **unobservable** |
| command inter-arrival | bag wall and command ROS source stamps separately | measured at observers/publisher source, not lease |
| state source age at adapter update | local `px4_input_trace` state ROS stamp and update ROS time | measured for emitted trace, not all attempted callbacks |

L1/L2 and several obstacle-heavy profiles have source-compatible evidence; profile identities are explicit in CSV. L3 dense mapping has diagnostic samples but no controlled repeat. L4 near-deadline and L5 CPU contention were not isolated; L6 stale/recovery is unobserved. No timeout, deadline or safety gate was adjusted. The current telemetry cannot support a causal event-paired transport tail or a policy reserve, so a new external capture or separate minimal instrumentation branch is required before temporal protocol approval.
