# Remaining boundary debt

This cut does not change world-source stale/revision policy, PX4 Hold callback ordering, or the emergency planner runtime path. Those remain separate boundary-evidence tasks.

The ROSIDL `NavigationCommand` wire schema changed. No in-repository live consumer requiring the prior 74-field schema was found, but deployment consumers outside this repository must be checked before rollout. Historical bags require the matching old message overlay for deserialization. The observer diagnostic topic is best effort, so missing evidence may make a runtime claim not evaluable while flight behavior remains governed by the control command and adapter-local checks.

The source- and mission-parity architecture gate is satisfied, but the runner's `tracking_experiment_mode="off"` is not an unsuppressed adapter policy in the pinned SITL source: zero coefficients under `use_sim_time=true` cause `TRACKING_EXPERIMENT_BYPASS`. Baseline and new runs share this mismatch and the evaluator correctly marks them `INCONCLUSIVE`/not qualification eligible. Repairing this requires a separate safety-contract decision; this cut did not change the gate.

Adapter callback p50/p95/p99/max and mutex-held time have no before/after instrumentation. The command CDR size is reduced, but adding a 50 Hz observer topic increases combined serialized payload; actual DDS throughput and CPU overhead remain unmeasured. Old-wire every-sample C++ replay is also incomplete because the pinned old rosbag requires its matching old ROSIDL overlay. These are evidence limits, not inferred performance gains.

The isolated world-source stale/revision behavior, PX4 Hold callback ordering, and emergency runtime path remain the three larger boundary-evidence debts. The Core-pause fault was focused command-lease evidence with a simultaneous mapping backlog, not a qualification of all Hold orderings.
