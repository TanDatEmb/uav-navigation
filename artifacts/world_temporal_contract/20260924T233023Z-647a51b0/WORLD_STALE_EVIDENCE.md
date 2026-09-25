# Isolated world-source stale evidence

**Result: NOT RUN / evidence gap.** No isolated SITL fault in this branch stops only registered world observations while proving odometry, health, Core timer, planner process, PX4 adapter and simulator remain alive. No existing runtime parameter/service/harness was found that provides this isolation. Pausing the whole Core or registered odometry publisher would not satisfy the required experiment.

Source-level expected behavior is: timer/planner checks compare latest immutable world's `observation_stamp_ns` to ROS now using the unchanged 500 ms window; command publication skips if stale; `suspendCommandForWorldFreshness` suspends only its exact active execution snapshot. The active identity remains stored. Later fresh mapping publication runs exact full/disjoint recertification before it can resume. This is source proof, not runtime evidence.

Do not infer isolated fault behavior or map `TEMPORAL_SAFETY=PASS` from unit tests. The branch cannot reach `WORLD_TEMPORAL_CONTRACT_HARDENED` without this runtime evidence and evaluator-attributed world transaction lineage.
