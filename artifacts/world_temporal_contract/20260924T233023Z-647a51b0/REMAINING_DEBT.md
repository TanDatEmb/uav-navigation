# Remaining debt

1. C0-SW world transaction event classes, exact lifecycle joins, writer accounting and evaluator axes are not implemented; world evidence eligibility is unavailable.
2. Isolated world-source stale and fresh-world recovery SITL with live odometry/health/Core timer/planner/PX4 adapter is not available or executed.
3. Mapping publication non-committed failure dispositions need reachability/negative-path proof before narrowing generic fail-stop behavior.
4. Live same-tick world publication is rejected by MappingActor while store permits it; historical A3 remains unresolved.
5. Localization reset barrier matrix has source fences and existing component tests but no comprehensive world callback/recertification/pending integration race proof.
6. `WorldSnapshotIdentity` has source time only; world receive/publication progress is not independently represented.
7. External Gazebo fidelity and PX4 closed-loop tracking/stopping remain deferred and unchanged.
