# Safety invariants for the admission refactor

1. A malformed or temporally invalid replacement yields `REJECT_RETAIN_PREVIOUS`; it does not revoke the already admitted command. The incumbent remains subject to its own receive, source-state, estimator, world and terminal gates.
2. A stale odometry lease at command acceptance yields `REJECT_FAIL_NAVIGATION`; a tracking-envelope failure yields `REJECT_SAFETY_STOP`. A nonincreasing sample ID yields `REJECT_RETAIN_PREVIOUS`. Terminal authority already closed yields `IGNORE_AFTER_TERMINAL`.
3. Session checks retain existing health/localization, mode activation, mission, request and world monotonicity ordering. A predecessor command remains admissible while desired successor planning is in progress; post-cutover older samples remain subject to sample and identity gates.
4. Successful `NavigationCommandAdmission` is published only after the command is committed under `trajectory_mutex_`. A rejection event cannot be mistaken for a success receipt.
5. `NavigationExecutionDiagnostics` and `NavigationCommandRejection` are observer-only. Their loss, delay, duplication or malformed payload cannot authorize or veto a command. The adapter never subscribes to `NavigationExecutionDiagnostics`.
6. `NavigationCommand` still carries PVAJ, yaw, exact identity and lease, certified continuation, role/status, and the two emergency velocity-only authorization inputs. No safety threshold, tracking policy, world policy, Hold policy or planner algorithm is changed.
7. The 100 ms adapter command receive lease, 200 ms adapter state freshness, 500 ms runtime freshness and downstream Hold fence remain unchanged. SITL and component evidence must verify end-to-end parity; source structure alone is not qualification.
