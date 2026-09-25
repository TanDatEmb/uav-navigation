# Root cause R1 — predecessor publication revoked by desired-gate advance

Observed symptom:
Migration `long_featured` accepted a waypoint prefix, then stopped producing an executable command; the adapter's 100 ms lease expired and PX4 Hold followed. The three retained failed runs reached prefixes `[0,1]`, `[0]`, and `[0,1]`. A fresh traced run reached `[0,1,2]` before the same failure.

First causal divergence:
At sim 41.768 s in `external-mode-check-20260923T104546-270858`, H4/H5 show mission request 4 installed as **desired** goal epoch 5 while predecessor request 3, execution epoch 4, bundle generation 8 remained available and hot-retained. At that same simulated timestamp `publishCommand()` invoked `failClosedLocked()` and cleared execution epoch to zero. The next publication reported `planner_available=0`, `sampled_bundle=0`. This happened before the adapter's stale-command report or Hold request.

Mechanism:
The failed migration's final publication transaction required the sampled `command_goal` to match the mutable `active_goal_` and `active_goal_epoch_`. MissionProgress advanced `active_goal_` to the successor before `ExecutionTimelineStore` cut over from the predecessor. The final check therefore rejected a still-authoritative predecessor sample. Its stale-publication fallback observed the same predecessor bundle and treated this rejected publication as a fatal current-execution failure, clearing `command_goal_epoch_` and the bundle. The Release stack trace in the pre-fix session points from `publishCommand()+0x37b2` to `failClosedLocked()` at the H4/H5 transition. This is an ownership-identity error, not a candidate validator or adapter timeout error.

Why baseline did not fail:
The pinned baseline retains adapter-side MissionController and an asynchronous ROS goal handoff. Its fresh matched seed-0 `long_featured` run accepted all five waypoints and kept predecessor command delivery during the successor handoff. It did not expose the Core-local desired-gate/execution-owner mismatch at the observed boundary. This empirical difference does not prove the old code was immune to every scheduling interleaving.

Why migration failed:
The first architecture cut correctly moved mission acceptance into Core. That made `desired gate N+1` with `executing predecessor N` a normal, lasting state rather than merely an adapter-to-Core ROS timing window. The old final publication predicate still treated desired goal identity as command authority, so it could reject every predecessor sample in that state and then revoke the predecessor.

Fix:
Authorize an in-flight sample against the current **executing** goal, command goal epoch, localization epoch and exact active bundle. Keep the final state freshness, world identity, bundle pointer, certificate, command lease and publication transaction checks. Use the same execution identity in the stale-publication fallback, and preserve fail-closed action for a genuinely stale current execution. The desired mission gate remains independent until atomic execution cutover. The separate ModeStatus heartbeat seam was corrected so an established activation survives a delayed recurring ACTIVE status; it was not the observed R1 first divergence. Exact finite adapter admission receipts and the adapter's local lease still guard continuation.

Why fix preserves single-writer authority:
MissionProgress alone accepts waypoints and creates successor intent. ExecutionTimelineStore/Episode retain and atomically replace the executable bundle. The adapter only admits individual sampled commands under its local safety boundary; it does not write mission progress or goals. No waypoint handoff latch or second mission writer was added.

Why no safety threshold change is needed:
The first failed sample was rejected by a goal-identity predicate even though H5 reported an available retained predecessor. The 100 ms command lease, 200 ms initial mode boundary, 500 ms runtime freshness, planner deadlines, tracking limits, certificates and world policy are unchanged. The defect is the identity used at publication, not an undersized timeout.

Regression test:
`DesiredPassGateAdvanceCannotRevokeInFlightPredecessorSample` checks the exact final-publication identity seam. `EndToEndHotHandoffRetainsPredecessorUntilAtomicCutover` spans MissionProgress, goal creation, execution store/Episode, command sampling and adapter identity behavior, including delayed successor, failed candidate, atomic cutover and late old sample rejection. `EstablishedActivationKeepsMeasuredCrossingWhenModeHeartbeatIsLate` checks the separate mode heartbeat seam. Runtime parity and safety-fence results are recorded in `SITL_RESULTS.md`.
