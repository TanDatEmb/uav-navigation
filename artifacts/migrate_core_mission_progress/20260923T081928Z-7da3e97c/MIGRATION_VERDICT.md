# Migration verdict

`MISSION_AUTHORITY_CUT_FAILED`

The source cut achieved one product-path mission writer: Core loads the immutable mission, owns measured route progress and accepted gates, increments request identity, creates successor goals internally, and publishes completion. The PX4 adapter no longer instantiates or links `MissionController` and has no successor-goal publisher. Component/model tests passed, including ordered PASS crossing and measured STOP. The adapter retained local freshness, health, frame, command lease, tracking and Hold authority.

The completion gate failed in focused SITL. With the same `long_featured` profile, seed 0, external PX4 checkout and tracking configuration, the pinned baseline completed 2/2 missions while this migration completed 0/3. Migration runs stopped safely on unavailable or stale planner commands after accepted prefixes `[0,1]`, `[0]`, `[0,1]`. A single root cause is not established; source changes in mission event timing, command admission and adapter boundary need targeted trace comparison. The result is a differential liveness regression signal and satisfies the task's `FAILED` criterion for an unresolved correctness defect. Safety Hold worked; that does not make incomplete mission progression acceptable.

The previous shadow evidence was diagnostic and is not promotion evidence for the migrated binary. The focused SITL reports are also qualification-ineligible because the existing simulated zero-coefficient tracking policy produces a bypass/config mismatch. No threshold was weakened or bypass added in this cut. A `PASS` cannot be inferred from component tests or the lease fault alone.

Keep this branch as a reviewable failed implementation and evidence set. Do not merge/deploy it as a completed architecture cut. The next work is a focused repair of the mission-to-execution liveness seam using synchronized Core GOAL/admission/candidate/world/lease traces, followed by the same matched baseline/migration runs. No second architecture migration is authorized by this verdict.
