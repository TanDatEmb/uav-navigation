# Target semantic model V2 — provisional, not approved

**Verdict: NOT_ADMITTED.** This is a semantic candidate after Blockers A–D, not production design or permission to create a shadow reducer. It preserves independent facts rather than generating a Cartesian mega-state.

## Domains and transitions

**Mission domain:** `DesiredMissionIntent` (requested route/waypoint/request), `MeasuredRouteState` (ordered cursor, previous source-stamped measurement), optional bounded `WaypointCrossingObservation`, and `AcceptedMissionProgress`. Physical crossing and accepted gate are distinct. The mission decision may accept a matching crossing plus exact current readiness and emit a successor request; it does not activate execution.

**Execution domain:** one typed lifecycle candidate `NoExecution | Tracking | RecoverableBraking | CommittedStopping | Stopped | ReleasingAuthority`, with *orthogonal* immutable active bundle, optional staged bundle, current certificate and source lease. Sampled `CandidateRole` and bundle kind remain orthogonal to global lifecycle. `RecoverableBraking` is not admitted until the B policy/reachability proof closes. A committed stop cannot be replaced by late MAIN unless policy explicitly redefines commitment. Desired and active execution identities are separate during hot retarget.

**World domain:** WorldModel owns latest immutable snapshot. Active execution carries the certified world identity, validity interval and validation result. New world observation can coexist with an older still-valid active certificate. A recertification job has its own identity if asynchronous policy is approved; job existence is not a `world_valid` flag. The exact expiry action depends on B and timing evidence.

**PX4 boundary:** last locally admitted reference and source/receive lease; local state, health, frame and reset witnesses; `HoldTransfer` request/ACK/scheduled-operation/completion/retry facts; separately stamped `VehicleStatusWitness`; External Mode activation/deactivation and operator/failsafe takeover. The adapter may reject stale references; it does not mirror planner or mission policy without an explicit independent need.

## Transition constraints

1. New goal changes desired intent; active bundle and certificate remain identified until a successor admission transaction.
2. PASS acceptance consumes a valid physical observation and exact readiness; successor activation is a later independent execution event.
3. Braking policy transition requires measured-state continuity and a certificate; BACKUP sample role alone cannot select recoverability.
4. World recertification updates active certificate only with exact execution identity; failed validation selects certified brake/stop or Hold, never an unmarked publication gap under a future heartbeat policy.
5. Hold request, command ACK, ModeCompleted, fresh AUTO_LOITER status, and executor charge/takeover are orthogonal. No callback alone grants flight authority.

`SPECIFICATION_GAP`: bounded crossing retention; B public API intent/future pre-commit policy (product call graph currently does not invoke `onTrajectory`); C firmware/status witness; D WCET and lease feasibility. Because these high-criticality facts fail admission, **SHADOW_REDUCER_READY = NO**. No `NavigationCore`, reducer, new `ControlReference`, or `MissionProgress` production type was added.
