# Target state proposal and admission gate

This is a critique model, not a committed product design. The independent domains are:

1. **MissionProgress:** `MeasuredRouteCursor` carries route revision, localization epoch, segment index, segment fraction, arc length, measurement source time and a bounded previous measured segment. `AcceptedMissionGate` carries the last policy-accepted waypoint/request and acceptance witness. A self-intersection gives multiple projections for the same position; projection needs route order, prior cursor and ambiguity handling. A pinned nondegenerate arc can identify a segment, but a monotonic scalar alone cannot reconstruct the current measured projection after reverse motion. Odometry gaps invalidate a crossing witness, not the accepted gate. STOP acceptance uses measured position/speed/confirmation rather than projection alone.
2. **ExecutionAuthority:** desired intent, active immutable bundle, optional staged successor, certified world/lease, and one lifecycle decision record. A possible sum type is `NoExecution | Executing | Stopping | Holding | ReleaseRequested`; it must also carry orthogonal staged successor and lease. MAIN/BACKUP are roles of bundle samples; sampling BACKUP may trigger global stopping only after an explicit product decision. Current adapter source permits pre-stop nominal recovery, so one-way stopping is a proposed policy change.
3. **PX4Boundary:** last locally admitted reference/receive witness, local state/health/frame/reset witnesses, External Mode lifecycle and a Hold protocol state. `Inactive | FollowingReference | HoldRequested | HoldInFlight | ApiCompleteAwaitStatus | HoldConfirmed | AuthorityLost` is a candidate, with retry deadline/context. `ApiCompleteAwaitStatus` is required if API Success and VehicleStatus are independent; Deactivated is not simply Success.

`PlanningContextId` may be a derived equality key over semantic mission/intent/localization/world/dynamics/active-predecessor identity. Keep raw identities in result and evidence; world recertification can change world identity without replacing trajectory semantics, so key construction and invalidation must be specified.

## State Admission Test — mandatory for each proposed persistent field

Record answers and evidence for all twelve before accepting a new field: (1) independent fact/decision; (2) exact derivability from authoritative state plus current observation; (3) why it survives an event; (4) sole writer; (5) start event; (6) invalidation; (7) mutual exclusion; (8) why enum/variant cannot replace booleans; (9) product or diagnostic authority; (10) why replicated cross-process state rather than event; (11) physical/protocol information lost on deletion; (12) invariant/fault test. Missing evidence means `UNRESOLVED` and no target admission.

## World lease and command lease

Proposed semantics: continue only while the **last certified world** and active command certificate remain inside their declared freshness/lease; asynchronously recertify on a new world; if certification cannot finish before expiry, transition to stopping via a certified stop or PX4 Hold. Once stopping is activated, a late nominal result cannot restore nominal until a measured stop **if that one-way policy is approved**. No new `world_valid` boolean is needed: use certificate identity, expiry and execution variant. Queue delay, map update latency and recertification tails must be measured before this replaces current suspend/resume behavior.

## PASS_THROUGH event semantics

Crossing is an observation with route/epoch/segment/time identity. Acceptance is a separate mission policy decision. Successor readiness is execution state. A crossing before readiness must either remain as a bounded, independently valid measured event or be explicitly rejected by policy with a reason; silently overwriting prior sample is not equivalent. Cases requiring tests: witness first/crossing first, high-speed ball skip, sharp corner, coincident PASS→STOP, BACKUP active, map revision, self-intersection, reverse, odometry gap, localization reset.

## Formal scope

`tests/target_model.py` explores event interleavings for intent/active/staged, measured crossing, world version/certificate, lease expiry, stopping and Hold API/status. It checks no stale plan changes active, no acceptance without crossing, no nominal exit from stopping before measured stop, and no Hold confirmation from API completion. The model is deliberately small; it does not prove geometry, queue timing, ROS/PX4 transport or equivalence with TARGET.
