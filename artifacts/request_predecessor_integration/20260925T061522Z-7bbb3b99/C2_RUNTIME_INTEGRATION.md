# C2 request-predecessor runtime integration

## Contract

`PlanningHistory` must describe the exact active immutable execution captured at the request boundary. The predecessor is not the desired successor identity, and planner warm-start state cannot grant the emergency correction exception.

## Producer correction

`runCycle()` captured `transition_bundle` early for scheduling and retained-command decisions, then later captured `request_timeline` and checked its active generation against the scheduled `PlanningKey`. It previously populated predecessor evidence from the earlier pointer. The request now builds its history from that same `request_timeline` snapshot used by the admission guard. The named helper `NavigationRuntimeNode::makePlanningHistory()` copies only generation, localization/goal/request identity, kind/role, measured prior velocity, and—only for an emergency bundle—a finished finite declared endpoint. It does not retain a `CandidateBundle` or validator callable.

If active identity changes before the snapshot check, the existing scheduled-generation guard discards the request. If it changes after the snapshot, normal request/candidate fences remain responsible for discarding stale work.

## Consumer correction

The bounded route-correction predicate is now the single production function `authorizeEmergencyCorrection()`, called from `Planner::authorizeAndStage()`. It receives no warm-start/cache argument. The actual planner call and deterministic negative controls exercise this same gate. Existing candidate world, dynamics, yaw, anchor, and handoff validation remains downstream and unchanged.

## Runtime proof

`NavigationRuntimeTerminalMonitorStrict.AdmittedEmergencyWitnessAuthorizesRealStopCorrectionRequest` drives a real `NavigationRuntimeNode` fixture through:

1. a certified STOP predecessor and current measured state;
2. a real `runCycle()` tracking-envelope emergency transition;
3. an admitted `CandidateBundleKind::kEmergencyBrake` in `ExecutionAuthority`;
4. current-world recertification and a measured endpoint state;
5. the same request-history producer used by `runCycle()`;
6. a valid `PlanningRequest` with predecessor evidence and current request identity;
7. the real `PlannerFacade::plan(request)` path, including `authorizeAndStage()` and the route-regression certificate;
8. an admitted terminal STOP candidate ending inside the declared acceptance ball.

The test asserts that the emergency endpoint is outside the ordinary acceptance radius, the request witness generation/kind match the active owner record, and the output endpoint returns inside acceptance. The test records predecessor generation, localization epoch, goal epoch, request ID, kind/role, endpoint coordinates, request world generation/revision, and corrected endpoint error as gtest properties.

This is deterministic runtime/component evidence with the actual runtime node, execution owner, request producer, planner facade, and candidate authorization. It is not PX4/SITL, firmware-consumption, or flight evidence.

## Identity separation and substitutions

- Planner authorization tests use predecessor goal/request IDs that differ from the successor request key; the exact predecessor generation and localization still bind the exception.
- False-accept control supplies an ordinary request predecessor while describing a later emergency cache state in the test narrative. The production authorization interface has no cache input and denies the exception.
- False-reject control retains emergency E1 in the request while the test models a later ordinary cache E2. The same production predicate permits E1's bounded correction.
- Wrong candidate localization/goal/request identity, wrong predecessor generation/localization, missing endpoint, wrong kind/role, invalid correction endpoint, and out-of-budget geometry all deny the exception.

No synthetic success value is written into a candidate or certificate. The new function only evaluates the existing request-owned witness and correction geometry.
