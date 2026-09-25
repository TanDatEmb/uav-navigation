# Request predecessor evidence contract

`PlanningPredecessorEvidence` is a value witness in `PlanningHistory`, not a `CandidateBundle` pointer. It contains bundle generation, localization epoch, predecessor goal epoch/request ID, bundle kind/role, and an optional declared endpoint. Validity requires nonzero identity, a valid kind-role pairing, and finite endpoint when present; emergency predecessor evidence requires the endpoint.

Runtime builds it from the exact immutable `transition_bundle` obtained from `ExecutionAuthority` when constructing the request. For emergency braking, it uses `sampleAtDeclaredEnd()` and accepts only a finished finite declared endpoint. `PlanningRequest::valid()` binds the witness to the history generation and request localization; committed-future requests also bind their anchor to the exact active predecessor. Predecessor goal/request identity is allowed to differ from successor identity.

`Planner::authorizeAndStage` receives the immutable request context explicitly. The bounded emergency route-correction exception requires request identity to match the candidate command identity, then consumes `request.history.emergencyEndpointFor(...)` against the request's committed bundle generation and localization epoch. A missing/mismatched witness grants no exception. Candidate world, dynamic, yaw, anchor and handoff checks still apply.

No full CandidateBundle or callable validator is copied into PlanningHistory. `previous_bundle_generation` and `previous_velocity_world` remain because they have other request-contract consumers. No mutable planner request mirror was added.

Deterministic tests cover malformed/missing witnesses, generation/localization mismatch, kind/role/endpoint validity, cache substitution false accept/reject scenarios, and predecessor/successor identity separation. The latter are request-contract tests; the nominal SITL cohort did not execute a deliberately forced emergency-brake-to-correction case, so that specific integration behavior remains unobserved at runtime.
