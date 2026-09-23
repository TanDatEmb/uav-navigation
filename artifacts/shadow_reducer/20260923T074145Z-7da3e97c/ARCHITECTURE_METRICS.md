# Architecture metrics at pinned baseline

Counts have different denominators and are not presented as a direct whole-product LOC comparison.

| Metric | AS-IS pinned candidate inventory / source | Shadow model |
|---|---:|---:|
| Persistent behavioral candidate fields | 239 across 15 declaring types | 16 top-level semantic slots (Mission 6, Execution 5, PX4 5), plus 8 nested Hold protocol slots and 1 diagnostic quality slot = **25** mutable slots; immutable typed witness payloads additionally retain necessary information. |
| Persistent bool candidates | 54 exact `Type=bool` in inventory | 0 boolean state slots. |
| Lifecycle state enums/FSMs in scoped model | MissionControllerState, ExecutionEpisode phase/recovery and PX4 transfer booleans coexist; complete repository enum count not measured | 1 ExecutionPhase enum; Hold remains orthogonal protocol witnesses, not a flattened enum. |
| Authority owners | Core execution, MissionController mission, NavigationMode adapter mirrors, PX4 executor Hold, WorldModel latest world (5 semantic owners in scoped flow) | Core mission+execution writer, PX4 boundary, WorldModel boundary (3), with computation workers authority-free. |
| State-writer surfaces | 15 declaring types in the 239-candidate inventory; direct writer-function count is not established from declarations alone. | One serialized reducer writer for each of three domains; `HoldTransfer` is mutated only through PX4 boundary events. |
| Identity predicate search | 110 matching lines for selected `*epoch/*revision/*request_id/*world_generation ==` patterns in `navigation_runtime_node.cpp` and `navigation_mode_node.cpp`; lexical lower bound, not all sites | 1 `ContextKey` equality in stale staging admission plus matching typed gate/continuation identities. |
| Proven identity simplification | At least one multi-factor stale-result admission decision in the scoped execution flow; 110 lexical sites are candidates, not deletion proof. | One `ContextKey` equality replaces the modeled stale-stage admission check. Exact product-site deletion count belongs to the first migration diff. |
| Cross-owner transition width | PASS acceptance and Hold involve at least Core↔MissionController↔adapter/PX4; exact object-level maximum not remeasured here | Mission acceptance 1 Core-owned object; execution activation 1 Core-owned object; Hold 1 PX4-owned protocol object plus explicit Core release event. |
| Mutexes | 10 direct `std::mutex` declarations in three selected Core/adapter/MissionController headers; subset only, not all semantic authority mutexes | 0 in offline Python reducer. This is not a runtime contention claim. |
| Mutexes on semantic authority transitions | Lower bound 3 distinct: `MissionController::mutex_`, `NavigationMode::trajectory_mutex_`, and Core `command_execution_lease_failure_latch_.transitionMutex()` (source headers and Core call sites). This is not a full lock graph. | Shadow transitions are serialized in one offline loop; production would still need boundary synchronization. |
| Control-plane replicated facts | at least four scoped replicated fact groups (mission intent, accepted gate, execution lifecycle, Hold transfer) each appear in ≥2 mutable representations; group-level D_f lower bound 2, exact all-field D_f not claimed | typed cross-process receipts are read-only observations; Core policy facts have one writer. |

The strongest **measured reduction** is 17 target deletions + 14 derivations + 22 merges among 239 candidates, while retaining 158 conservative boundary/computation fields and 10 unresolved. The model's 25 mutable slots describe only the three authority/protocol domains, so 239→24 must **not** be quoted as a whole-product state reduction. No performance or latency improvement is claimed.
