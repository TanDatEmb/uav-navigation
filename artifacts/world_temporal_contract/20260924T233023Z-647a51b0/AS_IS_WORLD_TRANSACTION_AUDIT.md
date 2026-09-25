# AS-IS World transaction audit

**Audit baseline:** `647a51b060a6b2d0f66b6eb3c69dd990fd20986a`.
Source is authoritative; ADR text is design/history evidence only.

## Ownership answers

| Question | Source-derived answer |
|---|---|
| Who owns mutable map integration? | `navigation_mapping::MappingActor::Impl` owns the mutable `RuntimeMappingMap`, local world counters, changed-region accumulator and current immutable snapshot. `process()` validates and integrates one `MappingObservation`. The mapping backend is not shared with planner queries. |
| Who owns immutable world publication? | `navigation_mapping::WorldSnapshotStore` owns an atomic pointer to the latest immutable view and its `publication_gate_`. Runtime composition holds the store; mapping callback asks it to publish. |
| Who owns world identity? | `MappingActor` constructs identity `(localization_epoch,generation,revision,observation_stamp_ns)` and embeds it in each immutable snapshot. Store enforces publication ordering. ExecutionAuthority separately records the world identity against which its current active/staged authority has been transactionally reconciled. |
| Who owns active/pending execution? | `ExecutionAuthority` owns active and staged bundles/goals, activation time, world identity, lineage/version, admission scope and lifecycle. Runtime obtains immutable snapshots and calls bounded owner operations. |
| Who validates active execution against a new world? | The runtime mapping callback uses the exact active `CandidateBundle`: a disjoint changed-region proof is a fast path; otherwise `CandidateBundle::validateWorld(new_snapshot, authorization_wall_time_s)` invokes the bundle's immutable world validator outside owner/publication locks. `ExecutionAuthority` only accepts the result tied to the exact observed active/pending pointers and timeline version. |
| Who may revoke active execution? | `ExecutionAuthority::publishWorldIdentityIfCurrentAndFinalizeRevocationImpl` clears/fails the exact active authority when no exact active owner is expected or its preparation/revalidation failed, conditional on current timeline version, active pointer, pending pointer and prior world identity. A finalizer performs exact runtime side effects only for the observed owner. Other runtime safety paths may also revoke through ExecutionAuthority APIs. |
| Who may discard pending execution? | ExecutionAuthority world transaction independently retains a validated pending copy or drops the exact staged record. A pending validation failure does not itself revoke a valid active bundle. Runtime invalidation cleanup clears `PendingGoalOwner` only by exact pointer via `clearIfCurrent`. |
| Who may suspend command publication? | Runtime detects stale world source time in command/planning paths and calls `suspendCommandForWorldFreshness`, which uses an exact execution snapshot/token and `ExecutionAuthority::suspendIfCurrentSnapshot`. The adapter has its own independent command receive lease. |
| Who may resume a suspended command? | Runtime mapping callback, after a committed exact-world recertification, checks `worldFreshnessSuspendedCommandMayResume`, then under lifecycle locks rechecks active goal/bundle identity, localization epoch, validity, failure latch and command-exposure latch before `observeRetainedCommand`. Freshness alone is insufficient. |
| Which locks linearize each transition? | Mapping publication: `WorldSnapshotStore::publication_gate_`. Execution transaction: `ExecutionAuthority::mutex_`, comparing timeline version and exact bundle pointers before mutation. Runtime cross-owner ordering in the mapping callback: `localization_transition_mutex_ → input_mutex_ → command_execution_lease_failure_latch_.transitionMutex() → publication_gate_ → ExecutionAuthority mutex`. Candidate/world validation is performed before those locks. |

## Source path and current contract

`MappingActor::process()` creates a new immutable `MappingWorldSnapshot` after successful observation integration. Runtime mapping callback snapshots active and pending execution, performs changed-region proofs or complete immutable revalidation outside locks, then calls `WorldSnapshotStore::publishAndFinalizeDecision()`. Its finalizer calls ExecutionAuthority's conditional world transaction. The store publishes the new pointer only if finalization returns `kCommitted`; otherwise old world remains published.

`kSuperseded` is explicitly treated as optimistic concurrency: telemetry is recorded and callback returns without mutating newer execution. Every other non-committed result currently triggers `command_store->invalidate()` and throws to fail-stop the mapping worker. This is observable source behavior, but the safety and recovery rationale for each returned decision has not yet been separately proven.

On world invalidation, ExecutionAuthority removes exact active and pending execution, fails its lifecycle, and invokes runtime finalizer. Runtime finalizer consumes hot retarget, clears replan skip and terminal/completion witnesses; exact pending goal is then conditionally cleared. On freshness-only suspension, active identity is retained, exposure becomes suspended, and future resume requires a fresh committed world plus exact recertification.

## Provisional gaps for subsequent audit

1. `CandidateBundle::valid_until_ns` is used by sample validity, command authorization, candidate admission, splice/activation and world recertification. Recertification may advance it to `now + data_freshness_window` but caps it at declared analytic endpoint. The complete producer/consumer audit is still required before claiming the field has one coherent semantic.
2. `WorldSnapshotIdentity` contains source observation stamp but no receive/integration/publication timestamp. Source freshness therefore proves freshness of source evidence, not independently that mapping publication is progressing.
3. Mapping callback's generic non-committed fallback performs unconditional command invalidation after the finalizer returned; each possible decision needs reachability and policy classification.
4. `MappingActor` increments generation on observed localization epoch advance; no same-epoch backend reconstruction/generation increment was found in the first pass. ADR-011's broader reset/bag-rewind/config-reset statement is not yet fully evidenced by the current producer path.
5. Fresh-world resume semantics are implemented as automatic resume after exact successful recertification and latch checks. Confirm against accepted product contract and runtime evidence before treating this as fully specified policy.

## Source locations

- `src/mapping/navigation_mapping/src/mapping_actor.cpp`: mutable owner, observation validation/integration and snapshot identity production.
- `src/mapping/navigation_mapping/include/navigation_mapping/world_snapshot_store.hpp`: publication gate and monotonic identity.
- `src/mapping/navigation_mapping/include/navigation_mapping/mapping_world_snapshot.hpp`: immutable snapshot and change-history proof.
- `src/planning/navigation_planning/include/navigation_planning/candidate_bundle.hpp`: bundle identity, validity and immutable world validator.
- `src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp`: world identity transaction and active/pending lifecycle mutation.
- `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`: mapping callback, freshness suspension/resume and command authorization.
- `docs/adr/ADR-010-navigation-world-model.md`, `docs/adr/ADR-011-world-snapshot-ownership.md`: design intent and prior evidence state.
