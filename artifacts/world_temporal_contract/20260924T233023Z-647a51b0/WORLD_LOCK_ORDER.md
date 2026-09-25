# World lock order (AS-IS)

## Mapping world publication path

The runtime comment and lexical scopes establish:

```text
localization_transition_mutex_
  → input_mutex_
    → command_execution_lease_failure_latch_.transitionMutex()
      → WorldSnapshotStore::publication_gate_
        → ExecutionAuthority::mutex_
```

The `ExecutionAuthority` method performs optimistic snapshot/preparation work outside its mutex and takes it again for final compare/mutate. Candidate active/pending `validateWorld`, change-history scanning, ROS clock reads, allocations/copy preparation, and endpoint freshness calculation are outside the lock chain.

## Command freshness suspend path

Runtime captures an `ExecutionAuthoritySnapshot`, reads world/state witnesses, then acquires the localization/input/latch transition locks and calls `suspendIfCurrentSnapshot`. Publication authorization uses owner snapshot/token revalidation to linearize against world recertification, activation and fail-close.

## Risks and evidence

No new mutex is proposed. Existing store test `PublicationCannotInterleaveAnAuthorizedCommit` uses promises/barriers and proves the publication gate linearizes commit. Existing runtime tests cover mapping/reset drain ordering and world recertification races; this audit still needs a cross-path lock-order/deadlock test or explicit proof that no reverse acquisition path exists. Full candidate validation must never be moved inside publication or authority locks.
