# Execution world certificate contract

## Current source-backed contract

An executable candidate carries immutable world identity and validation capability. `ExecutionAuthority` owns the active/staged bundle and its exposure disposition; `WorldModel`/`WorldSnapshotStore` own the currently published immutable world. A newer world does not by itself prove an old path unsafe: runtime validates the exact active and pending bundles against the new snapshot outside owner locks, then `publishAndFinalizeDecision()` gates the world pointer and dependent execution mutation together.

For a freshness-only source-time expiry, runtime conditionally suspends the exact active execution. The identity/bundle is retained; publication is unavailable until a fresh world recertifies that exact execution. A failed current-world certificate invalidates the exact execution according to existing fail-closed behavior. A stale world callback must not mutate a newer execution or newer published world; existing version/pointer fencing plus the three added latch tests cover active replacement, pending replacement, and suspended-execution replacement at the execution-owner component boundary.

Freshness uses the existing ROS-time source-stamp contract and 500 ms window. The active certificate identity is generation/revision/source stamp plus the exact immutable candidate and localization epoch. Validation time and source-observation time remain separate concepts.

## Limits

- The source contract does not presently expose an independent steady receive/integration/publication lease for the world stream.
- No isolated mapping-only source-stale SITL fault has been run; therefore the end-to-end chain from source silence through runtime suspension, command stream, adapter lease and PX4 Hold is unproven here.
- No dedicated producer-owned C0-SW world transaction lifecycle events or world-specific loss accounting are present in the evidence stream.
- Recovery is bounded by exact fresh-world recertification in source/component semantics; the full live runtime recovery sequence is not established by this artifact.

See `WORLD_RACE_MATRIX.md` and `OPEN_FINDINGS.md` for gates that remain open.
