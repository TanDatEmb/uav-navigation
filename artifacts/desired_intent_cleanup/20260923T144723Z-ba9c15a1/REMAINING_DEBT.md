# Remaining boundary evidence debt

These items are outside the desired-intent cleanup. Their policies and thresholds were not changed by this architecture work.

1. **Isolated world source-staleness and revocation:** source/component tests cover world recertification and revocation, but the cut lacks an isolated runtime/SITL injection proving the stale-source boundary end to end. The focused pillar attempts did not reach the historical world-revision invalidation point; an earlier high-anchor-error/no-certified-suffix path stopped them. Do not treat those runs as evidence that source-staleness is qualified.

2. **Emergency path and PX4 Hold ordering:** deterministic component coverage does not establish an isolated emergency-candidate SITL path or every Core/PX4 Hold callback and `VehicleStatus` ordering. Prior Core-pause evidence observed stale-PVA Hold behavior, and prior injected BACKUP evidence observed safety-suffix ownership and measured restart; neither closes emergency-path or takeover-ordering evidence. Hold policy and adapter-local admission remain unchanged.

3. **Sample 347 adapter pre-validation reject:** nominal session `135131-418913` had one transient pre-validation sample rejection between admitted neighbors. No lease, identity, or continuity rejection was attributed to it, but the exact adapter precheck cause was not logged. Preserve this as a diagnostic debt; it does not justify changing an admission gate.

4. **Diagnostic latency tail:** the previous execution-authority cut recorded one 801 us transport-publish maximum versus 176 us in its repaired reference, with no setpoint or lease gap and no causal attribution. This remains an observed tail, not a demonstrated regression or performance claim.

The evidence gaps above are not product behavior changes and are not flight qualification claims. See the pinned execution-authority cut's `REMAINING_DEBT.md` and `BASE_PROVENANCE.md` for the prior boundary observations and PX4 checkout provenance.
