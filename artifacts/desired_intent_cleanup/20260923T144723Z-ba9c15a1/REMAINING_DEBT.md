# Remaining boundary evidence debt

These items are outside the desired-intent cleanup. Their policies and thresholds were not changed by this architecture work.

1. **Isolated world source-staleness and revocation:** source/component tests cover world recertification and revocation, but the cut lacks an isolated runtime/SITL injection proving the stale-source boundary end to end. The focused pillar attempts did not reach the historical world-revision invalidation point; an earlier high-anchor-error/no-certified-suffix path stopped them. Do not treat those runs as evidence that source-staleness is qualified.

2. **Emergency path and PX4 Hold ordering:** deterministic component coverage does not establish an isolated emergency-candidate SITL path or every Core/PX4 Hold callback and `VehicleStatus` ordering. This cut's Core-pause fault observed stale-PVA Hold behavior, and its injected BACKUP run observed safety-suffix ownership and measured restart; neither closes emergency-path or takeover-ordering evidence. Hold policy and adapter-local admission remain unchanged.

3. **Isolated adapter pre-validation rejection:** prior nominal session `135131-418913` rejected sample 347 between admitted neighbors. New-cut nominal session `151450-509084` also has one `valid=0 command_present=1` pre-admission rejection, with subsequent admission and no lease, identity, or continuity failure. The exact precheck cause is still not separated by the log. Preserve this diagnostic debt; it does not justify changing an admission gate.

4. **Diagnostic latency tail:** the previous execution-authority cut recorded one 801 us transport-publish maximum versus 176 us in its repaired reference, with no setpoint or lease gap and no causal attribution. This cut measured maxima `72/280/62 us` across its three nominal sessions. These limited samples do not prove a performance improvement or a worst-case deadline.

The evidence gaps above are not product behavior changes and are not flight qualification claims. See the pinned execution-authority cut's `REMAINING_DEBT.md` and `BASE_PROVENANCE.md` for the prior boundary observations and PX4 checkout provenance.
