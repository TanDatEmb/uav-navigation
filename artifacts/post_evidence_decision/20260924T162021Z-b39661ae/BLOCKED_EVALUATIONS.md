# Evaluations blocked by the selected path

| Evaluation | Current disposition | Why it remains blocked |
|---|---|---|
| Integrated C0 tracking coverage | `NOT_EVALUABLE` | Missing approved, versioned coverage/pairing policy; lineage is incomplete independently. |
| Integrated C0 truth-relative tracking acceptance | `NOT_EVALUABLE` | Missing approved four-limit acceptance policy and complete reference lineage. |
| Measured-motion acceptance | `NOT_EVALUABLE` | No approved metric/filter/role/phase acceptance contract. |
| Integrated `qualification_eligible` | false in 10/10 raw replays | Tracking/motion policy missing, 140 unresolved lifecycle transactions, 5,957 unjoined references. |
| Historical A3 same-stamp changed-world reference | `SOURCE_TIMESTAMP_DUPLICATE_CONFLICT` | Certificate revision changed at one source tick; observer order cannot authorize a correction. |

Timestamp semantics for exact PVA heartbeat and strictly increasing measured odometry are defined; they are not a substitute for missing policy. All ten original outcomes remain in the denominator: eight `COMPLETE`, two `PAUSED_SAFETY_STOP`. No new nominal cohort or new flight-performance conclusion is produced here.
