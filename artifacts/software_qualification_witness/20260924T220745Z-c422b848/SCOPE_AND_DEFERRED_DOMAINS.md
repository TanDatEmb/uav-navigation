# Scope and deferred domains

C0-SW judges the repository-owned software decision and its evidence. It does not assert that Gazebo/PX4 tracking or measured stopping meets a flight-performance policy. Existing product safety gates still act: a stale state or excessive terminal anchor must cause the prescribed software response. A correct fail-closed response can coexist with a physical-performance failure.

| Axis | C0-SW role |
| --- | --- |
| Product logic, authority identity, temporal safety, evidence completeness | Required and separately reported |
| Fault handling | Required as a campaign; per nominal-run axis remains `SUITE_EVIDENCE_REQUIRED` |
| Flight performance | `DEFERRED_C0_IFP` |
| Environment validity | `DEFERRED_SIMULATION_VALIDITY` |

The five primary runs completed their missions, but their original integrated runner verdict was `FAIL`. This remains visible in `FRESH_COHORT.csv`; C0-SW reevaluation does not amend integrated flight qualification. The verification cohort contains two `FAILED_COMPONENT` and one `PAUSED_SAFETY_STOP` result, all retained in the denominator and marked `NOT_EVALUABLE` for per-run C0-SW outcome.
