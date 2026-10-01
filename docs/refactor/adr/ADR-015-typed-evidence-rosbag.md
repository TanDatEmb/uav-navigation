# ADR-015: Typed evidence as ROS msgs recorded in rosbag2 (MCAP)

**Status:** accepted; implementation in P2 (bag storage switch in P0).

## Decision

- New package `nav_evidence_msgs`. One msg type per evidence family, for example
  `LifecycleEvent`, `CommandAdmissionEvidence`, `WorldTransactionEvidence`,
  `RetainedDecisionEvidence`, `PlannerCycleEvidence`,
  `SetpointInputEvidence`, `ConfigWitness`. The final list comes from WP-A2.
- Every enum is a msg constant with an **explicit value**; values are never
  reused. Every evidence msg carries `producer_id`, `producer_instance_id`,
  `sequence` (monotonic per producer), `source_stamp` + `clock_domain`,
  `schema_version`, and the `nav_safety_profile` hash.
- Topics are `/evidence/<family>`, QoS reliable, keep-last depth ≥ 1000,
  volatile. Recording uses rosbag2 with `--storage mcap`.
- The judge reads bags with the pure-Python `rosbags` library (no ROS needed
  in CI) through decoders generated from the msg definitions.
- `DiagnosticArray` stays observability-only. During P2 both channels are
  emitted (dual emit); the string channel is removed once P2 closes.
- Gap detection: sequence gaps per producer are evidence loss and make the
  result `NOT_EVALUABLE`, never PASS.

## Rationale

RC4, V7, R-01/R-05/R-07: string parsing and ordinal comparison let the judge
drift from the product. rosbag2 is already the raw-evidence store (runner
`_start_runtime_evidence_bag`, currently sqlite3), so no second logging
mechanism is introduced.

## Consequences

Bag size grows; this is accepted for beta (SITL only). The existing evidence
topic list (`RUNTIME_EVIDENCE_TOPICS`) is extended, not replaced.
