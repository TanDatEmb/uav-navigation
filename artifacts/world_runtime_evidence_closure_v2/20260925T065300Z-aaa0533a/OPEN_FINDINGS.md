# Open findings

| ID | Finding | Class | Required closure |
|---|---|---|---|
| WRE2-01 | Isolated world source-stale and fresh recovery SITL not run. | RUNTIME_EVIDENCE_GAP | A test-only mapping-observation relay fault that pauses only this stream, with live odometry/health/Core/clock/adapter witnesses; preserve raw session and show suspension, command behavior, lease and recovery. |
| WRE2-02 | World transaction lifecycle is absent from the loss-accounted C0-SW evidence stream. | EVIDENCE_CONTRACT_DEFECT | Producer-owned world transaction witness with sequence/identity/disposition plus writer drop accounting and an evaluator-ready independent metric; do not infer from command samples. |
| WRE2-03 | Unsafe revision invalidation and same-source-tick revision are component/source covered but not a focused integrated runtime observation. | RUNTIME_EVIDENCE_GAP | Controlled revision advance and exact runtime transaction trace. |
| WRE2-04 | The world identity has no independent steady receive/integration/publication freshness fact. | WORLD_CONTRACT_EVIDENCE_GAP | Define whether source-time freshness is sufficient or whether publication progress needs its own contract; no threshold invented here. |
| WRE2-05 | Same-epoch mapping backend rebuild/bag rewind generation semantics remain unspecified in live producer path. | POLICY_OR_CONTRACT_GAP | Establish whether these reset modes are supported and define identity progression before changing producer behavior. |

No threshold, world traversal policy, unknown policy, command lease, adapter behavior, or PX4 policy was changed. No product defect was proven by this source/component-only work.
