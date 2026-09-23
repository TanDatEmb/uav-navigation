# Semantic duplication metric

`D_f = number of independently mutable authoritative representations of the same semantic fact`, **after** separating desired from active, measured from accepted, latest world from certificate, command source lease from receive lease, and PX4 ACK/completion/status. Do not count a read-only cross-process observation as a second policy authority. Do not count two fields within one atomic owner twice unless they can independently diverge.

| Proven fact | AS-IS D_f | Why | TARGET expectation |
|---|---:|---|---:|
| active bundle generation | 2 | `ExecutionTimelineStore.committed_` generation and `ExecutionEpisodeSnapshot.active_generation` have separate mutable owners/updates | 1 in an immutable active execution record |
| latest world | 1 | `WorldSnapshotStore.latest_` | 1 |
| Hold request operation | 1 owner with multiple orthogonal facts | pending/inflight/deadline are not interchangeable | 1 owner, still multiple facts |
| Hold observed nav state | 1 | cached `px4_hold_confirmed_`; freshness information is missing | 1 typed witness |
| measured crossing observation | 0 persistent | computed and discarded each update | 1 bounded typed observation if policy approved |

**Highest proven AS-IS `D_f = 2`** for active execution generation within this scoped audit. This is not a whole-product maximum; many clusters remain `?`, including desired intent, safety disposition and localization mirrors. The 239 field count is neither numerator nor denominator of D_f. The target may add fields to preserve missing independent facts while reducing *decision* duplication. No product change or runtime performance improvement follows from this metric alone.

Regeneration: `python3 tools/cluster_inventory.py` from this artifact directory; mapping output is `FIELD_CLUSTER_MAP.csv`. Verify source pin separately with `git diff 7da3e97c -- src docs/safety` (the audit branch contains earlier docs only).
