# Exact duplicate source stream

The duplicate reason in A2/A3 comes from `pva_command`, the evaluator's
**navigation reference**, rather than `propagated_odometry` or
`ground_truth_odometry`. Raw monitor rows for both odometry streams had zero
adjacent duplicate source stamps in each run. The PVA stream had 15 extra
rows across three same-epoch timestamp groups in A2 and 18 extra rows across
four groups in A3.

Examples from A2, source clock `ros_time`, epoch `11557321553903`:

| Source stamp ns | PVA sample IDs | Request / bundle | Raw record sequences | Observation |
| ---: | --- | --- | --- | --- |
| 36179999999 | 1061–1062 | 3 / 11 | 5281, 5285 | Same trajectory time and PVA. |
| 38996000000 | 1237–1242 | 3 / 13 | 6124, 6129, 6132 and later | Same trajectory time and PVA. |
| 43043999999 | 1490–1499 | 4 / 14 | final command heartbeat group | Same trajectory time and PVA; sample 1499 was rejected for odometry receive staleness. |

The evaluator's `_source_time_status(reference)` applies a strict-increase
rule to every published command row and emits `SOURCE_TIMESTAMP_DUPLICATE`.
The repeated command rows have increasing sample IDs and repeated sim-time
reference states while the command timer continues in steady time. In A2,
all 15 extras are exact same-state/certificate heartbeats. In A3, two rows
change `world_revision` at the same source stamp (194→195 at 23611999999 ns,
244→245 at 28731999999 ns); they are **not** exact certificate duplicates.
This explains the evaluator reason; it does **not** explain loss of adapter
odometry.

The focused evaluator repair collapses only exact same-state/certificate
heartbeats for time-indexed tracking metrics, while preserving every raw row
and checking each raw sample's lifecycle lineage. A conflicting duplicate
still fails qualification. Replay of pinned A2 now accounts for 1499 raw /
1484 canonical references and removes its duplicate reason. A3 accounts for
2919 raw / 2903 canonical references, leaving both
`SOURCE_TIMESTAMP_DUPLICATE_CONFLICT` and `SOURCE_TIMESTAMP_DUPLICATE` for the
two world-revision changes. Both remain NOT_EVALUABLE for other gaps.
