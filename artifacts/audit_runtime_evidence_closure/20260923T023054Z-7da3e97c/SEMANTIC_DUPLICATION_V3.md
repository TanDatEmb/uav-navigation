# Semantic Duplication Factor V3

`D_f` counts independently mutable **authoritative** representations of one semantic fact. Distinguish desired/active, measured/accepted, latest/certified world and PX4 request/status before counting. A read-only cross-process observation is not another product policy owner. V3 does not add implementation fields, so target numbers are expectations only.

| High-criticality fact | Proven AS-IS `D_f` | V3 target expectation | Evidence/uncertainty |
|---|---:|---:|---|
| desired mission intent | `?` | 1 | MissionController/runtime mirrors not fully writer-counted; do not turn lexical field count into D_f. |
| active execution identity / bundle generation | **2 scoped** | 1 | `ExecutionTimelineStore.committed_` and `ExecutionEpisodeSnapshot.active_generation` independently mutable; same V2 source proof. |
| measured crossing observation | **0 persistent** | 1 if bounded typed observation admitted | Current per-update geometry discarded; retaining it needs E1 age/invalidation decision. |
| accepted mission progress | `?` | 1 | Mission acceptance and runtime goal/progress mirrors require exact writer graph. |
| active world certificate | `?` | 1 execution authority | Bundle certificate and runtime copies may differ; not fully enumerated. |
| latest world | **1** | 1 | WorldModel snapshot owner; consumers read immutable evidence. |
| braking disposition | `?` | 1 product lifecycle, sampled role orthogonal | Historical API is a separate compatibility contract, not proof of second native authority. |
| PX4 Hold observed nav status | **1 local cached boolean**, but missing source/freshness witness | 1 typed PX4 status observation | PX4 itself remains external authority; request/ACK/completion are independent facts, not duplication. |

Highest **proven scoped** AS-IS `D_f=2`; global maximum unknown. V3 expected authority `D_f=1` per fact remains **NOT_ADMITTED**, not a measured reduction. The extra typed physical/status witnesses would preserve previously missing information rather than inflate duplication.
