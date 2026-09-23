# Conditional PR sequence

| Phase | Old/new state and owners | Dual-write / rollback | Evidence gate |
|---|---|---|---|
| A. Freeze | no state removed; pin source/config/contract/provenance | none; abandon audit branch harmlessly | exhaustive matrix, baseline replay and repeated scenario distributions |
| B. Shadow reducer | new diagnostic-only state; old runtime remains sole authority | shadow state must never feed admission; rollback removes observer | event ordering, queue overflow, latency/WCET, equivalence traces |
| C. MissionProgress | move accepted gate/cursor from MissionController after protocol defined | short measured comparison window, one authoritative writer; rollback before enabling | self-intersection, reverse, gap, reset, crossing/readiness permutations |
| D. ExecutionAuthority | move Episode and Store decision writes to reducer; Store immutable payload retained | no permanent dual-write; guarded cutover with feature flag only during test deployment | active/staged, failed solve retention, world recertification, braking policy and lease faults |
| E. PX4 boundary | remove only proved planner/mission mirrors; keep local veto and Hold protocol | boundary remains independent; rollback old protocol version | delayed/reordered/lost command, reset, operator takeover, Hold API/status ordering |
| F. Protocol split | diagnostics move to trace message; control retains all safety witnesses | dual-publish for measurement only, old control sole authority until switchover | field-by-field consumer audit, sequence/lease/frame negatives |
| G. Candidate cleanup | only if deterministic pure-data replay needs it | no authority change; revert independently | hash/replay and validation equivalence |

Each phase must name removed and added fields in the conservation matrix and re-run admission tests. No permanent dual authority. Phase B is the smallest safe first cut; implementation must wait for the decisions in `DECISIONS_REQUIRED.md` and coverage gate.
