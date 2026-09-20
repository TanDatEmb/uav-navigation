# Coverage and limits

| Measure | Captured result |
|---|---|
| field_inventory_count | 30 |
| field_inventory_basis | selected state/identity/lease fields on critical decision path; not every private member or numeric algorithm value |
| field_with_source_refs | 30/30 model fields have at least one source reference; writer/reader inventories remain non-exhaustive |
| transition_inventory_count | 16 |
| transition_basis | behavior-significant selected transitions from mission through PX4 authority; not an exhaustive enumeration of every runCycle branch, reset, exception or helper |
| decision_sinks | 5/5 requested sink classes represented: goal, candidate commit/replace/revoke, command expose/sample/publish, recovery/emergency/hold, PX4 handover |
| scenario_count | 12/12 requested scenarios addressed with source path or explicitly unresolved limits |
| writer_completeness | NOT_ESTABLISHED: direct and indirect C++ write set not exhaustively AST-indexed; exact writer/reader lists describe inspected critical paths |
| test_coverage | Tests read and selected assertions linked; not run in this audit |
| runtime_trace_coverage | 0 runtime traces acquired |
| semantic_review | Manual source review is independent from model lint but bounded; see unresolved paths and limitation section |
| objects modeled | 12 |
| fields modeled | 30 |
| transitions modeled | 16 |
| decision sinks modeled | 5 |
| scenarios addressed | 12 |
| source evidence references | 68 |
| package manifests inventoried | 23 |
