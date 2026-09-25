# World event producer audit

The source was inspected before any implementation change.

| Fact/event | Component that knows it | Current boundary | Evidence available today | Needed producer witness |
|---|---|---|---|---|
| Immutable mapping snapshot constructed | `MappingActor` called from `NavigationRuntimeNode` mapping worker | `result.snapshot` after `mapping_actor->process()` | Periodic `navigation_mapping/world_model` latest identity/counters | One event per produced snapshot/update, with source identity and receive/steady time. |
| World publication committed/superseded | Runtime mapping callback + `WorldSnapshotStore::publishAndFinalizeDecision()` | exact return `WorldCommitDecision` | Latest mapping diagnostic does not preserve callback transaction or owner versions | Event from exact callback result, including prior/new identity and transaction disposition. |
| Active/pending retained or invalidated | Runtime immutable candidate validation + `ExecutionAuthority` transaction | `retain_validated_bundle`, `retain_validated_pending`, `invalidated_current`, post-transaction snapshot | Aggregate fast/full counters and current command identity only | Event with pre/post timeline and active/pending generation/request identities plus validation path. |
| World freshness suspension | `NavigationRuntimeNode::suspendCommandForWorldFreshness()` | `suspendIfCurrentSnapshot(expected)` success | Aggregate suspension count and execution snapshot; no event sequence | Event only when exact suspension mutation succeeds. |
| Exact execution recertified/resumed | Runtime mapping callback after committed world transaction and `observeRetainedCommand()` | exact current-bundle checks and owner mutation | Aggregate recovery count; command samples show subsequent world identity but cannot prove transaction lineage alone | Event emitted at successful exact resume, with prior/new world and same bundle generation. |
| Event receipt/loss | Existing external-mode scenario observer and `EvidenceWriter` | `_diagnostics()` → `_record()` → bounded writer | Writer has submitted/accepted/written/drop/error counters and per-category counts | Preserve each producer event as its own `world_transaction` record; reducer checks sequence/identity and writer accounting. |

No World lifecycle producer event currently exists. Periodic latest-state diagnostics cannot be promoted into transaction evidence by timestamp proximity. `runtime_instance_id/session_id` are observer session identities today; Runtime does not own a shared C0 session token. The proposed join therefore uses session-scoped producer sequence plus exact World/execution identities and does not claim a runtime-global UUID.

Source references: `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp` mapping worker construction and `suspendCommandForWorldFreshness`; `src/mapping/navigation_mapping/include/navigation_mapping/world_snapshot_store.hpp`; `src/execution/navigation_execution` ExecutionAuthority APIs; `tools/runtime/external_mode_scenario.py::_diagnostics` and `_record`; `tools/runtime/evidence_writer.py` bounded loss-accounted writer.
