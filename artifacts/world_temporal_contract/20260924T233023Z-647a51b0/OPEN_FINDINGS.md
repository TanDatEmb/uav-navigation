# Open findings

| ID | Finding | Classification | Blocking impact / required next evidence |
|---|---|---|---|
| WTC-01 | Dedicated producer-owned world transaction events are absent from C0-SW lifecycle reducer/writer. | EVIDENCE_CONTRACT_DEFECT | Add exact transaction identities/events and evaluator joins before world lifecycle can be eligible. |
| WTC-02 | No isolated mapping/world-source stale SITL injection was found. | EVIDENCE_CONTRACT_DEFECT / RUNTIME_EVIDENCE_GAP | Need a fault that stops only world observation advancement while proving other streams and processes live. |
| WTC-03 | Generic non-committed mapping publication fallback invalidates command and fail-stops mapping worker for every decision except `kSuperseded`. | WORLD_CONTRACT_DEFECT / REACHABILITY_UNPROVEN | Prove reachability and intended disposition per typed decision; tests for all negative decisions and publication/execution visibility. No behavior weakening yet. |
| WTC-04 | World identity carries source stamp but not receive/integration/publication timestamp. | DIAGNOSTIC_LIMITATION | Determine whether a required safety contract needs independent pipeline-progress evidence; do not call source age receive freshness. |
| WTC-05 | Same-epoch backend reconstruction/bag rewind/config reset generation semantics not implemented in live MappingActor producer. | WORLD_CONTRACT_DEFECT candidate / scope-limited | Establish whether such reset exists in deployed runtime; current observed generation advance is localization reset only. |
| WTC-06 | World invalidation/suspension/resume runtime events are not transaction-attributed for C0-SW. | EVIDENCE_CONTRACT_DEFECT | Exact producer evidence, writer accounting and world scenario reducer needed. |
| WTC-07 | CandidateBundle's evaluator/validator are opaque callables and not readily serializable for replay. | ARCHITECTURAL_DEBT | Current source revalidation works; serialization/replay redesign not proven necessary in this cut. |
| WTC-08 | Historical A3 equal-source-stamp revision conflicts lack exact raw transaction evidence. | EVIDENCE_GAP | Preserve NOT_EVALUABLE; no retrospective inference. |

No safety threshold, planner algorithm, mapping algorithm or PX4 Hold protocol changed.
