# Evidence index

| Evidence | Location / identity | Class | Notes |
|---|---|---|---|
| Base source | `aaa0533a6b747432a501f17c49d5229118055f7a` | source provenance | C1/C2 convergence-closed base. |
| PX4 checkout | `deaff86ee335dd697677bcfc2415a23878e1b895` | external provenance | Dirty state listed in `BASE_PROVENANCE.md`, preserved. |
| PX4 binary | SHA256 `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc` | binary provenance | SITL binary identified; no run was performed. |
| New execution-owner races | `src/execution/navigation_execution/test/test_execution_authority.cpp` | deterministic component | Three latch-controlled stale recertification races. |
| Localization reset races | `src/runtime/navigation_runtime/test/test_navigation_runtime_shutdown.cpp` | deterministic runtime-component | Existing blocked mapping callback tests, freshly run with navigation_runtime CTest. |
| Prior source audits | `artifacts/world_temporal_contract/20260924T233023Z-647a51b0/` | source/component | Identity, freshness and existing temporal contract. |
| Prior runtime evidence audit | `artifacts/world_runtime_evidence_closure/20260925T020509Z-bbbf34ab/` | audit | Previously gated by convergence; no world SITL run. |
| Runtime world session | none | not available | Isolated source-stale fault not run; no raw files/hash. |
