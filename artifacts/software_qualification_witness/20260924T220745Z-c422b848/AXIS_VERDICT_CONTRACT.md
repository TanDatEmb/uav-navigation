# Axis verdict contract

| Axis | Meaning in C0-SW | Primary cohort |
| --- | --- | --- |
| `PRODUCT_LOGIC` | Mission acceptance/terminal outcome attributable to software facts | PASS for 5/5 |
| `AUTHORITY_IDENTITY` | Exact lifecycle, execution and adapter lineage | PASS for 5/5 |
| `TEMPORAL_SAFETY` | Current command/state identity and receipt evidence for the observed outcome | PASS for 5/5 |
| `EVIDENCE_COMPLETENESS` | Required witness and writer integrity | PASS for 5/5 |
| `FAULT_HANDLING` | Focused fault suite, independently assessed | `SUITE_EVIDENCE_REQUIRED` |
| `FLIGHT_PERFORMANCE` | Integrated physical tracking and stopping | `DEFERRED_C0_IFP` |
| `ENVIRONMENT_VALIDITY` | Gazebo/native simulation fidelity | `DEFERRED_SIMULATION_VALIDITY` |

The verification cohort's `FAILED_COMPONENT` runs 2/3 and safety-stop run 5 are `NOT_EVALUABLE` at the per-run C0-SW outcome gate, despite complete identity/writer evidence. The evaluator must not turn a measured flight-performance failure into a software PASS, nor turn a correctly observed fail-closed response into proof of the physical motion threshold. `FAULT_HANDLING` is not silently green from nominal mission completion.
