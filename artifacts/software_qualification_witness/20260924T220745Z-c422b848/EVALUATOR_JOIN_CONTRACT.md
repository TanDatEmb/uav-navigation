# Evaluator join contract

`reduce_lifecycle` builds transactions from exact producer IDs and checks each observed phase against the immutable identity. `evaluate_software_qualification` checks exact Core reference identity and exact adapter admission or typed rejection; it counts both missing and conflicting references. It separately checks setpoint-update trace coverage. It does not use callback-time or source-time proximity to bind a required C0-SW lifecycle or reference.

The evaluator fails closed for missing policy metadata, missing/mismatched tracking-off configuration, active/unknown planner fault injection, incomplete writer accounting, missing required lifecycle/result/activation/retained sequence, missing reference or adapter receipt, untyped adapter rejection, and unattributed outcome. A generic `PAUSED_SAFETY_STOP` label and observed Hold do not prove the measured gate decision; that run remains `NOT_EVALUABLE` until its causal gate witness is assessed.

The original integrated `evaluation.qualification_eligible` semantics are not replaced by software eligibility. Reports expose scope and `software_qualification_eligible` separately; original runner `FAIL` remains unchanged. Offline reevaluation generated `FRESH_COHORT.csv` without rewriting historical `report.json`.
