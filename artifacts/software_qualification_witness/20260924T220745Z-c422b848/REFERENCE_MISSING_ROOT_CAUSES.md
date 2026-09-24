# Reference missing root causes

Historical cohort: 32,745 command references, 26,788 joined to qualification-valid lineage, 5,957 missing exact lineage. This is a historical evidence gap; old raw files were not backfilled.

| Root class at reference level | Count | Share of historical references | Required? | Producer/fix |
| --- | ---: | ---: | --- | --- |
| No exact *valid* lifecycle transaction for the reference | 5,957 | 18.2% | Yes | Core producer emits bundle/source and transition ownership; recorder preserves it |

The prior transaction-level causes were 87 missing bundle owner, 30 missing export owner and 23 missing consumer/supersession witness. These are **transaction counts**, not disjoint reference counts. Existing historical raw data does not prove which of those subcauses applies to each of the 5,957 references; subdividing them numerically would fabricate attribution. The single reference-level root class is exact and reproducible by the predecessor raw reevaluation script.

Fresh contract repair: immutable candidate producer cycle; explicit heading/emergency generation source; explicit activation, supersession and retry events; exact Core authorization; exact adapter admission/rejection; recorder passes identity through. Fresh required reference missing/conflict = 0/0 in all 10 captured sessions. This closes the *fresh* lineage gap. It does not prove every Core sample produced a separate PX4 setpoint callback: 147 admitted primary references lack a same-sample trace, and their cause (coalescing vs observer loss) is unproven.
