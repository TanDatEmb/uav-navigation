# Qualification eligibility

The C0-SW eligibility predicate is scoped and explicit. It requires declared `C0_SW/C0_SW_V1` provenance, requested/effective tracking off at both boundaries, suppression off, no planner fault injection, a C0 requested speed, loss-free finalized required writers, exact required lifecycle/reference identities, exact adapter receipt/rejection, and an assessable product outcome. If the outcome is an attributable software failure, it can be *eligible* and still assessed `FAIL`. C0-IFP eligibility remains separate and was not granted by this branch.

Primary: 5/5 consecutive C0-SW eligible and assessment PASS after offline reevaluation at `db43b608`; original integrated runner FAIL retained. Verification: 2/5 eligible/PASS; two `FAILED_COMPONENT` and one `PAUSED_SAFETY_STOP` remain NOT_EVALUABLE. Each verification run nevertheless has zero required lifecycle/ref gap or conflict and no required writer loss. The distinction is causal outcome assessment, not a fabricated lineage repair.

No claim of HITL, flight qualification, or PX4 firmware consumption follows from C0-SW eligibility.
