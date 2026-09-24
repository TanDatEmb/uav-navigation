# Qualification-grade nominal evidence gate

Mission `COMPLETE` and evaluator `qualification_eligible` are independent. A nominal true tracking-off run requires requested/effective configuration match, required stream completeness, lifecycle transaction attribution by exact identity, valid reference lineage, explicit tracking coverage/acceptance policy, explicit motion acceptance policy, valid source/frame witnesses, and no safety violation. The current A1/A2/A3 reports fail eligibility. This branch cannot supply numeric coverage, tracking or motion limits by inferring from successful runs or copying synthetic test fixtures. Until an approved versioned policy is provided, future runs remain `NOT_EVALUABLE` even if mission completes.

The raw PVA heartbeat accounting repair removes A2's duplicate reason while retaining every raw lifecycle record. A3's two same-source-stamp world-revision changes remain conflicts. No evaluator reason is suppressed to create eligibility.
