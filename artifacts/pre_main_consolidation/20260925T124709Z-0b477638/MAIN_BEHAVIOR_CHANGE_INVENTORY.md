# Main behavior change inventory

This campaign's behavior-bearing delta: NO flight/source behavior change (`src/` byte-identical to incoming base). The campaign changes report/evaluator projection and tool gates: runtime verdict is separated from scope-specific qualification; exact producer-witnessed fail-closed outcomes can be assessed as C0-SW PASS; status-only safety stops remain NOT_EVALUABLE. No C0-IFP threshold, motion/tracking policy, safety gate, planner algorithm, PX4 policy, or eligibility rule is loosened.

The integration PR against historical/current `main` necessarily contains the previously approved architecture stack (96 first-party source paths in a 519-file overall diff). Those changes are inherited from the stabilized incoming lineage and are validated by this campaign's consolidated regressions; they are not newly introduced by this pre-main campaign.
