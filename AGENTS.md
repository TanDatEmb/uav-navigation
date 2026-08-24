# Repository working contract

Before changing estimation, mapping, planning, control, PX4 integration, runtime
budgets, safety gates, or validation thresholds, read
`docs/architecture/runtime_safety_decision_ledger.md`.

Any temporary bypass, relaxed gate, disabled validation, fallback-only path, or
test-specific behavior must be recorded in that ledger in the same change. A
record must name its owner, scope, safety impact, evidence, removal condition,
and verification command. Never silently convert a workaround into product
behavior.

For every correctness or performance change, review three levels:

1. System ownership and end-to-end safety contract.
2. Detailed source, units, runtime artifact, and test evidence.
3. Adversarial review for local optimization, hidden bypasses, latency tails,
   and regressions in another layer.

Do not tune hard-gate values from a single SITL run. Measure a distribution on
SITL and representative recorded sensor data first, and keep behavior changes
separate from observability/refactor commits.
