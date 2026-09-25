# Calculation and option analysis

## Available distribution is characterization, not a threshold source

The incoming ten-run raw/evaluator analysis reports per-run minimum / median / maximum:

| Statistic | Position error, m | Velocity error, m/s |
|---|---|---|
| p95 | 0.218 / 0.276 / 0.496 | 0.376 / 0.553 / 1.052 |
| maximum | 0.272 / 0.446 / 0.769 | 0.756 / 1.032 / 1.647 |

The motion report gives measured vector acceleration p95 1.087 / 1.521 / 4.140 m/s² and maximum 2.883 / 6.929 / 11.379 m/s². Sampled-command jerk p95 was 1.584 / 4.565 / 13.531 m/s³ and maximum 6.846 / 23.088 / 28.790 m/s³ across mixed roles. These figures identify the range a proposed policy would confront; they do not decide acceptable risk.

Ten run-level observations are too few for a reliable tail limit. Even if all ten had passed a predeclared limit, zero observed violations would leave a one-sided 95% binomial upper bound on the violation probability of `1 - 0.05^(1/10) ≈ 25.9%`. In fact, two missions had terminal safety stops. A threshold chosen from these maxima and checked against these same runs has no independent validation.

## Tracking budget calculation to perform after scope decision

For source-aligned vectors, `reference - truth = (reference - estimator) + (estimator - truth)`, with an additional bounded transform/time-alignment term if comparisons are not simultaneous in one frame. A deterministic norm bound can sum independently justified upper bounds. A p95 bound **cannot** generally be obtained by summing three component p95 values: either measure the joint aligned error directly on a holdout set, or allocate tail probabilities `α_i` with `Σα_i ≤ 0.05` and use component `1-α_i` bounds. Define a separate hard-tail/maximum requirement; a sample maximum alone is not a guaranteed bound.

The current 0.25 m planner tracking budget is command-to-LIO and supplies only one component. The LIO-to-truth error and frame/clock alignment budgets are not pinned. Pairing uncertainty can be bounded only after a relative-speed/acceleration envelope and maximum source-time offset are declared, for example `distance uncertainty ≤ v_rel·Δt + 0.5·a_rel·Δt²` under an independently justified bounded-motion assumption. Neither `Δt` policy nor the needed uncertainty envelope is approved here.

## Coverage and motion calculation to perform

Choose the evaluation window from actual command authority, then compute source-paired valid duration divided by required duration, longest uncovered interval, and pairing-gap distribution. Set permitted gaps from the hazard/dynamics time budget and validate deliberate observer dropout. Never equate zero writer drops with complete producer evidence.

For measured acceleration/jerk, specify whether the input is independent truth, LIO, PX4 or command; derivative method, filter, sample rate, source clock, role and transition windows must be fixed before computing p95/max. The MAIN command and physical/BACKUP envelopes are candidate certification constraints, not generic measured-motion thresholds. A mixed-role jerk maximum has no single applicable role limit.

**Engineering recommendation on evidence, not policy choice:** gather independent estimator/truth alignment, hazard and controller envelopes, and a predeclared holdout protocol first. The decision maker may then approve either an integrated flight-performance policy or an explicit deferment. No numeric policy is installed in this branch.
