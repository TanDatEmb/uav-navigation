# Decisions requiring policy authority

These are proposals for a decision maker, **not approved thresholds**. A policy record must name owner, C0 scope (currently requested speed 1–5 m/s), version, source, effective date, measurement definitions, exclusions, and independent validation evidence. The evaluator currently accepts no missing or unversioned policy as authority.

## P-COV-01 — Independent-truth coverage and pairing

**Question / why required:** What fraction of the declared navigation evaluation window must have source-time/frame-valid command-to-truth pairs, how long may an uncovered interval last, and how far apart may consecutive matched samples be? `evaluation.py::_tracking_coverage()` requires `min_coverage_ratio`, `max_uncovered_interval_s`, `max_pairing_gap_s`, version and provenance. The last field controls spacing between matched timestamps, not source skew within one pair. Without these values, tracking remains `NOT_EVALUABLE`.

**Current behavior:** Complete raw capture can still lack attributable or temporally paired tracking evidence. The ten-run cohort has no approved C0 coverage record, and all ten also have lifecycle lineage gaps.

**Option A:** Require near-continuous valid coverage over the full declared interval of Core execution authority, with bounded gaps derived from state/reference sampling and dynamics. Effect: strongest protection against hiding unsafe segments; intermittent transport/observer loss can make otherwise good flights not evaluable. Qualification requires an explicit positive bound and an independent gap/fault cohort.

**Option B:** Define narrower mission-active subwindows with separately justified startup/terminal transition treatment. Effect: can avoid counting non-authoritative setup/teardown time; unsafe exclusions could hide the very terminal failures seen in two runs. Qualification remains blocked until the omitted intervals have a separate witness and their treatment is approved.

**Evidence needed:** exact evaluation-window authority transitions, reference/truth sampling distributions, frame/clock uncertainty, worst credible relative speed/acceleration across gaps between matched samples, recorder drop accounting, and deliberate dropout tests. Do not derive a PASS gap from the observed maximum gap of the same cohort.

## P-TRK-02 — C0 tracking acceptance against independent truth

**Question / why required:** What `p95` and maximum position/velocity errors constitute acceptable closed-loop tracking for the C0 mission and hazards? `_tracking_acceptance_status()` needs four numbers, version and provenance. Planner command-to-LIO budget and PX4 local admission bounds are different contracts.

**Current behavior:** The evaluator reports truth-relative error but cannot issue integrated tracking PASS/FAIL. In ten runs, per-run position p95 was 0.218–0.496 m and maximum 0.272–0.769 m; velocity p95 was 0.376–1.052 m/s and maximum 0.756–1.647 m/s. Two runs did not complete because terminal measured anchor error was 0.762/0.759 m against the unchanged 0.750 m gate.

**Option A:** Approve hazard-allocated truth-relative position/velocity limits, including separate p95 and hard-tail purposes, after quantifying command-to-estimator, estimator-to-truth and frame/time uncertainty. Effect: integrated tracking can eventually yield PASS/FAIL; a strict limit can expose real product/PX4 performance failures, while a loose limit can hide them. Validate on independent holdout SITL and representative sensor data before using it for qualification.

**Option B:** Keep truth tracking descriptive and defer integrated flight-performance qualification. Effect: no unearned PASS or new flight safety permission; integrated C0 eligibility remains `NOT_EVALUABLE` even when software logic is correct. A software-only conclusion must have an explicitly separate scope and cannot reuse integrated `qualification_eligible` as if it covered flight performance.

**Evidence needed:** mission clearance/endpoint risk allocation, independent-truth accuracy, estimator error distribution, transform/time alignment uncertainty, representative 1–5 m/s trajectories, terminal and transient segments, and holdout evaluation. Do not copy test fixture limits or the ten-run maxima into a policy.

## P-MOT-03 — Measured motion acceptance

**Question / why required:** Which measured acceleration, jerk, stop-go, chattering, role-switch and completion-time measures are qualification requirements, under what filter/time basis and segment scope? `evaluate_motion_quality()` is descriptive; `evaluate_session()` emits `MOTION_ACCEPTANCE_POLICY_UNAVAILABLE`.

**Current behavior:** MAIN command 5/5/8 and physical/BACKUP 12/12/30 certify candidate roles; they do not define post-hoc measured motion acceptance. The ten-run role-mixed sampled-command jerk maximum reached 28.790 m/s³, which cannot be judged against MAIN 8 without exact role segmentation.

**Option A:** Approve role/phase-specific measured-motion objectives with exact derivative/filtering method, uncertainty budget, transient exclusions and independent validation. Effect: catches unpleasant or potentially unsafe closed-loop motion, but derivative noise and role mixing can produce false failures if poorly specified. It enables a future motion PASS/FAIL only after versioned policy and complete execution lineage.

**Option B:** Keep motion metrics descriptive and defer integrated motion quality. Effect: preserves existing candidate safety gates and avoids a fabricated flight-performance claim; integrated qualification remains `NOT_EVALUABLE` while motion is required.

**Evidence needed:** platform dynamics and controller capability, independently measured P/V/A where possible, sensor sampling/noise and derivative filter, MAIN/BACKUP/EMERGENCY and transition segmentation, mission time/comfort objective, repeated SITL plus representative recordings and holdout validation.

No option is selected by this document. The policy authority must decide scope and acceptance objective; engineering can then calculate candidate numeric limits from independent budgets and test them without changing product safety thresholds in this branch.
