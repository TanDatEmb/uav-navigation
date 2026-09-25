# Open findings

## F-01 — steady-stop derivative boundary correction absent
- Class: MISSING_CURRENT_BEHAVIOR.
- Source: old commit 12c8352bcbedf49672358964a9c4258fccbdc0bc; canonical steady-stop branch in backup_braking.hpp and withinNumericalDynamicLimit.
- Risk: an extrema value above the exact acceleration/jerk threshold but inside the numerical boundary allowance is accepted; runtime incidence is not measured.
- Required evidence: deterministic boundary inputs around closed-form duration; assert exact derivative constraints after bounded correction or rejection with existing synthesis failure. Preserve abort behavior and thresholds.
- Next owner: focused canonical braking repair, then rerun Phase A.

## F-02 — predecessor authorization uses mutable planner warm-start
- Class: MISSING_CURRENT_BEHAVIOR.
- Source: current PlanningHistory omits previous_bundle; authorizeAndStage reads planner_warm_start_.snapshot(). Old bcff7c96 pins immutable predecessor evidence.
- Risk: bounded route-regression exception can rely on mutable/latest planner state rather than predecessor evidence captured for this solve. Runtime rebind/emergency paths also stage through the same warm-start owner.
- Required evidence: deterministic interleaving where request history names P1, warm-start changes to P2 before authorization, and P2 cannot grant P1's exception; positive exact predecessor case; world/dynamic/anchor validation remains mandatory.
- Next owner: focused request-owned predecessor-history repair, then rerun Phase A.

## F-03 — World runtime evidence remains gated
- Class: EVIDENCE_CONTRACT_DEFECT / not assessed in this branch.
- Required later: R1-R4 barriers, isolated source-stale and fresh recovery, same-source-tick revision, C0-SW world transaction/loss accounting.
