# Timestamp evidence results

The pinned run 5 accepted measured streams showed zero adjacent source duplicates/regressions in ground truth (3,573), propagated odometry (3,499) and corrected odometry (680). Diagnostic streams can repeat source stamps and are not trajectory measurement streams. Existing evaluator tests retain exact heartbeat collapse, changed-certificate duplicate conflict and regression detection; new tests distinguish strict from nondecreasing policy explicitly.

Historical A2 exact PVA heartbeats were already canonicalized without deleting raw records. Historical A3 has two changed world revisions at unchanged source stamps and remains conflicting. No raw duplicate was converted to PASS by the policy enum.
