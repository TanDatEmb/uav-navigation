# Lifecycle raw re-evaluation

`reevaluate_raw.py` loads `scenario.jsonl` and `samples.jsonl` of all ten pinned predecessor sessions with the current evaluator. Result: **140 unresolved, zero contradictory-event conflicts, zero qualification-eligible**. Valid transaction counts by run: 45, 33, 40, 24, 21, 9, 21, 15, 12, 11. Unresolved counts: 10, 9, 23, 19, 5, 16, 13, 13, 18, 14. Exact session paths and blockers are in `RAW_SESSION_REEVALUATION.csv`; transaction rows are in `LIFECYCLE_UNRESOLVED_CLASSES.csv`.

The replay also exposes **20 orphan delayed activation observations** without a unique export-owner cycle. These are now explicit `unbound_events`; they do not silently vanish or acquire the nearest cycle. Baseline recorder request rows had inherited cache identity, so this historical capture cannot be made authoritative by the new recorder fix.

The new `evidence_outcome` field distinguishes `RESOLVED`, explicit `SUPERSEDED`, explicit `INTENTIONALLY_ABSENT`, `MISSING_EVIDENCE` and `CONFLICTING_EVIDENCE` without changing the existing fail-closed reducer `status`. An orphan delayed activation is now surfaced in `unbound_events` and makes status `INCOMPLETE`. It is not matched to the nearest planner cycle.

Gate `qualification-required unresolved=0` is **not met**. Historical raw records do not contain enough exact transition evidence to close it retroactively. The evidence producer requires a bounded exact execution transition witness in a subsequent capture; no success claim is made from the vocabulary improvement alone.
