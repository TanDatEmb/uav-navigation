# Reference lineage results

All ten pinned predecessor sessions have `REFERENCE_LINEAGE_MISMATCH` under raw re-evaluation. Of **32,745** raw executable NavigationCommand references, **26,788** join a valid exact transaction and **5,957** do not. An unjoined sample is invalid *qualification lineage evidence*; the count does not mean the corresponding flight command was invalid. This follows from incomplete lifecycle transactions, not from a permissible exact heartbeat duplicate. The evaluator does not infer a missing bundle-owner cycle from request identity, source stamp proximity or observer order.

Baseline valid raw reference IDs and exact mismatches are reproducible with `reevaluate_raw.py`; per-run valid/invalid counts are in `RAW_SESSION_REEVALUATION.csv`. The current branch does not claim lineage closure or retroactively assign IDs. New producer-owned transition evidence is required before a fresh cohort can meet the zero-mismatch gate.
