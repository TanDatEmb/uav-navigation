# Raw session re-evaluation

`RAW_SESSION_REEVALUATION.csv` is generated from the ten exact session paths in the prior `NOMINAL_COHORT.csv` by `reevaluate_raw.py`. It loads `scenario.jsonl`, `samples.jsonl`, metadata and monitor evidence with the current evaluator; it does not read the prior report's metric values except for the original assessment label used in the comparison column.

Result: original and current assessments are `NOT_EVALUABLE` for all ten; qualification eligibility is false for all ten. The raw session file hashes are indexed in the prior product-stability `RAW_EVIDENCE_MANIFEST.csv` and `EVIDENCE_INDEX.md`. These historical sessions cannot be retroactively endowed with a missing producer ID, capture policy or authority witness.
