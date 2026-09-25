# Main known limitations

- C0-IFP motion/tracking acceptance policy and GT pairing lineage remain NOT_EVALUABLE; C0-SW is independently assessable and does not inherit those blockers.
- Nominal terminal recovery is not fully repeatable (2/5 safe terminal stops); this is pre-beta stability debt.
- One completed session is runtime FAIL due the current BRAKING-inclusive cruise-speed assertion; interpretation remains unresolved as a performance contract, not hidden.
- N1 source is sparse at RegisteredScan input; missing independent pre-bridge generation identity prevents exact upstream component attribution.
- Full live-FMU integration tests, EMERGENCY runtime and PX4 authority handover remain unclosed.
- World event CPU / queue high-water not measured; no drops observed.
- PX4 checkout was dirty external state; pinned binary and SHA were recorded, external dirt was not part of candidate.
