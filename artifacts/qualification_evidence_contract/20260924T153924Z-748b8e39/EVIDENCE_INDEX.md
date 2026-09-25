# Evidence index

| Claim | Source | Evidence class | Limit |
|---|---|---|---|
| Baseline/branch/PX4 identity | `BASE_PROVENANCE.md`, live Git and SHA256 | Provenance | PX4 checkout has unrelated pre-existing dirty paths. |
| 140 unresolved, 0/10 eligible | `reevaluate_raw.py`, `RAW_SESSION_REEVALUATION.csv`, `LIFECYCLE_UNRESOLVED_CLASSES.csv` and prior raw `scenario.jsonl`/`samples.jsonl` | Raw session replay | Historical missing producer IDs cannot be fabricated. |
| Lifecycle phase meaning | `tools/runtime/external_mode_scenario.py`, `tools/runtime/evaluation.py`, `LIFECYCLE_EVENT_CATALOG.md` | Source plus deterministic tests | Recorder is an observer of producer diagnostics. |
| PVA heartbeat and odometry policy | `tools/runtime/evaluation.py`, `tools/runtime/tests/test_evaluation.py`, `STREAM_TIMESTAMP_POLICY.md` | Source plus unit | A3 changed-certificate same-stamp conflict remains. |
| Missing tracking/motion policy | `tools/runtime/evaluation.py`, `TRACKING_POLICY_CONTRACT.md`, `MOTION_POLICY_CONTRACT.md`, prior scenario configs | Source plus raw distribution | No independent approved numerical policy. |
| Deferred simulator/terminal behavior | Prior product-stability `TERMINAL_RESULTS.csv`, `TEMPORAL_STALL_EVENTS.csv`, `SCOPE_AND_DEFERRED_FINDINGS.md` | SITL observation | No new product or qualification closure. |
| Branch validation | `TEST_EVIDENCE.md`, authoritative Release build manifest, `test-results/` | Build, CTest, Python, static guards | Passing tests do not establish integrated qualification. |

The raw cohort file hashes and paths are in `artifacts/qualification_product_stability/20260924T140134Z-dee89a7e/RAW_EVIDENCE_MANIFEST.csv` and its `EVIDENCE_INDEX.md`. Large raw bags/logs remain outside Git. Original mission outcomes and evaluator assessment are not rewritten.
