# Incoming Evidence Contract delivery

The input is branch `codex/qualification-evidence-contract-closure-20260924`, HEAD and evidence-contract SHA `b39661ae16036ae6199077a03736ece81e79d4d1`, base and product-behavior base `748b8e3924a0042b975382468d675df4c987a3c2`. Its verdict is **`POLICY_PROVENANCE_BLOCKED`**. The incoming worktree was clean when inspected. `src/` has no byte change from the product-behavior base; evidence recorder/evaluator, tests, and audit artifacts did change. The ten-run predecessor cohort's product source was `96ed8d089e576c505abdc15500746e9b5be10e50`, with no `src/` diff to the behavior base.

PX4 checkout `/home/letandat/Dev/Autopilot` is `deaff86ee335dd697677bcfc2415a23878e1b895` with pre-existing dirty paths; binary `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4` SHA256 is `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`. Pinned gitlinks: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db` and `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.

| Required input | Verified result |
|---|---|
| Lifecycle unresolved / conflicting | 140 total / 0 in the ten-run raw replay; 87 missing bundle identity, 30 missing export-owner identity, 23 missing consumer/supersession witness. The incoming artifact does not separately enumerate required versus optional unresolved rows; the required-zero gate is unmet. |
| Reference lineage | 26,788/32,745 exact references valid for qualification; 5,957 unjoined. All ten runs have lineage mismatch. This does not classify flight commands as invalid. |
| Timestamp policy | Versioned source-time policy: exact PVA heartbeat may repeat; measured odometry strictly increases within epoch. Historical changed-world same-stamp A3 remains conflicting. |
| Tracking policy | No approved C0 coverage or independent-truth acceptance values; evaluator requires version and provenance. |
| Motion policy | Measured motion remains descriptive; no approved acceptance contract. |
| Qualification | 0/10 eligible, all ten `NOT_EVALUABLE`. Original mission outcome: 8 `COMPLETE`, 2 `PAUSED_SAFETY_STOP`. |
| Raw-session reevaluation | Ten original `scenario.jsonl`/`samples.jsonl` sessions replayed; no report summary promoted to evidence. |
| New nominal cohort | 0 runs on the incoming evidence branch. |
| Behavior-changing files | No product `src/` change, no safety-threshold change. Evidence semantics changed in `tools/runtime/evaluation.py` and `tools/runtime/external_mode_scenario.py`. |

Primary evidence: incoming `VERDICT.md`, `BASE_PROVENANCE.md`, `LIFECYCLE_UNRESOLVED_CLASSES.md`, `REFERENCE_LINEAGE_RESULTS.md`, `STREAM_TIMESTAMP_POLICY.md`, `QUALIFICATION_ELIGIBILITY.md`, `RAW_SESSION_REEVALUATION.md`, and `NEW_NOMINAL_COHORT.md` under `artifacts/qualification_evidence_contract/20260924T153924Z-748b8e39/`. Live Git, gitlink, PX4 HEAD and binary hash checks confirmed the pinned identifiers.
