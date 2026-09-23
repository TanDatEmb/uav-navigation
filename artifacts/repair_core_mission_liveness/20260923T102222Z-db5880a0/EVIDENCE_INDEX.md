# Evidence index and provenance

| Claim | Primary evidence | Level / limit |
|---|---|---|
| Base and product pins | `BASE_PROVENANCE.md`; base `db5880a05ffbb82f3f528bc18934af0cb0390bab`; original `7da3e97cb399c2e39d62cfe60213a45e8a92300e` | Git/tree/submodule/PX4 binary provenance |
| Baseline completes and retains predecessor | `CAUSAL_TIMELINE_BASELINE.md`; session `103615-258561`; prior migration `SITL_RESULTS.md` sessions `095057-201248` and `095812-211924` | Focused SITL, no qualification |
| First failed boundary | `CAUSAL_TIMELINE_MIGRATION_PRE_FIX.md`; session `104546-270858` `logs/mapping.log` H4/H5/stack, `scenario.jsonl`, rosbag `/navigation/command_admission` | Source + runtime trace; exact failed base |
| Publication identity mechanism | `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp` final `publishCommand()` transaction and stale fallback; `src/runtime/navigation_runtime/include/navigation_runtime/runtime_boundaries.hpp` | Source proof |
| Core-only mission writer | `src/runtime/navigation_runtime/src/mission_progress.cpp`; `tools/check_mission_authority_cut.py`; `STATE_DIFF.md` | Source + static guard |
| Adapter local admission | `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp` command acceptance/receipt; `INTERFACE_AUTHORITY_CLASSIFICATION.md` | Source; receipt does not prove firmware consumption |
| Baseline accepted MAIN continuation | pinned baseline `mission_controller.cpp` PASS branch around line 569 | Source proof for receipt role |
| Repair liveness and gaps | `CAUSAL_TIMELINE_REPAIR.md`, `HANDOFF_TIMING.csv`, `SITL_RESULTS.md`; sessions `112040-309344`, `112251-312566`, `112456-315691` `scenario.jsonl`, `logs/mapping.log`, `rosbag` | Matched `off` 3/3 focused SITL; `relaxed` 3/3 supplementary; not qualification |
| World/certificate and lease safety | pillar `111139-302607` world revision 510 invalidation/Hold; pause `112903-322160` 650.146 ms, adapter stale PVA/Hold; first pause `112700-318895` Hold request only | Separate focused runtime observations; pause does not isolate world-source staleness and also triggers mapping cloud loss |
| Deterministic and static regressions | `TEST_EVIDENCE.md`, build/test logs under `/tmp/uav-repair-*.log`, package CTest XML | Component/model/static; not SITL qualification |
| Safety contract | `docs/safety/runtime_safety_current.md`, `docs/safety/runtime_safety_index.md`; ledger validator | Contract record; numeric thresholds unchanged |

All named runtime sessions live under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260923T<suffix>/`. Session `metadata.json` records exact source fingerprint, `VALID` Release manifest, submodule and external PX4 checkout/binary identities. Documentation artifacts were completed after the runtime sessions; product source/config bytes were not changed after the matched `off` build. A final clean-HEAD Release build is required to regenerate the authoritative manifest for the committed tree. External PX4 checkout was dirty before this repair and the captured diff/status are in each session `provenance/` directory. Session `report.json` is used for mission outcome; its overall `FAIL` is kept because versioned qualification is ineligible. Fault-run `BLOCKED` is preserved separately.
