# Evidence index

## Source and tests

- Base provenance: `BASE_PROVENANCE.md`.
- Callback semantics source audit: `PX4_ROS2_CALLBACK_SEMANTICS.md`.
- Deterministic handover reducer: `src/px4/px4_navigation_external_mode/test/test_px4_authority_handover.cpp`, 6/6 direct tests and included in 94/94 CTest.
- Release manifest: `/home/letandat/Dev/uav-navigation-px4-authority-emergency/install/.uav_navigation_build_manifest.json`.
- Python suite: 459 passed, 1 skipped.
- Static guard/ledger command results: recorded in `TEST_EVIDENCE.md`.

## H2 raw runtime evidence

Final-source rerun: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260926T011400-401857`. Hash manifest and interpretation are in `H2_WORLD_STALE_HOLD.md`.

Earlier run (captured before the final timeout retry edit): `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260926T005456-384877`.

| File | Size | SHA-256 |
|---|---:|---|
| `scenario.jsonl` | 3,191,616 | `5c2b98153bb3df0e0e437a416f2ac06d23f0bebd5a24c21e807cb0d796199a15` |
| `world_observation_gate.jsonl` | 7,586 | `44b021ca160f32772982eff887cf1aa40eadf67b29ffb7cbf4505ce857628e67` |
| `execution_timeline.jsonl` | 11,700,009 | `108998c2aa8b70e28525eea7483d6497b41a0d7c101c9eba38eb5900958f228d` |
| `report.json` | 2,601,742 | `647be4dd0c0eb54d9372cc75897b063fadf180c856ce67ae37464b7a97a9dba0` |
| `metadata.json` | 248,871 | `f2d85baa528243310a3450a0ec09d389a6fcdd8baac546b6f96acbf84c43dff9` |
| `runtime.json` | 240,579 | `c527434c14fe2b1a0951ea57b5534be7507376ffc0d0c91eac53d064ce57634e` |

The runner preserved rosbag, logs, resolved config, PX4 parameter/status snapshots, and provenance in this session directory. The top-level evaluator report is `BLOCKED` because the injected fault run did not complete the mission; treat it as fault evidence, not nominal acceptance.
