# Evidence index

Pinned product source `33ec33626c43e20b84029853925f402532517285` derives from base `2b9279b42f3dce0d2565e4ef008852b018baf99f`. `BASE_PROVENANCE.md` records the external PX4 checkout, dirty status and binary hash. Raw SITL sessions remain under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/`; they are not committed.

| Purpose | Session directory suffix | `report.json` bytes / SHA256 | `rosbag_0.db3` bytes / SHA256 |
| --- | --- | --- | --- |
| Nominal N1 | `030555-816276` | 24,984,393 / `eef1aed84f9182f24d519da622709bcf914d6875f44170bb959db3e31efb6ae0` | 526,016,512 / `7af2cd94b5def8471b5f876e84863153a163b54457579a865508c96cb7ecd417` |
| Nominal N2 | `030839-821361` | 28,454,108 / `89e8c7a854a35131dc1dc7ab37613ef01cce493f93ad8e3e2480a6cf6c077bc5` | 562,327,552 / `28575bbcb7ed848e17753f8f6d157dd0cfce346336a7e0fe13785c56346b765c` |
| Nominal N3 / typed rejection | `031122-825633` | 23,492,868 / `208ca785a7f26a3f52ddcd5b85c8177bc6c9505fa4ea556fba6d17e2085fb59e` | 512,442,368 / `b31df23aa38ed2ad0e4dd0f15494ac4156809286fb5f1f773df19c356d48adef` |
| Core pause | `050320-27474` | 4,446,042 / `9d275923e51ba579e2b77f4f66088cf5af174fa8ca3a3847aafa4ab5ab0552f1` | 287,969,280 / `72330a46142ef4fdf301dc13e460127a59d6b944b7c5fc6b718cfea05d5780c2` |
| Repeated BACKUP | `051755-34772` | 46,094,755 / `676d6b49cd8f54ab4fdf2773cc8a5675ca739db28099a9be95942acd9962950f` | 625,733,632 / `0cbae425d53831b202b119fa432b885c86a236b6e7b19ad50b4cfccf76cc80bf` |
| Exact `kOptimizationFailed` | `052428-38549` | 27,481,888 / `847da6785c42d28ffbb12d14837342949a2cf841daabab7a8abcd7664437bcfd` | 582,164,480 / `bd8f0e12116a3ba3b8cd6032ae3f7bd3fac327c3b67fafb079b9e6345f541884` |

For each row, the complete directory name is `external-mode-check-20260924T` plus the suffix. Each session also contains `metadata.json`, `scenario.jsonl`, process and runtime logs, config snapshots, and evaluator output. `SITL_RESULTS.md` records the actual outcome and exclusions. These SHA256 values were computed directly from the files at the stated paths; do not substitute a later `latest` symlink target.

Source/test anchors:

- `src/contracts/navigation_contracts/msg/NavigationCommand.msg`, `NavigationExecutionDiagnostics.msg`, `NavigationCommandRejection.msg`: final wire contract.
- `src/contracts/navigation_contracts/include/navigation_contracts/navigation_command_contract.hpp`: typed shape and temporal assessments.
- `src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/command_admission_assessment.hpp`, `src/navigation_mode_node.cpp`: typed dynamic admission and exact commit/rejection protocol.
- `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`: Core command and observer publication.
- `tools/runtime/command_diagnostics.py`, `tools/runtime/external_mode_scenario.py`: offline exact-key evidence join.
- `tools/check_navigation_command_contract.py`: static 27-field allowlist and diagnostic separation.
- `TEST_EVIDENCE.md`: Release/CTest/Python/static command results; `/tmp` logs are local convenience logs, not retained raw evidence.

Evidence levels: source and component proof for predicate equivalence; raw SITL observation for 3/3 mission completion and focused faults; **no flight qualification**. The runner's `FAIL`/`NOT_EVALUABLE` due versioned evaluation and tracking config mismatch is preserved.
