# Audit validation and evidence level

Executed on this branch:

- `python3 -m py_compile tools/*.py` — passed.
- Compiled `tools/route_crossing_replay.cpp` with GCC 13.3, C++20, and the pinned TARGET `src/contracts/navigation_mission/src/{mission,route_progress}.cpp` plus Eigen/yaml-cpp; ignored binary SHA-256 `140d692473435af206e201cc3e675b21855d336cfc691864c43760c0766c13f3`. This is an audit model harness, not the installed product executable.
- Extracted 14 exact-TARGET bags and runner odometry timelines; replayed the pinned geometry against each resolved mission. `tools/analyze_e1.py` generated 65 sample opportunities in 32 distinct waypoint episodes; `E1_RUN_COVERAGE.csv` records raw capture sparsity and chosen stream.
- `tools/analyze_e3.py` generated 237 normalized Hold-related rows while preserving wall/steady/sim/PX4 boot clocks; `tools/analyze_e4.py` generated 266 per-run metric summaries. All four CSVs parse with consistent headers and nonempty rows.
- `git diff 7da3e97cb399c2e39d62cfe60213a45e8a92300e -- src config docs/safety` is empty; original product checkout remained at TARGET without local modifications.

Evidence separation: source/history proof establishes mechanisms; replay/model checks geometry and ordering opportunities; existing runtime artifacts show only their captured run behavior; there is no new controlled SITL campaign and no flight qualification. The selected evaluator reports are FAIL/BLOCKED, not acceptance runs. `EVIDENCE_NOT_AVAILABLE` and `UNOBSERVABLE_WITH_CURRENT_TARGET` are retained wherever a required stage was not captured. No gate was relaxed to pass a test.
