# Test evidence

## Pinned baseline before the desired-intent cut

- `/tmp/uav_desired_intent_baseline_release.log`: Release `colcon build` used `--cmake-force-configure`, completed 23 packages in 11m58s, and wrote the authoritative build manifest. This is the pre-cut reference build.
- `/tmp/uav_desired_intent_baseline_ctest.log`: focused selection `navigation_runtime navigation_mission navigation_execution navigation_contracts px4_navigation_external_mode`; 29 CTest targets passed (1 + 1 + 2 + 9 + 16 across five packages).
- `/tmp/uav_desired_intent_baseline_python.log`: Python runtime contract suite, 390 tests, `OK (skipped=1)`. Its trailing `STOPPED / Runtime report: FAIL` is a separate runtime report emitted by the test harness and is not a successful SITL result.

## Desired-intent cut component/static evidence

- `/tmp/uav_desired_intent_full_build1.log`: Release build with `--cmake-force-configure`, 23 packages completed in 1m20s. Four packages emitted stderr warnings. The build wrote an authoritative manifest, but its source record was dirty (`git_head=ba9c15a1214ca8ba54ec8f301f1e4056a3322848`, `git_dirty=true`); this is not a clean-source final release manifest.
- `/tmp/uav_desired_intent_full_ctest1.log`: same five-package focused selection; 31 CTest targets passed (1 + 1 + 2 + 9 + 18), zero failures.
- `/tmp/uav_desired_intent_full_python2.log`: Python runtime contract suite, 390 tests, `OK (skipped=1)`. It also ends with a separate `STOPPED / Runtime report: FAIL`; do not count that line as nominal SITL completion or flight evidence.
- Static checks rerun in the current worktree: `tools/check_execution_authority_cut.py` PASS; `tools/check_mission_authority_cut.py` PASS; `tools/check_desired_intent_cut.py` PASS; `tools/validate_runtime_safety_ledger.py` PASS (500 lines, 706 decisions, 36 gates, 34 active gates, 5 bypasses, 1 active bypass). These are source/structure checks.
- `git diff --check` is tracked separately after these documentation edits.

The builds, CTest targets, Python contract tests and static checks are component/build evidence. They do not establish matched nominal SITL parity, PX4 flight behavior, or flight qualification. The Python suite's trailing runtime report is explicitly not a 3/3 nominal SITL result. No qualification claim is made here.
