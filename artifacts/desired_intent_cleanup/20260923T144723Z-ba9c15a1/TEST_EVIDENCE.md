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

## Clean source checkpoint used for focused SITL

- After committing source and source-evidence docs, `/tmp/uav_desired_intent_clean_build.log` records a clean authoritative Release build at `473b2121817b2ba248c0c6eb40f9cfed89a3c2d3`: **23 packages** completed, four unchanged third-party warning emitters, manifest at `install/.uav_navigation_build_manifest.json`.
- `validate_manifest()` returned VALID with `git_dirty=false`, exact source HEAD `473b2121817b2ba248c0c6eb40f9cfed89a3c2d3`, source SHA256 `8ffce50ad2bd3adefcc0e056e1f990b5589b7369eb23a271e25ce6d0fedfacef`, and manifest SHA256 `22af33048922a94ecfcfa8149f17909a419b027616b31b260fbeb5c32e9f9b3b`. All new-cut SITL sessions in `SITL_RESULTS.md` record this clean manifest and source fingerprint.
- Product source was unchanged after this checkpoint. The last product-source edit is commit `c9006768`; subsequent edits document measured evidence only. A final clean-HEAD build and test invocation is a delivery gate after those evidence-only commits, not a substitute for this pinned SITL provenance.
- Focused SITL: 3/3 consecutive nominal mission completions, Core-pause stale-command/Hold fence, repeated-replan BACKUP/measured restart, terminal STOP witness. Exact session references and limitations are in `SITL_RESULTS.md`. The runner's versioned qualification outcome is not called PASS.

The builds, CTest targets, Python contract tests and static checks are component/build evidence. They do not establish matched nominal SITL parity, PX4 flight behavior, or flight qualification. The Python suite's trailing runtime report is explicitly not a 3/3 nominal SITL result. No qualification claim is made here.
