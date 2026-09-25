# Test evidence

## Fresh build

Command, with ROS Jazzy and the exact C2 base underlay sourced:

```text
colcon build --packages-select navigation_execution navigation_runtime --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

Result: **PASS**, both selected packages built in Release. The build reported that `navigation_runtime` and dependencies are also present in the C2 underlay; the selected packages were rebuilt in this overlay and test executables ran from this worktree.

## CTest

```text
ctest --test-dir build/navigation_execution --output-on-failure
2/2 passed

ctest --test-dir build/navigation_runtime --output-on-failure
19/19 passed
```

The execution tests include the three new latch-controlled world recertification races. Runtime tests include `test_navigation_runtime_shutdown`, with both localization-reset/old-mapping-callback cases, and `test_localization_epoch_reset`.

## Python and static checks

- `python3 -m unittest discover -s tools/runtime/tests`: **444 passed, 1 skipped**.
- Mission authority guard: PASS.
- Execution authority guard: PASS.
- Desired intent guard: PASS.
- Fail-closed ownership guard: PASS.
- `tools/validate_runtime_safety_ledger.py`: PASS (`current=500`, `decisions=706`, `gates=36`, `active_gates=34`, `bypasses=5`, `active_bypasses=1`).
- `git diff --check`: PASS at documentation/test checkpoint.

These results are source/component validation; they do not substitute for the missing isolated stale-world SITL or C0-SW runtime witness.
