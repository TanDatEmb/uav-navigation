# Test evidence

- Authoritative clean Release build at product SHA `68289532b93641973238329d0ec6c4b0626b43f9`: `tools/runtime/build.py --mode release build`, 23 packages finished; build manifest source git head matches SHA. Manifest SHA256: `542d33801c698369c708d624f4a97e6a4fa8525d2bc295cb26f578cb9617a353`.
- Fresh focused CTest at that SHA: 19/19 targets pass across 7 relevant packages.
- Runtime Python suite: `443 passed, 1 skipped`.
- Static checks: mission authority, execution authority, desired intent, fail-closed ownership all PASS.
- Safety ledger: PASS (`current=500`, `decisions=706`, `gates=36`, `active_gates=34`, `bypasses=5`, `active_bypasses=1`).
- `git diff --check`: PASS on clean product source before documentation creation.
- Full workspace CTest: not clean; 13 third-party pinned `px4_ros2_interface_lib`/example test failures from lint/FMU setup remain external and were not relabeled PASS.
