# H3.4-python hunk ledger

- `docs/safety/runtime_safety_current.md`: update only the H3.4 ledger row.
- `tools/runtime/external_mode_scenario.py`: reject a route whose final
  waypoint behavior is not `stop`.

Safety scope: terminal mission-contract validation in the diagnostic/runtime
scenario loader; no threshold, lease, PX4, or command-authority change.
Verification: runtime validator tests, mission/planner/trajectory tests,
`python3 tools/validate_runtime_safety_ledger.py`, and `git diff --check`.
