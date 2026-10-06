#!/usr/bin/env bash
# S0 pruning: remove main-only packages and tooling (tracked files only).
# Owner-run. `tools/uavnav/s0_prune.sh -n` performs a dry run.
# Every removed file stays recoverable from main and from HEAD~.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

paths=(
  src/runtime
  src/execution
  src/px4
  src/estimation/fast_lio_ros
  src/estimation/fast_lio_tools
  src/contracts/navigation_contracts
  src/navigation_bringup
  tools/runtime
  tools/tests
  tools/refactor
  tools/benchmarks
  tools/gate.sh
  tools/data.py
  tools/validate_runtime_safety_ledger.py
  tools/check_mission_authority_cut.py
  tools/check_dependency_direction.py
  tools/verify_baseline_migration.py
  docs/evidence
)

for p in "${paths[@]}"; do
  git rm -r "$@" -- "$p"
done
