#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(git -C "$root" rev-parse --show-toplevel)"
snapshot="${AUDIT_SOURCE_SNAPSHOT:-$(dirname "$repo_root")/uav-navigation-as-is-audit-20260920/artifacts/architecture_as_is/20260920T-as-is-local/baseline/source_snapshot}"
g++ -std=c++20 -Wall -Wextra -Werror \
  -I /usr/include/eigen3 \
  -I "$snapshot/src/contracts/navigation_contracts/include" \
  "$root/tests/h0_tracking_loader_truth_table.cpp" \
  -o "$root/validation/h0_tracking_loader_truth_table" \
  > "$root/validation/logs/h0_compile.log" 2>&1
"$root/validation/h0_tracking_loader_truth_table" > "$root/validation/logs/h0_run.log" 2>&1
cat "$root/validation/logs/h0_run.log"
