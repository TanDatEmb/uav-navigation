#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(git -C "$root" rev-parse --show-toplevel)"
snapshot="${AUDIT_SOURCE_SNAPSHOT:-$(dirname "$repo_root")/uav-navigation-as-is-audit-20260920/artifacts/architecture_as_is/20260920T-as-is-local/baseline/source_snapshot}"
g++ -std=c++20 -O0 -g -pthread \
  -I /usr/include/eigen3 \
  -I "$snapshot/src/px4/px4_navigation_external_mode/include" \
  -I "$snapshot/src/contracts/navigation_mission/include" \
  -I "$snapshot/src/mapping/navigation_world_model/include" \
  -isystem "$root/validation/install/navigation_world_model/include" \
  -isystem "$root/validation/install/navigation_planning/include" \
  -isystem "$root/validation/install/navigation_common/include" \
  "$root/tests/h1_mission_counterexample.cpp" \
  "$snapshot/src/px4/px4_navigation_external_mode/src/mission_controller.cpp" \
  "$snapshot/src/contracts/navigation_mission/src/route_progress.cpp" \
  -o "$root/validation/h1_mission_counterexample" \
  > "$root/validation/logs/h1_compile.log" 2>&1
"$root/validation/h1_mission_counterexample" > "$root/validation/logs/h1_run.log" 2>&1
cat "$root/validation/logs/h1_run.log"
