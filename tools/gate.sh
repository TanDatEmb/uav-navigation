#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
PYTHON=${PYTHON:-/usr/bin/python3}

log_header() {
  printf 'gate: head=%s\n' "$(git rev-parse HEAD)"
  printf 'gate: dirty_paths=%s\n' "$(git status --porcelain | wc -l)"
}

static_gate() {
  printf '%s\n' 'gate: static'
  git diff --check -- . ':!artifacts/**'
  "$PYTHON" tools/validate_runtime_safety_ledger.py
  "$PYTHON" tools/check_mission_authority_cut.py
  "$PYTHON" tools/refactor/check_citations.py . docs/refactor
  "$PYTHON" tools/check_dependency_direction.py
}

python_gate() {
  printf '%s\n' 'gate: python'
  "$PYTHON" --version
  "$PYTHON" -m unittest discover -s tools/tests -p 'test_*.py' -v
  "$PYTHON" -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v
}

ros_gate() {
  printf '%s\n' 'gate: ros'
  command -v colcon >/dev/null
  if [[ -n "${PACKAGES:-}" ]]; then
    read -r -a packages <<< "$PACKAGES"
  else
    mapfile -t packages < <(colcon list --names-only | awk '$1 != "px4_ros2_cpp" {print $1}')
  fi
  ((${#packages[@]} > 0))
  colcon build --packages-select "${packages[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
  colcon test --packages-select "${packages[@]}" --event-handlers console_direct+
  colcon test-result --verbose
}

main() {
  log_header
  case "${1:-all}" in
    static) static_gate ;;
    python) python_gate ;;
    ros) ros_gate ;;
    all) static_gate; python_gate; ros_gate ;;
    *) printf 'usage: %s {static|python|ros|all}\n' "$0" >&2; return 64 ;;
  esac
}

main "$@"
