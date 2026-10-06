#!/usr/bin/env bash
# Rebuild gate: tools/uavnav/gate.sh [static|python|ros|all]
# The last line printed is GATE_RESULT=PASS or GATE_RESULT=FAIL.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

PYTHON="${PYTHON:-/usr/bin/python3}"
stage="${1:-all}"
status=FAIL
trap 'echo "GATE_RESULT=${status}"' EXIT

stage_static() {
  echo "gate: static"
  git diff --check
  git diff --cached --check
  "$PYTHON" tools/check_documentation.py . docs
}

stage_python() {
  echo "gate: python"
  if ! compgen -G "tools/uavnav/tests/test_*.py" > /dev/null; then
    echo "gate: python: no tests"
    return 0
  fi
  "$PYTHON" -m unittest discover -s tools/uavnav/tests -t . -v
}

stage_ros() {
  echo "gate: ros"
  make build
  make test
}

case "$stage" in
  static) stage_static ;;
  python) stage_python ;;
  ros)    stage_ros ;;
  all)    stage_static; stage_python; stage_ros ;;
  *) echo "usage: $0 [static|python|ros|all]" >&2; exit 2 ;;
esac
status=PASS
