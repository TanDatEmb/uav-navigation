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
  command -v colcon >/dev/null || {
    printf '%s\n' 'gate: ros: colcon is required' >&2
    return 1
  }

  local -a packages=()
  if [[ -n "${PACKAGES:-}" ]]; then
    read -r -a packages <<< "$PACKAGES"
  else
    declare -A package_paths=()
    local package_name package_path changed_path reverse_name
    while read -r package_name package_path _; do
      [[ -n "$package_name" && -n "$package_path" ]] || continue
      package_paths["$package_name"]="$package_path"
    done < <(colcon list)

    declare -A changed=()
    while IFS= read -r changed_path; do
      [[ -n "$changed_path" ]] || continue
      for package_name in "${!package_paths[@]}"; do
        package_path="${package_paths[$package_name]}"
        if [[ "$changed_path" == "$package_path"/* ]]; then
          changed["$package_name"]=1
        fi
      done
    done < <(
      {
        git diff --name-only origin/main...HEAD
        git diff --name-only
        git diff --cached --name-only
        git ls-files -o --exclude-standard -- src
      } | sort -u
    )

    declare -A selected=()
    for package_name in "${!changed[@]}"; do
      selected["$package_name"]=1
      while read -r reverse_name; do
        [[ -n "$reverse_name" ]] && selected["$reverse_name"]=1
      done < <(colcon list --packages-above "$package_name" | awk '{print $1}')
    done
    if [[ "${#selected[@]}" -eq 0 ]]; then
      printf '%s\n' 'gate: ros: no changed ROS package; nothing to build'
      return 0
    fi
    mapfile -t packages < <(printf '%s\n' "${!selected[@]}" | sort)
  fi

  ((${#packages[@]} > 0)) || {
    printf '%s\n' 'gate: ros: PACKAGES is empty' >&2
    return 64
  }
  printf 'gate: ros packages='; printf '%s ' "${packages[@]}"; printf '\n'
  colcon build --packages-select "${packages[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
  printf '%s\n' 'gate: ros: px4_ros2_cpp CTest is attachment-only (G1) and is skipped'
  colcon test --packages-select "${packages[@]}" --packages-skip px4_ros2_cpp --event-handlers console_direct+
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
