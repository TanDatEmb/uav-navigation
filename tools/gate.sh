#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
PYTHON=${PYTHON:-/usr/bin/python3}
GATE_RESULT=FAIL

finish_gate() {
  printf 'GATE_V3_RESULT=%s\n' "$GATE_RESULT"
}

trap finish_gate EXIT

log_header() {
  printf 'gate: head=%s\n' "$(git rev-parse HEAD)"
  printf 'gate: dirty_paths=%s\n' "$(git status --porcelain | wc -l)"
}

static_gate() {
  printf '%s\n' 'gate: static'
  if git rev-parse --verify origin/main >/dev/null 2>&1; then
    git diff --check origin/main...HEAD -- . ':!artifacts/**'
  else
    printf '%s\n' 'gate: static: origin/main unavailable; baseline diff check not measured' >&2
  fi
  git diff --check -- . ':!artifacts/**'
  "$PYTHON" tools/validate_runtime_safety_ledger.py
  "$PYTHON" tools/check_mission_authority_cut.py
  "$PYTHON" tools/refactor/check_citations.py . docs/refactor
  "$PYTHON" tools/check_dependency_direction.py
}

python_gate() {
  printf '%s\n' 'gate: python'
  "$PYTHON" --version
  local status=0
  "$PYTHON" -m unittest discover -s tools/tests -p 'test_*.py' -v || status=$?
  "$PYTHON" -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v || status=$?
  return "$status"
}

ros_gate() {
  printf '%s\n' 'gate: ros'
  command -v colcon >/dev/null || {
    printf '%s\n' 'gate: ros: colcon is required' >&2
    return 1
  }
  git rev-parse --verify origin/main >/dev/null || {
    printf '%s\n' 'gate: ros: origin/main is required' >&2
    return 1
  }

  local -a packages=()
  local colcon_list_output
  colcon_list_output="$(colcon list)" || {
    printf '%s\n' 'gate: ros: colcon list failed' >&2
    return 1
  }
  if [[ -n "${PACKAGES:-}" ]]; then
    read -r -a packages <<< "$PACKAGES"
  else
    declare -A package_paths=()
    local package_name package_path changed_path reverse_name
    while read -r package_name package_path _; do
      [[ -n "$package_name" && -n "$package_path" ]] || continue
      package_paths["$package_name"]="$package_path"
    done <<< "$colcon_list_output"

    declare -A changed=()
    local changed_paths
    changed_paths="$({
      git diff --name-only origin/main...HEAD
      git diff --name-only
      git diff --cached --name-only
      git ls-files -o --exclude-standard -- src
    } | sort -u)" || {
      printf '%s\n' 'gate: ros: failed to collect changed paths' >&2
      return 1
    }
    while IFS= read -r changed_path; do
      [[ -n "$changed_path" ]] || continue
      for package_name in "${!package_paths[@]}"; do
        package_path="${package_paths[$package_name]}"
        if [[ "$changed_path" == "$package_path"/* ]]; then
          changed["$package_name"]=1
        fi
      done
    done <<< "$changed_paths"

    declare -A selected=()
    for package_name in "${!changed[@]}"; do
      selected["$package_name"]=1
      local reverse_packages
      reverse_packages="$(colcon list --packages-above "$package_name")" || {
        printf 'gate: ros: reverse dependency query failed for %s\n' "$package_name" >&2
        return 1
      }
      while read -r reverse_name; do
        [[ -n "$reverse_name" ]] && selected["$reverse_name"]=1
      done < <(printf '%s\n' "$reverse_packages" | awk '{print $1}')
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
  for package_name in "${packages[@]}"; do
    rm -rf "build/$package_name/test_results"
  done
  rm -rf build/px4_ros2_cpp/test_results
  printf '%s\n' 'gate: ros: px4_ros2_cpp CTest is attachment-only (G1) and is skipped from the blocking result'
  colcon test --packages-select "${packages[@]}" --packages-skip px4_ros2_cpp --event-handlers console_direct+
  # Blocking result: only the selected packages' own result directories (G1:
  # px4_ros2_cpp results, stale or fresh, must never enter the verdict).
  local result_status=0
  for package_name in "${packages[@]}"; do
    [[ "$package_name" == px4_ros2_cpp ]] && continue
    [[ -d "build/$package_name/test_results" ]] || continue
    colcon test-result --verbose --test-result-base "build/$package_name/test_results" || result_status=1
  done
  ((result_status == 0)) || {
    printf '%s\n' 'gate: ros: blocking CTest results contain failures' >&2
    return 1
  }
  if grep -Eq '^px4_ros2_cpp[[:space:]]' <<< "$colcon_list_output"; then
    mkdir -p artifacts/gate
    local px4_log=artifacts/gate/px4_ros2_cpp-ctest.log
    local px4_rc=0
    if colcon test --packages-select px4_ros2_cpp --event-handlers console_direct+ >"$px4_log" 2>&1; then
      :
    else
      px4_rc=$?
    fi
    printf 'gate: ros: px4_ros2_cpp attachment rc=%s log=%s\n' "$px4_rc" "$px4_log"
  else
    printf '%s\n' 'gate: ros: px4_ros2_cpp is not present; attachment not run'
  fi
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
GATE_RESULT=PASS
