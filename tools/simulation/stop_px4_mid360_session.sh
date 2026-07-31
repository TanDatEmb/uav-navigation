#!/usr/bin/env bash
set -Eeuo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SESSION_ROOT="${SESSION_ROOT:-${ROOT_DIR}/.artifacts/simulation}"
SESSION_DIR="${SESSION_DIR:-${SESSION_ROOT}/latest}"
[[ -d "${SESSION_DIR}" ]] || { echo "No session at ${SESSION_DIR}"; exit 0; }
PID_DIR="${SESSION_DIR}/pids"
shopt -s nullglob

signal_groups() {
  local signal_name="$1"
  for file in "${PID_DIR}"/*.pgid; do
    local pgid name
    pgid="$(<"${file}")"; name="$(basename "${file}" .pgid)"
    [[ "${pgid}" =~ ^[0-9]+$ ]] || continue
    if kill -0 -- "-${pgid}" 2>/dev/null; then
      echo "${signal_name} ${name} group ${pgid}"
      kill "-${signal_name}" -- "-${pgid}" 2>/dev/null || true
    fi
  done
}
signal_groups INT
sleep 3
signal_groups TERM
sleep 2
signal_groups KILL

# Fail-closed fallback: only descendants whose command line contains this exact session path.
while read -r pid command_line; do
  [[ -n "${pid}" ]] || continue
  [[ "${command_line}" == *"${SESSION_DIR}"* ]] && kill -TERM "${pid}" 2>/dev/null || true
done < <(ps -u "${USER}" -o pid=,args=)

python3 "${ROOT_DIR}/tools/simulation/report_generator.py" --session "${SESSION_DIR}" || true
remaining=0
for file in "${PID_DIR}"/*.pgid; do
  pgid="$(<"${file}")"
  if kill -0 -- "-${pgid}" 2>/dev/null; then echo "orphan process group: ${pgid}"; remaining=$((remaining+1)); fi
done
python3 - "${SESSION_DIR}/summary.json" "${remaining}" <<'PY'
import json,sys
p=sys.argv[1]
try:
 d=json.load(open(p)); d["system"]["orphan_processes"]=int(sys.argv[2]); open(p,"w").write(json.dumps(d,indent=2,sort_keys=True)+"\n")
except (OSError,ValueError,KeyError): pass
PY
echo "Stopped session: $(readlink -f "${SESSION_DIR}")"
exit "${remaining}"
