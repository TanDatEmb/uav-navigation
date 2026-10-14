#!/usr/bin/env bash
# Start Gazebo with the project-owned x500_mid360 model, then attach standard
# PX4 x500 SITL (autostart 4001) to the existing Gazebo entity.
# No PX4 source patch, custom airframe, or copy into the PX4 submodule is needed.
#
# Environment:
#   UAVNAV_PX4_PROFILE    required: PX4 localization profile file (config/px4/profiles/{gps,no_gps}.yaml),
#                         validated by tools/uavnav/px4_profile.py (D20, D28, D33/O23); invalid -> exit 2
#   UAVNAV_SPAWN_YAW_RAD  optional: ENU spawn yaw of x500_mid360 [rad], applied with the world's
#                         set_pose service before PX4 attaches (F34 check, D33/O24)
#   UAVNAV_SYS_FAILURE_EN optional: 1 sets SYS_FAILURE_EN=1 (failure injection; the harness sets it for a GNSS
#                         outage run only, S1c Task 1); anything else but empty refuses start
#   GZ_GUI=0              headless Gazebo server (the SITL harness always sets it)
#   SESSION_DIR           run directory; Gazebo log in $SESSION_DIR/logs, PX4 working directory
#                         (fresh parameters, PX4 logs) in $SESSION_DIR/px4 (default SESSION_DIR:
#                         log/px4_mid360/<UTC stamp>); a working directory that already holds parameters refuses start
# Started and torn down by tools/uavnav/sitl_hover.sh, which owns the process groups.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-/usr/bin/python3}"
CHECK_ONLY=0
case "${1:-}" in
  "") ;;
  --check-only) CHECK_ONLY=1 ;;  # validate paths, Gazebo, profile and spawn yaw; start nothing
  *) echo "usage: $0 [--check-only]" >&2; exit 2 ;;
esac

detect_px4_dir() {
  if [[ -n "${PX4_DIR:-}" ]]; then printf '%s\n' "${PX4_DIR}"
  elif [[ -d "${HOME}/Dev/Autopilot" ]]; then printf '%s\n' "${HOME}/Dev/Autopilot"
  else printf '%s\n' "${HOME}/Autopilot"; fi
}

sanitize_path_for_gzsim() {
  local old_path="${PATH:-}"
  local cleaned=""
  local part
  IFS=':' read -r -a _parts <<< "${old_path}"
  for part in "${_parts[@]}"; do
    [[ -z "${part}" ]] && continue
    [[ "${part}" == "/opt/ros/jazzy/opt/gz_tools_vendor/bin" ]] && continue
    if [[ -z "${cleaned}" ]]; then
      cleaned="${part}"
    else
      cleaned="${cleaned}:${part}"
    fi
  done
  export PATH="${cleaned}"
}

detect_gz_command() {
  local candidate
  if [[ -n "${GZ_COMMAND:-}" ]]; then
    candidate="${GZ_COMMAND}"
    if command -v "${candidate}" >/dev/null 2>&1 && env -u GZ_CONFIG_PATH "${candidate}" sim --versions >/dev/null 2>&1; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  fi
  # ROS may prepend a non-simulator `gz` wrapper; scan every visible binary.
  while IFS= read -r candidate; do
    [[ -z "${candidate}" ]] && continue
    if env -u GZ_CONFIG_PATH "${candidate}" sim --versions >/dev/null 2>&1; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done < <(type -aP gz 2>/dev/null)

  # Fallback for shells where `type -aP` is unavailable.
  if command -v gz >/dev/null 2>&1 && env -u GZ_CONFIG_PATH gz sim --versions >/dev/null 2>&1; then
    printf '%s\n' "gz"
    return 0
  fi
  return 1
}

gz_run() {
  env -u GZ_CONFIG_PATH "${GZ_COMMAND}" "$@"
}

PX4_DIR="$(detect_px4_dir)"
GZ_COMMAND="$(detect_gz_command || true)"
PX4_BUILD="${PX4_DIR}/build/px4_sitl_default"
PX4_BIN="${PX4_BUILD}/bin/px4"
PX4_ROOTFS="${PX4_BUILD}/rootfs"
PX4_GZ_ENV="${PX4_ROOTFS}/gz_env.sh"
UAV_MODELS="${WS_DIR}/src/sim/uav_simulation/models"
UAV_WORLDS="${WS_DIR}/src/sim/uav_simulation/worlds"
WORLD_NAME="${PX4_GZ_WORLD:-px4_lio_smoke}"
WORLD_FILE="${UAV_WORLDS}/${WORLD_NAME}.sdf"
MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_mid360}"

for required in "${PX4_BIN}" "${PX4_ROOTFS}" "${PX4_GZ_ENV}" "${PX4_BUILD}/etc" \
  "${UAV_MODELS}/x500_mid360/model.sdf" "${UAV_MODELS}/lidar_mid360/model.sdf" \
  "${WORLD_FILE}"; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: required path is missing: ${required}" >&2
    [[ "${required}" == "${PX4_BIN}" ]] && echo "Build: cd ${PX4_DIR} && make px4_sitl_default" >&2
    exit 1
  fi
done
if [[ -z "${GZ_COMMAND}" ]]; then
  echo "ERROR: Gazebo Simulator is unavailable. Install Gazebo Harmonic so 'gz sim' works, or set GZ_COMMAND to a simulator-capable binary." >&2
  echo "Hint: the currently visible 'gz' only provides topic/service/log/param/msg tools, not 'sim'." >&2
  exit 1
fi
echo "Gazebo command : ${GZ_COMMAND}"

# Localization profile (tier c, SYSTEM_DESIGN §0, §4.1, §6.2). Validate it before anything starts.
if [[ -z "${UAVNAV_PX4_PROFILE:-}" ]]; then
  echo "ERROR: UAVNAV_PX4_PROFILE is required (config/px4/profiles/gps.yaml or no_gps.yaml)" >&2
  exit 2
fi
if ! PROFILE_ENV="$("${PYTHON}" "${WS_DIR}/tools/uavnav/px4_profile.py" --profile "${UAVNAV_PX4_PROFILE}" --format env)"; then
  echo "ERROR: invalid PX4 profile ${UAVNAV_PX4_PROFILE} (see the message above)" >&2
  exit 2
fi
if [[ -n "${UAVNAV_SPAWN_YAW_RAD:-}" ]]; then
  # gz.msgs.Pose request for the world's set_pose service: the include pose of the model in the
  # world file with its yaw replaced. A non-finite or unparsable yaw refuses start.
  if ! SPAWN_POSE_REQ="$("${PYTHON}" - "${WORLD_FILE}" "${MODEL_NAME}" "${UAVNAV_SPAWN_YAW_RAD}" <<'PY'
import math, sys, xml.etree.ElementTree as et
world, name, yaw_text = sys.argv[1:4]
yaw = float(yaw_text)
if not math.isfinite(yaw) or abs(yaw) > 2 * math.pi:
    sys.exit(f"UAVNAV_SPAWN_YAW_RAD must be finite and within +-2*pi, got {yaw_text}")
for include in et.parse(world).getroot().iter("include"):
    if include.findtext("name") == name:
        x, y, z, roll, pitch, _ = (float(v) for v in (include.findtext("pose") or "0 0 0 0 0 0").split())
        break
else:
    sys.exit(f"model {name} is not included in {world}")
cr, sr = math.cos(roll / 2), math.sin(roll / 2)
cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
print(f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
      f'orientation: {{x: {q[0]:.9f}, y: {q[1]:.9f}, z: {q[2]:.9f}, w: {q[3]:.9f}}}')
PY
)"; then
    echo "ERROR: invalid UAVNAV_SPAWN_YAW_RAD=${UAVNAV_SPAWN_YAW_RAD}" >&2
    exit 2
  fi
fi
case "${UAVNAV_SYS_FAILURE_EN:-}" in
  ""|1) ;;
  *) echo "ERROR: UAVNAV_SYS_FAILURE_EN must be 1 or unset, got ${UAVNAV_SYS_FAILURE_EN}" >&2; exit 2 ;;
esac
if [[ "${CHECK_ONLY}" == "1" ]]; then
  echo "PX4 binary     : ${PX4_BIN} (ok)"
  echo "PX4 rootfs     : ${PX4_ROOTFS} (ok; gz_env.sh ok; etc ${PX4_BUILD}/etc ok)"
  echo "World / model  : ${WORLD_FILE} / ${MODEL_NAME} (ok)"
  echo "PX4 profile    : ${UAVNAV_PX4_PROFILE} (valid)"
  echo "  ${PROFILE_ENV//$'\n'/$'\n'  }"
  if [[ -n "${SPAWN_POSE_REQ:-}" ]]; then echo "Spawn pose     : ${SPAWN_POSE_REQ}"; fi
  if [[ -n "${UAVNAV_SYS_FAILURE_EN:-}" ]]; then echo "Harness        : SYS_FAILURE_EN=1 (failure injection)"; fi
  exit 0
fi

# PX4's generated environment supplies its Gazebo plugins/server config and its
# upstream x500 resources. Initialise variables first because this script uses
# `set -u` while PX4's generated environment appends to them.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${GZ_SIM_SERVER_CONFIG_PATH:-}"
# shellcheck disable=SC1090
source "${PX4_GZ_ENV}"
EXTRA_MODELS="${PX4_GZ_EXTRA_RESOURCE_PATH:-}"
if [[ -n "${EXTRA_MODELS}" ]]; then
  export GZ_SIM_RESOURCE_PATH="${EXTRA_MODELS}:${UAV_MODELS}:${UAV_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"
else
  export GZ_SIM_RESOURCE_PATH="${UAV_MODELS}:${UAV_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"
fi
export GZ_IP="${GZ_IP:-127.0.0.1}"
# ROS setup may point GZ_CONFIG_PATH to tools-only vendor entries.
# Clear it so Gazebo Sim and PX4's gzsim rcS checks use the real simulator libs.
unset GZ_CONFIG_PATH
sanitize_path_for_gzsim

# The run's directory, or a new UTC-stamped one when started by hand: logs and the PX4 working directory.
RUN_BASE="${SESSION_DIR:-${WS_DIR}/log/px4_mid360/$(date -u +%Y%m%dT%H%M%SZ)}"
GZ_LOG_DIR="${RUN_BASE}/logs"
mkdir -p "${GZ_LOG_DIR}"
GZ_LOG="${GZ_LOG_DIR}/gazebo.log"

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "${GZ_PID:-}" ]] && kill -0 "${GZ_PID}" 2>/dev/null; then
    if kill -TERM "${GZ_PID}" 2>/dev/null; then
      if wait "${GZ_PID}" 2>/dev/null; then :; fi
    fi
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

if [[ "${GZ_GUI:-1}" == "0" ]]; then
  gz_run sim -r -s "${WORLD_FILE}" >"${GZ_LOG}" 2>&1 &
else
  gz_run sim -r "${WORLD_FILE}" >"${GZ_LOG}" 2>&1 &
fi
GZ_PID=$!

echo "Gazebo PID     : ${GZ_PID}"
echo "Gazebo log     : ${GZ_LOG}"
echo "World          : ${WORLD_NAME}"
echo "Existing model : ${MODEL_NAME}"
echo "PX4 autostart  : 4001 (standard x500)"

for _ in $(seq 1 30); do
  if gz_run topic -l 2>/dev/null | grep -qx "/world/${WORLD_NAME}/clock"; then break; fi
  if ! kill -0 "${GZ_PID}" 2>/dev/null; then
    echo "ERROR: Gazebo exited during startup. See ${GZ_LOG}" >&2; exit 1
  fi
  sleep 1
done
if ! gz_run topic -l 2>/dev/null | grep -qx "/world/${WORLD_NAME}/clock"; then
  echo "ERROR: timed out waiting for Gazebo world ${WORLD_NAME}. See ${GZ_LOG}" >&2; exit 1
fi

if [[ -n "${SPAWN_POSE_REQ:-}" ]]; then
  echo "Spawn yaw      : ${UAVNAV_SPAWN_YAW_RAD} rad (ENU) via /world/${WORLD_NAME}/set_pose"
  # Capture, then match: `| grep -q` could SIGPIPE gz and fail the pipeline under pipefail.
  SET_POSE_REPLY="$(gz_run service -s "/world/${WORLD_NAME}/set_pose" --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean --timeout 5000 --req "${SPAWN_POSE_REQ}" 2>&1)" || true
  if [[ "${SET_POSE_REPLY}" != *"data: true"* ]]; then
    echo "ERROR: set_pose of ${MODEL_NAME} failed (${SPAWN_POSE_REQ}): ${SET_POSE_REPLY}" >&2
    exit 1
  fi
fi

export PX4_GZ_STANDALONE=1
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_SIMULATOR=gz
export PX4_GZ_WORLD="${WORLD_NAME}"
export PX4_GZ_MODEL_NAME="${MODEL_NAME}"

###############################################################################
# PX4 parameters: the profile (EKF2, tier c) plus the harness parameters (D33/O23).
# Nothing else: every other parameter keeps the PX4 default, because PX4 starts in a
# fresh working directory each run (no parameters.bson carried over from an earlier run).
###############################################################################

# PX4_PARAM_* inherited from the caller's shell would be applied by rcS too: drop them.
while IFS= read -r inherited; do
  echo "WARNING: ignoring inherited ${inherited}" >&2
  unset "${inherited}"
done < <(compgen -e | grep '^PX4_PARAM_' || true)

eval "${PROFILE_ENV}"

# Harness only (O23), never part of a profile:
export PX4_PARAM_UXRCE_DDS_SYNCT=0   # Gazebo /clock is the only time source (D29 identity)
export PX4_PARAM_SIM_GZ_EN_ODOM=0    # Gazebo truth odometry must never compete with LIO EV
export PX4_PARAM_SIM_GZ_EN_GPS=1     # GNSS stays simulated (telemetry in no_gps, aiding in gps)
export PX4_PARAM_COM_RC_IN_MODE=1    # MAVLink-only manual control: the ROS stick stream (O24)
if [[ "${UAVNAV_SYS_FAILURE_EN:-}" == 1 ]]; then
  export PX4_PARAM_SYS_FAILURE_EN=1  # VEHICLE_CMD_INJECT_FAILURE accepted (GNSS outage run only, S1c Task 1)
fi
# NAV_DLL_ACT=0 and CBRK_SUPPLY_CHK are set by px4_mid360_startup.sh after rcS.

if [[ -v PX4_SIM_MODEL ]]; then unset PX4_SIM_MODEL; fi
if [[ -v PX4_GZ_MODEL ]]; then unset PX4_GZ_MODEL; fi

PX4_WORK_DIR="${RUN_BASE}/px4"   # always new: PX4 starts with default parameters
if [[ -e "${PX4_WORK_DIR}/parameters.bson" ]]; then
  echo "ERROR: ${PX4_WORK_DIR} already holds PX4 parameters; use a new SESSION_DIR" >&2
  exit 2
fi
mkdir -p "${PX4_WORK_DIR}"

echo
echo "PX4 is attaching to the existing Gazebo model."
echo "PX4 profile    : ${UAVNAV_PX4_PROFILE}"
compgen -e | grep '^PX4_PARAM_' | sort | while IFS= read -r name; do
  echo "PX4 parameter  : ${name#PX4_PARAM_}=${!name}"
done
echo "PX4 working dir: ${PX4_WORK_DIR}"
echo
cd "${PX4_WORK_DIR}"
# A background/headless session must keep stdin open. With immediate EOF the
# PX4 shell continuously redraws its prompt and can grow px4.log by GB/minute.
tail -f /dev/null | "${PX4_BIN}" -w "${PX4_WORK_DIR}" -s "${WS_DIR}/tools/simulation/px4_mid360_startup.sh" \
  "${PX4_BUILD}/etc"
