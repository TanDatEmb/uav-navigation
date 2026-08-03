#!/usr/bin/env bash
# Start Gazebo with the project-owned x500_mid360 model, then attach standard
# PX4 x500 SITL (autostart 4001) to the existing Gazebo entity.
# No PX4 source patch, custom airframe, or copy into the PX4 submodule is needed.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

detect_px4_dir() {
  if [[ -n "${PX4_DIR:-}" ]]; then printf '%s\n' "${PX4_DIR}"
  elif [[ -d "${HOME}/Dev/Autopilot-p0.7-v1.17" ]]; then printf '%s\n' "${HOME}/Dev/Autopilot-p0.7-v1.17"
  elif [[ -d "${HOME}/Dev/Autopilot" ]]; then printf '%s\n' "${HOME}/Dev/Autopilot"
  else printf '%s\n' "${HOME}/Autopilot"; fi
}

PX4_DIR="$(detect_px4_dir)"
PX4_REQUIRED_GIT_SHA="${PX4_REQUIRED_GIT_SHA:-d6f12ad1c4f70ad3230afd7d86e971421e02fef4}"
PX4_BUILD="${PX4_DIR}/build/px4_sitl_default"
PX4_BIN="${PX4_BUILD}/bin/px4"
PX4_ROOTFS="${PX4_BUILD}/rootfs"
PX4_GZ_ENV="${PX4_ROOTFS}/gz_env.sh"
UAV_MODELS="${WS_DIR}/src/uav_simulation/models"
UAV_WORLDS="${WS_DIR}/src/uav_simulation/worlds"
WORLD_NAME="${PX4_GZ_WORLD:-px4_lio_smoke}"
WORLD_FILE="${UAV_WORLDS}/${WORLD_NAME}.sdf"
MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_mid360}"

for required in "${PX4_DIR}/.git" "${PX4_BIN}" "${PX4_ROOTFS}" "${PX4_GZ_ENV}" \
  "${UAV_MODELS}/x500_mid360/model.sdf" "${UAV_MODELS}/lidar_mid360/model.sdf" \
  "${WORLD_FILE}"; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: required path is missing: ${required}" >&2
    [[ "${required}" == "${PX4_BIN}" ]] && echo "Build: cd ${PX4_DIR} && make px4_sitl_default" >&2
    exit 1
  fi
done
PX4_ACTUAL_GIT_SHA="$(git -C "${PX4_DIR}" rev-parse HEAD)"
[[ "${PX4_ACTUAL_GIT_SHA}" == "${PX4_REQUIRED_GIT_SHA}" ]] || {
  echo "PX4 SHA mismatch: required ${PX4_REQUIRED_GIT_SHA}, actual ${PX4_ACTUAL_GIT_SHA}." >&2
  exit 66
}
[[ -z "$(git -C "${PX4_DIR}" status --porcelain)" ]] || {
  echo "PX4 source repository is dirty: ${PX4_DIR}" >&2; exit 66; }
command -v gz >/dev/null 2>&1 || { echo "ERROR: Gazebo command 'gz' is unavailable." >&2; exit 1; }

# PX4's generated environment supplies its Gazebo plugins/server config and its
# upstream x500 resources. Initialise variables first because this script uses
# `set -u` while PX4's generated environment appends to them.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${GZ_SIM_SERVER_CONFIG_PATH:-}"
# shellcheck disable=SC1090
source "${PX4_GZ_ENV}"
export GZ_SIM_RESOURCE_PATH="${UAV_MODELS}:${UAV_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_IP="${GZ_IP:-127.0.0.1}"

GZ_LOG_DIR="${SESSION_DIR:-${WS_DIR}/log/px4_mid360}/logs"
mkdir -p "${GZ_LOG_DIR}"
GZ_LOG="${GZ_LOG_DIR}/gazebo.log"

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "${GZ_PID:-}" ]] && kill -0 "${GZ_PID}" 2>/dev/null; then
    kill -TERM "${GZ_PID}" 2>/dev/null || true
    wait "${GZ_PID}" 2>/dev/null || true
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

if [[ "${GZ_GUI:-1}" == "0" ]]; then
  gz sim -r -s "${WORLD_FILE}" >"${GZ_LOG}" 2>&1 &
else
  gz sim -r "${WORLD_FILE}" >"${GZ_LOG}" 2>&1 &
fi
GZ_PID=$!

echo "Gazebo PID     : ${GZ_PID}"
echo "Gazebo log     : ${GZ_LOG}"
echo "World          : ${WORLD_NAME}"
echo "Existing model : ${MODEL_NAME}"
echo "PX4 autostart  : 4001 (standard x500)"

for _ in $(seq 1 30); do
  if gz topic -l 2>/dev/null | grep -qx "/world/${WORLD_NAME}/clock"; then break; fi
  if ! kill -0 "${GZ_PID}" 2>/dev/null; then
    echo "ERROR: Gazebo exited during startup. See ${GZ_LOG}" >&2; exit 1
  fi
  sleep 1
done
if ! gz topic -l 2>/dev/null | grep -qx "/world/${WORLD_NAME}/clock"; then
  echo "ERROR: timed out waiting for Gazebo world ${WORLD_NAME}. See ${GZ_LOG}" >&2; exit 1
fi

export PX4_GZ_STANDALONE=1
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_SIMULATOR=gz
export PX4_GZ_WORLD="${WORLD_NAME}"
export PX4_GZ_MODEL_NAME="${MODEL_NAME}"
export PX4_PARAM_UXRCE_DDS_SYNCT=0
unset PX4_SIM_MODEL PX4_GZ_MODEL 2>/dev/null || true

echo
echo "PX4 is attaching to the existing Gazebo model."
echo "PX4 UXRCE_DDS_SYNCT: ${PX4_PARAM_UXRCE_DDS_SYNCT} (simulation clock authority)"
echo "Other terminal: ros2 launch navigation_bringup fast_lio.launch.py config_file:=${WS_DIR}/src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml use_sim_time:=true"
echo
cd "${PX4_ROOTFS}"
# A background/headless session must keep stdin open. With immediate EOF the
# PX4 shell continuously redraws its prompt and can grow px4.log by GB/minute.
tail -f /dev/null | "${PX4_BIN}"
