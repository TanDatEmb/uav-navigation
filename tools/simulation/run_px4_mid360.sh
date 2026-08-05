#!/usr/bin/env bash
# Start Gazebo with the project-owned x500_mid360 model, then attach standard
# PX4 x500 SITL (autostart 4001) to the existing Gazebo entity.
# No PX4 source patch, custom airframe, or copy into the PX4 submodule is needed.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

detect_px4_dir() {
  if [[ -n "${PX4_DIR:-}" ]]; then printf '%s\n' "${PX4_DIR}"
  elif [[ -d "${HOME}/Dev/Autopilot" ]]; then printf '%s\n' "${HOME}/Dev/Autopilot"
  else printf '%s\n' "${HOME}/Autopilot"; fi
}

PX4_DIR="$(detect_px4_dir)"
PX4_BUILD="${PX4_DIR}/build/px4_sitl_default"
PX4_BIN="${PX4_BUILD}/bin/px4"
PX4_ROOTFS="${PX4_BUILD}/rootfs"
PX4_GZ_ENV="${PX4_ROOTFS}/gz_env.sh"
UAV_MODELS="${WS_DIR}/src/uav_simulation/models"
UAV_WORLDS="${WS_DIR}/src/uav_simulation/worlds"
WORLD_NAME="${PX4_GZ_WORLD:-px4_lio_smoke}"
WORLD_FILE="${UAV_WORLDS}/${WORLD_NAME}.sdf"
MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_mid360}"

for required in "${PX4_BIN}" "${PX4_ROOTFS}" "${PX4_GZ_ENV}" \
  "${UAV_MODELS}/x500_mid360/model.sdf" "${UAV_MODELS}/lidar_mid360/model.sdf" \
  "${WORLD_FILE}"; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: required path is missing: ${required}" >&2
    [[ "${required}" == "${PX4_BIN}" ]] && echo "Build: cd ${PX4_DIR} && make px4_sitl_default" >&2
    exit 1
  fi
done
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
    if kill -TERM "${GZ_PID}" 2>/dev/null; then
      if wait "${GZ_PID}" 2>/dev/null; then :; fi
    fi
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

###############################################################################
# PX4 Parameters
###############################################################################

# Simulation clock authority
export PX4_PARAM_UXRCE_DDS_SYNCT=0

###############################################################################
# MODE 1 : Standard PX4
# ---------------------------------------------------------------------------
# Use:
#   - GPS
#   - Barometer
#   - Magnetometer
#   - Range Finder
#   - All standard EKF2 sensors
#
# Uncomment this block and comment MODE 2.
###############################################################################

# export PX4_PARAM_SIM_GZ_EN_GPS=1
# export PX4_PARAM_SIM_GPS_USED=30
# export PX4_PARAM_EKF2_GPS_CTRL=7
# export PX4_PARAM_EKF2_BARO_CTRL=1
# export PX4_PARAM_EKF2_RNG_CTRL=1
# export PX4_PARAM_EKF2_MAG_TYPE=0
# export PX4_PARAM_EKF2_HGT_REF=1
# export PX4_PARAM_EKF2_EV_CTRL=0


###############################################################################
# MODE 2 : External Vision Only (LIO / VIO)
# ---------------------------------------------------------------------------
# Disable ALL navigation sensors from Gazebo and use ONLY External Vision.
#
# Sensors disabled:
#   - GPS
#   - Barometer
#   - Magnetometer
#   - Range Finder
#
# Navigation source:
#   - External Vision (ROS2 LIO / VIO)
#
# Comment MODE 1 above and uncomment this block.
###############################################################################

export PX4_PARAM_SIM_GZ_EN_GPS=0
export PX4_PARAM_SIM_GPS_USED=0
export PX4_PARAM_EKF2_GPS_CTRL=0
export PX4_PARAM_EKF2_BARO_CTRL=0
export PX4_PARAM_EKF2_RNG_CTRL=0
export PX4_PARAM_EKF2_MAG_TYPE=5
export PX4_PARAM_EKF2_HGT_REF=3
export PX4_PARAM_EKF2_EV_CTRL=15
#
# Optional:
# export PX4_PARAM_SIM_GZ_EN_ODOM=0

if [[ -v PX4_SIM_MODEL ]]; then unset PX4_SIM_MODEL; fi
if [[ -v PX4_GZ_MODEL ]]; then unset PX4_GZ_MODEL; fi

echo
echo "PX4 is attaching to the existing Gazebo model."
echo "PX4 UXRCE_DDS_SYNCT: ${PX4_PARAM_UXRCE_DDS_SYNCT} (simulation clock authority)"
echo "PX4 visual odometry: ROS LIO only (SIM_GZ_EN_ODOM=0; EKF2 EV=15; GPS/baro/range fusion=0)"
echo "Runtime stack is started by tools/runtime/runner.py."
echo
cd "${PX4_ROOTFS}"
# A background/headless session must keep stdin open. With immediate EOF the
# PX4 shell continuously redraws its prompt and can grow px4.log by GB/minute.
tail -f /dev/null | "${PX4_BIN}" -s "${WS_DIR}/tools/simulation/px4_mid360_startup.sh"
