#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ ! -f "${ROOT_DIR}/install/setup.bash" ]] || source "${ROOT_DIR}/install/setup.bash"
set -u

PX4_DIR="${PX4_DIR:-${HOME}/Dev/Autopilot}"
GZ_GUI="${GZ_GUI:-1}"
SESSION_ROOT="${SESSION_ROOT:-${ROOT_DIR}/.artifacts/simulation}"
OBSERVER_SAMPLE_HZ="${OBSERVER_SAMPLE_HZ:-2}"
POINTCLOUD_SAMPLE_EVERY="${POINTCLOUD_SAMPLE_EVERY:-10}"
AUTO_SNAPSHOT="${AUTO_SNAPSHOT:-1}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
PUBLISH_LOCAL_MAP="${PUBLISH_LOCAL_MAP:-}"
export OBSERVER_SAMPLE_HZ POINTCLOUD_SAMPLE_EVERY AUTO_SNAPSHOT ENABLE_RVIZ PUBLISH_LOCAL_MAP
CONFIG="${ROOT_DIR}/tools/simulation/config/px4_mid360_observer.yaml"

for command_name in ros2 python3 gz; do
  command -v "${command_name}" >/dev/null || { echo "Missing command: ${command_name}" >&2; exit 127; }
done
[[ -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]] || {
  echo "PX4 SITL binary missing under ${PX4_DIR}; build px4_sitl_default first." >&2; exit 66; }
[[ -f "${ROOT_DIR}/install/setup.bash" ]] || { echo "Workspace install/setup.bash missing; run make build." >&2; exit 66; }

arguments="$(python3 -c 'import json,os; print(json.dumps({"OBSERVER_SAMPLE_HZ":os.environ.get("OBSERVER_SAMPLE_HZ","2"),"POINTCLOUD_SAMPLE_EVERY":os.environ.get("POINTCLOUD_SAMPLE_EVERY","10"),"AUTO_SNAPSHOT":os.environ.get("AUTO_SNAPSHOT","1"),"ENABLE_RVIZ":os.environ.get("ENABLE_RVIZ","1"),"PUBLISH_LOCAL_MAP":os.environ.get("PUBLISH_LOCAL_MAP","")}))')"
SESSION_DIR="$(python3 tools/simulation/session_manager.py create --root "${SESSION_ROOT}" \
  --workspace "${ROOT_DIR}" --px4-dir "${PX4_DIR}" --gz-gui "${GZ_GUI}" \
  --arguments-json "${arguments}")"
export SESSION_DIR PX4_DIR GZ_GUI AUTO_SNAPSHOT
export ROS_LOG_DIR="${SESSION_DIR}/logs/ros"
mkdir -p "${ROS_LOG_DIR}"

launch_group() {
  local role="$1"; shift
  local log="${SESSION_DIR}/logs/${role}.log"
  nohup setsid bash -c 'echo "$$" > "$1"; shift; exec "$@"' _ \
    "${SESSION_DIR}/pids/${role}.pid" "$@" >>"${log}" 2>&1 &
  local pgid=$!
  echo "${pgid}" >"${SESSION_DIR}/pids/${role}.pgid"
  echo "Started ${role}: PGID=${pgid}"
}

launch_terminal() {
  local role="$1" title="$2"; shift 2
  if command -v xterm >/dev/null && [[ -n "${DISPLAY:-}" ]]; then
    local encoded=()
    printf -v command_line '%q ' "$@"
    launch_group "${role}" xterm -T "${title}" -geometry 120x32 -hold -e bash -lc \
      "echo \$\$ > $(printf %q "${SESSION_DIR}/pids/${role}.pid"); cd $(printf %q "${ROOT_DIR}"); source /opt/ros/jazzy/setup.bash; source install/setup.bash; exec ${command_line}"
  else
    launch_group "${role}" "$@"
  fi
}

launch_terminal px4 "PX4 + Gazebo | $(basename "${SESSION_DIR}")" \
  bash tools/simulation/run_px4_mid360.sh

deadline=$((SECONDS+60))
until gz topic -l 2>/dev/null | grep -qx '/world/px4_lio_smoke/clock'; do
  (( SECONDS < deadline )) || { echo "Gazebo clock startup timeout" >&2; exit 70; }
  sleep 1
done

BRIDGE_CONFIG="${ROOT_DIR}/src/uav_simulation/bridge/px4_mid360_bridge.yaml"
LIO_CONFIG="${ROOT_DIR}/src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml"
launch_terminal bridge "MID-360 bridge | $(basename "${SESSION_DIR}")" \
  ros2 run ros_gz_bridge parameter_bridge --ros-args -r __node:=px4_mid360_bridge \
  -p "config_file:=${BRIDGE_CONFIG}" -p use_sim_time:=true
launch_terminal fast_lio "FAST-LIO | $(basename "${SESSION_DIR}")" \
  ros2 run fast_lio_ros fast_lio_node --ros-args -r __node:=fast_lio \
  --params-file "${LIO_CONFIG}" -p use_sim_time:=true

deadline=$((SECONDS+45))
until ros2 topic list 2>/dev/null | grep -qx '/lidar/imu' &&
      ros2 topic list 2>/dev/null | grep -qx '/lidar/points'; do
  (( SECONDS < deadline )) || { echo "ROS sensor startup timeout" >&2; exit 70; }
  sleep 1
done

launch_terminal observer "PX4 MID-360 observer | $(basename "${SESSION_DIR}")" \
  python3 tools/simulation/sim_observer.py --session "${SESSION_DIR}" --config "${CONFIG}" \
  --sample-hz "${OBSERVER_SAMPLE_HZ}" --pointcloud-sample-every "${POINTCLOUD_SAMPLE_EVERY}"
launch_terminal status "PX4 MID-360 status | $(basename "${SESSION_DIR}")" \
  bash tools/simulation/live_px4_mid360_status.sh
if [[ "${ENABLE_RVIZ}" == "1" && "${GZ_GUI}" == "1" ]]; then
  launch_terminal rviz "RViz PX4 MID-360 | $(basename "${SESSION_DIR}")" \
    rviz2 -d "${ROOT_DIR}/src/navigation_estimator/fast_lio_ros/rviz/fast_lio.rviz"
fi
echo "Session started: ${SESSION_DIR}"
echo "Stop: make sim-px4-mid360-stop"
