#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ ! -f "${ROOT_DIR}/install/setup.bash" ]] || source "${ROOT_DIR}/install/setup.bash"
set -u

PX4_DIR="${PX4_DIR:-${HOME}/Dev/Autopilot-p0.7-v1.17}"
PX4_REQUIRED_GIT_SHA="${PX4_REQUIRED_GIT_SHA:-d6f12ad1c4f70ad3230afd7d86e971421e02fef4}"
GZ_GUI="${GZ_GUI:-1}"
SESSION_ROOT="${SESSION_ROOT:-${ROOT_DIR}/.artifacts/simulation}"
OBSERVER_SAMPLE_HZ="${OBSERVER_SAMPLE_HZ:-2}"
POINTCLOUD_SAMPLE_EVERY="${POINTCLOUD_SAMPLE_EVERY:-10}"
AUTO_SNAPSHOT="${AUTO_SNAPSHOT:-1}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
PUBLISH_LOCAL_MAP="${PUBLISH_LOCAL_MAP:-}"
SIM_PROFILE="${SIM_PROFILE:-debug}"
SUPERVISOR_ENABLED="${SUPERVISOR_ENABLED:-0}"
case "${SIM_PROFILE}" in
  debug)
    ;;
  benchmark|runtime)
    AUTO_SNAPSHOT=0
    ENABLE_RVIZ=0
    ;;
  *)
    echo "Unsupported SIM_PROFILE: ${SIM_PROFILE} (expected debug, benchmark, or runtime)." >&2
    exit 64
    ;;
esac
if [[ "${SIM_PROFILE}" == "runtime" ]]; then
  SUPERVISOR_ENABLED=1
fi
export OBSERVER_SAMPLE_HZ POINTCLOUD_SAMPLE_EVERY AUTO_SNAPSHOT ENABLE_RVIZ PUBLISH_LOCAL_MAP
export SIM_PROFILE SUPERVISOR_ENABLED
CONFIG="${ROOT_DIR}/tools/simulation/config/px4_mid360_observer.yaml"

for command_name in ros2 python3 gz MicroXRCEAgent; do
  command -v "${command_name}" >/dev/null || { echo "Missing command: ${command_name}" >&2; exit 127; }
done
[[ -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]] || {
  echo "PX4 SITL binary missing under ${PX4_DIR}; build px4_sitl_default first." >&2; exit 66; }
[[ -e "${PX4_DIR}/.git" ]] || { echo "PX4 source repository missing under ${PX4_DIR}." >&2; exit 66; }
PX4_ACTUAL_GIT_SHA="$(git -C "${PX4_DIR}" rev-parse HEAD)"
[[ "${PX4_ACTUAL_GIT_SHA}" == "${PX4_REQUIRED_GIT_SHA}" ]] || {
  echo "PX4 SHA mismatch: required ${PX4_REQUIRED_GIT_SHA}, actual ${PX4_ACTUAL_GIT_SHA}." >&2
  exit 66
}
[[ -z "$(git -C "${PX4_DIR}" status --porcelain)" ]] || {
  echo "PX4 source repository is dirty: ${PX4_DIR}" >&2; exit 66; }
[[ -f "${ROOT_DIR}/install/setup.bash" ]] || { echo "Workspace install/setup.bash missing; run make build." >&2; exit 66; }

arguments="$(python3 -c 'import json,os; print(json.dumps({"OBSERVER_SAMPLE_HZ":os.environ.get("OBSERVER_SAMPLE_HZ","2"),"POINTCLOUD_SAMPLE_EVERY":os.environ.get("POINTCLOUD_SAMPLE_EVERY","10"),"AUTO_SNAPSHOT":os.environ.get("AUTO_SNAPSHOT","1"),"ENABLE_RVIZ":os.environ.get("ENABLE_RVIZ","1"),"PUBLISH_LOCAL_MAP":os.environ.get("PUBLISH_LOCAL_MAP",""),"SIM_PROFILE":os.environ.get("SIM_PROFILE","debug"),"SUPERVISOR_ENABLED":os.environ.get("SUPERVISOR_ENABLED","0")}))')"
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

launch_terminal xrce_agent "Micro XRCE-DDS Agent | $(basename "${SESSION_DIR}")" \
  MicroXRCEAgent udp4 -p 8888

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
launch_terminal px4_ingress "PX4 odometry ingress | $(basename "${SESSION_DIR}")" \
  ros2 run px4_odometry_bridge px4_odometry_bridge_node --ros-args \
  -p use_sim_time:=true -p simulation_clock:=true

deadline=$((SECONDS+45))
until timeout 5 ros2 topic echo --no-daemon --once /px4/estimator_odometry >/dev/null 2>&1; do
  (( SECONDS < deadline )) || { echo "PX4 odometry bridge startup timeout" >&2; exit 70; }
  sleep 1
done

# The topic is volatile, so wait for a real simulation-epoch sample before
# starting FAST-LIO. This keeps the startup prior contract from racing the
# first PX4 publication while retaining the estimator's timestamp gates.
launch_terminal fast_lio "FAST-LIO | $(basename "${SESSION_DIR}")" \
  ros2 launch navigation_bringup fast_lio.launch.py \
  config_file:="${LIO_CONFIG}" use_sim_time:=true \
  livox_mount_xyz:="0 0 0.28" livox_mount_rpy:="0 0 0"

deadline=$((SECONDS+45))
until timeout 5 ros2 topic echo --no-daemon --once /lidar/imu >/dev/null 2>&1 &&
      timeout 5 ros2 topic echo --no-daemon --once /lidar/points >/dev/null 2>&1; do
  (( SECONDS < deadline )) || { echo "ROS sensor startup timeout" >&2; exit 70; }
  sleep 1
done

if [[ "${SUPERVISOR_ENABLED}" == "1" ]]; then
  launch_terminal supervisor "P0.8 odometry supervisor | $(basename "${SESSION_DIR}")" \
    ros2 launch odometry_supervisor odometry_supervisor.launch.py use_sim_time:=true
  deadline=$((SECONDS+30))
  until timeout 5 ros2 topic echo --no-daemon --once /navigation/odometry_supervisor/status >/dev/null 2>&1; do
    (( SECONDS < deadline )) || { echo "Odometry supervisor startup timeout" >&2; exit 70; }
    sleep 1
  done
fi

if [[ "${SIM_PROFILE}" == "debug" ]]; then
  launch_terminal observer "PX4 MID-360 observer | $(basename "${SESSION_DIR}")" \
    python3 tools/simulation/sim_observer.py --session "${SESSION_DIR}" --config "${CONFIG}" \
    --sample-hz "${OBSERVER_SAMPLE_HZ}" --pointcloud-sample-every "${POINTCLOUD_SAMPLE_EVERY}"
  launch_terminal status "PX4 MID-360 status | $(basename "${SESSION_DIR}")" \
    bash tools/simulation/live_px4_mid360_status.sh
  if [[ "${ENABLE_RVIZ}" == "1" && "${GZ_GUI}" == "1" ]]; then
    launch_terminal rviz "RViz PX4 MID-360 | $(basename "${SESSION_DIR}")" \
      rviz2 -d "${ROOT_DIR}/src/navigation_estimator/fast_lio_ros/rviz/fast_lio.rviz"
  fi
fi
echo "Session started: ${SESSION_DIR}"
echo "Stop: make sim-px4-mid360-stop"
