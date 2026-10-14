#!/usr/bin/env bash
# SITL harness (S1b Task 3 probe, Task 11 hover). SYSTEM_DESIGN §0, §6.2 (c), §7.2; D29; D33 (O19, O20, O23, O24).
#
#   tools/uavnav/sitl_hover.sh --probe --profile gps|no_gps [--hover-mode posctl|hold] [--dry-run]
#   tools/uavnav/sitl_hover.sh --hover --profile gps|no_gps [--spawn-yaw-rad RAD] [--pattern hover|altitude_sweep]
#                              [--hover-mode posctl|hold] [--gnss-outage-s S] [--dry-run]
#   (make sitl-probe PROFILE=gps [HOVER_MODE=hold] [DRY_RUN=1];
#    make sitl-hover PROFILE=gps [SPAWN_YAW_RAD=1.2] [PATTERN=hover|altitude_sweep] [HOVER_MODE=hold]
#                    [GNSS_OUTAGE_S=10] [DRY_RUN=1])
#
# Both start, each in its own session and process group (setsid), with GZ_IP=127.0.0.1 and ROS_LOG_DIR in the
# run directory: MicroXRCEAgent udp4 -p 8888; tools/simulation/run_px4_mid360.sh (GZ_GUI=0; Gazebo + PX4 SITL;
# the spawn yaw, if any).
#   --probe then starts ros_gz_bridge parameter_bridge with uav_simulation's px4_mid360_control_bridge.yaml,
#     uav_simulation gz_lidar_bridge and tools/uavnav/sitl_probe.py; bounded at 240 s.
#   --hover then starts `ros2 launch uavnav_bringup s1_estimation.launch.py` (the same two gz bridges, lio and
#     px4_bridge, use_sim_time; the event logs in the run directory) and tools/uavnav/sitl_hover.py; bounded at
#     300 s. --pattern altitude_sweep (S1b Task 13; gps profile and posctl only) flies 2.5 m steps to 20 m, 10 s
#     each, and is judged by the data checks only (sitl_checks.py --sweep); tools/uavnav/lio_calibration.py
#     analyses its directory. --gnss-outage-s S (S1c Task 1; gps, pattern hover, 0 < S <= 30): the driver injects
#     a GNSS outage (VEHICLE_CMD_INJECT_FAILURE, unit GPS, OFF then OK S s later) 20 s into the hover, and PX4 is
#     started with SYS_FAILURE_EN=1 (UAVNAV_SYS_FAILURE_EN=1 to run_px4_mid360.sh), for this run only.
# The ROS 2 processes (gz bridges, the launch, the driver) get FASTRTPS_DEFAULT_PROFILES_FILE=config/ros/fastdds_sitl.xml:
# a 16 MB shared-memory segment, so a 645 KB best-effort cloud is not lost in transport (Task 11; without it ~15 %
# of the clouds never reached lio). MicroXRCEAgent and PX4 keep their own DDS defaults.
# The run is bounded (wall, from the first process start); every group it started is torn down on EXIT, INT
# and TERM (SIGINT, then SIGTERM, then SIGKILL, by process group and by session, in reverse start order), and
# the teardown verifies with pgrep that none of its processes remain. It never signals a process it did not
# start. Then tools/uavnav/sitl_checks.py writes summary.json.
#
# Artifacts: log/sitl/<UTC stamp>-<profile>-probe/ or log/sitl/<UTC stamp>-<profile>-<pattern>[-yaw<RAD>][-gnssout<S>]/ only
# (process logs, ROS logs, PX4 working directory, records, event logs, *_meta.json, summary.json, harness.log).
# Before anything starts: log/sitl/.lock (flock; a second harness exits 2), tools/uavnav/sitl_guard.py (colcon build/test or another SITL process, matched on
# argv, own ancestors excluded), UDP 8888 free, no Gazebo world clock on GZ_IP, no SITL topic on the ROS graph.
# Exit: 0 PASS (every check and every driver phase); 1 FAIL (a check, a phase, a process or the teardown);
# 2 refused (usage, a colcon build or test is running, another SITL is running, a prerequisite is missing);
# 124 hard timeout; 130 interrupted.
# --dry-run runs every precheck (read-only) and prints the commands it would run; it starts nothing.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
PYTHON="${PYTHON:-/usr/bin/python3}"
PROBE_TIMEOUT_S=240        # hard bound of the probe run, from the first process start (wall seconds)
HOVER_TIMEOUT_S=300        # hard bound of the hover run (plan Task 11)
PROBE_MARGIN_S=10          # the driver's own budget ends this long before the hard bound
GRACE_INT_S=10             # teardown: wait after SIGINT, then SIGTERM
GRACE_TERM_S=5             # teardown: wait after SIGTERM, then SIGKILL
XRCE_PORT=8888

usage() {
  echo "usage: $0 --probe --profile gps|no_gps [--hover-mode posctl|hold] [--dry-run]" >&2
  echo "       $0 --hover --profile gps|no_gps [--spawn-yaw-rad RAD] [--pattern hover|altitude_sweep] [--hover-mode posctl|hold] [--gnss-outage-s S] [--dry-run]" >&2
  exit 2
}

MODE="" PROFILE="" HOVER_MODE="posctl" DRY_RUN=0 PATTERN="hover" SPAWN_YAW_RAD="${UAVNAV_SPAWN_YAW_RAD:-}" GNSS_OUTAGE_S=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --probe) [[ -z "${MODE}" ]] || usage; MODE=probe; shift ;;
    --hover) [[ -z "${MODE}" ]] || usage; MODE=hover; shift ;;
    --profile) [[ $# -ge 2 ]] || usage; PROFILE="$2"; shift 2 ;;
    --hover-mode) [[ $# -ge 2 ]] || usage; HOVER_MODE="$2"; shift 2 ;;
    --spawn-yaw-rad) [[ $# -ge 2 ]] || usage; SPAWN_YAW_RAD="$2"; shift 2 ;;
    --pattern) [[ $# -ge 2 ]] || usage; PATTERN="$2"; shift 2 ;;
    --gnss-outage-s) [[ $# -ge 2 ]] || usage; GNSS_OUTAGE_S="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage ;;
    *) echo "sitl_hover: unknown argument: $1" >&2; usage ;;
  esac
done
[[ "${MODE}" == probe || "${MODE}" == hover ]] || { echo "sitl_hover: --probe or --hover is required" >&2; usage; }
[[ "${PROFILE}" == gps || "${PROFILE}" == no_gps ]] || { echo "sitl_hover: --profile must be gps or no_gps" >&2; usage; }
[[ "${HOVER_MODE}" == posctl || "${HOVER_MODE}" == hold ]] || { echo "sitl_hover: --hover-mode must be posctl or hold" >&2; usage; }
case "${PATTERN}" in
  hover) ;;
  altitude_sweep)
    [[ "${MODE}" == hover ]] || { echo "sitl_hover: --pattern applies to --hover only" >&2; usage; }
    # Plan Task 13: gps only, because EV is lost when LIO degrades; the steps are flown with the throttle stick.
    [[ "${PROFILE}" == gps && "${HOVER_MODE}" == posctl ]] \
      || { echo "sitl_hover: --pattern altitude_sweep needs --profile gps and --hover-mode posctl" >&2; usage; } ;;
  *) echo "sitl_hover: --pattern must be hover or altitude_sweep" >&2; usage ;;
esac
if [[ -n "${SPAWN_YAW_RAD}" ]] && ! "${PYTHON}" -c 'import math, sys; v = float(sys.argv[1]); sys.exit(0 if math.isfinite(v) and abs(v) <= 2 * math.pi else 1)' "${SPAWN_YAW_RAD}" 2>/dev/null; then
  echo "sitl_hover: --spawn-yaw-rad must be a finite number of radians in [-2 pi, 2 pi]: ${SPAWN_YAW_RAD}" >&2; usage
fi
if [[ -n "${GNSS_OUTAGE_S}" ]]; then
  [[ "${MODE}" == hover && "${PROFILE}" == gps && "${PATTERN}" == hover ]] \
    || { echo "sitl_hover: --gnss-outage-s needs --hover --profile gps --pattern hover" >&2; usage; }
  "${PYTHON}" -c 'import math, sys; v = float(sys.argv[1]); sys.exit(0 if math.isfinite(v) and 0 < v <= 30 else 1)' "${GNSS_OUTAGE_S}" 2>/dev/null \
    || { echo "sitl_hover: --gnss-outage-s must be a number of seconds in (0, 30]: ${GNSS_OUTAGE_S}" >&2; usage; }
fi
PROFILE_FILE="${WS_DIR}/config/px4/profiles/${PROFILE}.yaml"
LIO_CONFIG="${WS_DIR}/config/lio/sim.yaml"
LIO_NODE_CONFIG="${WS_DIR}/config/lio/node_sim.yaml"
BRIDGE_CONFIG="${WS_DIR}/config/px4_bridge/sim.yaml"
DDS_PROFILE="${WS_DIR}/config/ros/fastdds_sitl.xml"

refuse() { echo "sitl_hover: REFUSED: $*" >&2; exit 2; }

# One harness at a time: log/sitl/.lock is held (flock, fd 9, non-blocking) until this script exits; the processes
# it starts do not inherit it. This closes the race of two starts that both pass the read-only prechecks below.
mkdir -p "${WS_DIR}/log/sitl"
exec 9>>"${WS_DIR}/log/sitl/.lock"
flock -n 9 || refuse "another sitl_hover.sh holds log/sitl/.lock (one SITL run at a time)"
[[ -f "${DDS_PROFILE}" ]] || refuse "missing ${DDS_PROFILE}"

###############################################################################
# Prechecks (read-only; also in --dry-run)
###############################################################################
# Never while a build or test runs (CLAUDE.local.md machine limits), and one SITL at a time. sitl_guard.py
# matches on each process argv (argv[0], or the script of an interpreter), not on a substring of a command
# line, and excludes this script's own ancestor chain (the shell, make, ...).
guard_rc=0
guard_out="$("${PYTHON}" "${WS_DIR}/tools/uavnav/sitl_guard.py")" || guard_rc=$?
case "${guard_rc}" in
  0) ;;
  3)
    if colcon_procs="$(grep '^colcon ' <<<"${guard_out}")"; then
      refuse "a colcon build or test is running (pid argv):"$'\n'"$(cut -d' ' -f2- <<<"${colcon_procs}")"
    fi
    refuse "another SITL process is running (stop it first; this script never kills it) (pid argv):"$'\n'"$(cut -d' ' -f2- <<<"${guard_out}")"
    ;;
  *) refuse "tools/uavnav/sitl_guard.py failed (exit ${guard_rc})" ;;
esac
if command -v ss >/dev/null 2>&1 && ss -Hlun "sport = :${XRCE_PORT}" | grep -q .; then
  refuse "UDP port ${XRCE_PORT} (MicroXRCEAgent) is already in use: $(ss -Hlunp "sport = :${XRCE_PORT}")"
fi
command -v MicroXRCEAgent >/dev/null 2>&1 || refuse "MicroXRCEAgent is not on PATH"
env --default-signal=INT true 2>/dev/null || refuse "env --default-signal is unsupported (coreutils >= 9.0 needed)"
[[ -f /opt/ros/jazzy/setup.bash ]] || refuse "ROS 2 Jazzy is not installed (/opt/ros/jazzy/setup.bash)"
[[ -f "${WS_DIR}/install/setup.bash" ]] || refuse "install/setup.bash is missing: make build PKGS=\"uav_simulation uav_description px4_msgs\""
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "${WS_DIR}/install/setup.bash"
set -u
command -v ros2 >/dev/null 2>&1 || refuse "ros2 is not on PATH after sourcing ROS"
SIM_SHARE="$(ros2 pkg prefix --share uav_simulation 2>/dev/null)" || refuse "uav_simulation is not installed: make build PKGS=uav_simulation"
CONTROL_BRIDGE_YAML="${SIM_SHARE}/bridge/px4_mid360_control_bridge.yaml"
[[ -f "${CONTROL_BRIDGE_YAML}" ]] || refuse "missing ${CONTROL_BRIDGE_YAML}: make build PKGS=uav_simulation"
[[ "${SIM_SHARE}" == "${WS_DIR}/install/"* ]] || refuse "uav_simulation resolves outside this workspace: ${SIM_SHARE}"
SIM_PREFIX="$(ros2 pkg prefix uav_simulation)"
[[ -x "${SIM_PREFIX}/lib/uav_simulation/gz_lidar_bridge" ]] || refuse "gz_lidar_bridge is not built: make build PKGS=uav_simulation"
BRIDGE_PREFIX="$(ros2 pkg prefix ros_gz_bridge 2>/dev/null)" || refuse "ros_gz_bridge is not installed (ros-jazzy-ros-gz-bridge)"
[[ -x "${BRIDGE_PREFIX}/lib/ros_gz_bridge/parameter_bridge" ]] || refuse "ros_gz_bridge parameter_bridge is missing"
"${PYTHON}" -c "import rclpy, numpy, px4_msgs.msg, rosgraph_msgs.msg, sensor_msgs.msg" 2>/dev/null \
  || refuse "${PYTHON} cannot import rclpy, numpy and px4_msgs (make build PKGS=px4_msgs)"
if [[ "${MODE}" == hover ]]; then
  # The estimation stack: make sitl-build.
  for pkg in uavnav_bringup uavnav_lio uavnav_px4_bridge; do
    prefix="$(ros2 pkg prefix "${pkg}" 2>/dev/null)" || refuse "${pkg} is not installed: make sitl-build"
    [[ "${prefix}" == "${WS_DIR}/install/"* ]] || refuse "${pkg} resolves outside this workspace: ${prefix}"
  done
  [[ -f "$(ros2 pkg prefix --share uavnav_bringup)/launch/s1_estimation.launch.py" ]] \
    || refuse "uavnav_bringup has no s1_estimation.launch.py: make sitl-build"
  [[ -x "$(ros2 pkg prefix uavnav_lio)/lib/uavnav_lio/lio_node" ]] || refuse "lio_node is not built: make sitl-build"
  [[ -x "$(ros2 pkg prefix uavnav_px4_bridge)/lib/uavnav_px4_bridge/px4_bridge_node" ]] \
    || refuse "px4_bridge_node is not built: make sitl-build"
  for f in "${LIO_CONFIG}" "${LIO_NODE_CONFIG}" "${BRIDGE_CONFIG}"; do [[ -f "${f}" ]] || refuse "missing ${f}"; done
  "${PYTHON}" -c "import uavnav_interfaces.msg, nav_msgs.msg" 2>/dev/null \
    || refuse "${PYTHON} cannot import uavnav_interfaces and nav_msgs (make sitl-build)"
fi
# Nothing else may publish the topics the run reads on this ROS_DOMAIN_ID (a bag, another sim, ...).
if ! graph="$(timeout 20 ros2 topic list --no-daemon --spin-time 2 2>/dev/null)"; then
  refuse "ros2 topic list failed (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
fi
if busy="$(grep -E '^/(clock|lidar/|fmu/|sim/|lio/|alignment|events)' <<<"${graph}")"; then
  refuse "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} already has SITL topics (stop their publisher first):"$'\n'"${busy}"
fi
export GZ_IP=127.0.0.1
# Another Gazebo on this transport (whatever its process looks like) shows up as a world clock topic.
if gz_topics="$(timeout 3 gz topic -l 2>/dev/null)"; then  # gz of the ROS env, its GZ_CONFIG_PATH kept
  if worlds="$(grep -E '^/world/[^/]+/clock$' <<<"${gz_topics}")"; then
    refuse "another Gazebo is running (GZ_IP=${GZ_IP}):"$'\n'"${worlds}"
  fi
else
  echo "sitl_hover: warning: gz topic -l did not answer within 3 s; relying on the process, port and ROS checks" >&2
fi
# PX4 binary, rootfs, Gazebo, world and model, the profile (O23 allow-list) and the spawn yaw.
SPAWN_ENV=()
if [[ -n "${SPAWN_YAW_RAD}" ]]; then SPAWN_ENV=(UAVNAV_SPAWN_YAW_RAD="${SPAWN_YAW_RAD}"); fi
# The failure injection is a harness parameter (never a profile one), enabled only for the outage run.
if [[ -n "${GNSS_OUTAGE_S}" ]]; then SPAWN_ENV+=(UAVNAV_SYS_FAILURE_EN=1); fi
echo "sitl_hover: run_px4_mid360.sh --check-only"
env UAVNAV_PX4_PROFILE="${PROFILE_FILE}" PYTHON="${PYTHON}" "${SPAWN_ENV[@]}" \
  tools/simulation/run_px4_mid360.sh --check-only | sed 's/^/  /' || refuse "run_px4_mid360.sh --check-only failed (see above)"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ "${MODE}" == probe ]]; then
  RUN_DIR="${WS_DIR}/log/sitl/${STAMP}-${PROFILE}-probe"
  TIMEOUT_S="${PROBE_TIMEOUT_S}"
else
  RUN_DIR="${WS_DIR}/log/sitl/${STAMP}-${PROFILE}-${PATTERN}${SPAWN_YAW_RAD:+-yaw${SPAWN_YAW_RAD}}${GNSS_OUTAGE_S:+-gnssout${GNSS_OUTAGE_S}}"
  TIMEOUT_S="${HOVER_TIMEOUT_S}"
fi
export ROS_LOG_DIR="${RUN_DIR}/ros_log"  # the ROS and launch logs stay in the run directory

# The Fast DDS profile of the ROS 2 processes (see the header); not for MicroXRCEAgent or PX4.
DDS_ENV=(FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}")
# name and command of each process group, in start order (the driver is started last, with its budget).
# shellcheck disable=SC2034  # read through the CMD_<name> nameref
{
  CMD_xrce_agent=(MicroXRCEAgent udp4 -p "${XRCE_PORT}")
  CMD_px4_gz=(env GZ_GUI=0 SESSION_DIR="${RUN_DIR}" UAVNAV_PX4_PROFILE="${PROFILE_FILE}" PYTHON="${PYTHON}"
              "${SPAWN_ENV[@]}" tools/simulation/run_px4_mid360.sh)
  CMD_control_bridge=(env "${DDS_ENV[@]}" ros2 run ros_gz_bridge parameter_bridge --ros-args
                      -p "config_file:=${CONTROL_BRIDGE_YAML}")
  CMD_lidar_bridge=(env "${DDS_ENV[@]}" ros2 run uav_simulation gz_lidar_bridge)
  CMD_estimation=(env "${DDS_ENV[@]}" ros2 launch uavnav_bringup s1_estimation.launch.py "lio_config:=${LIO_CONFIG}"
                  "lio_node_config:=${LIO_NODE_CONFIG}" "bridge_config:=${BRIDGE_CONFIG}" "events_dir:=${RUN_DIR}")
}
if [[ "${MODE}" == probe ]]; then
  GROUP_NAMES=(xrce_agent px4_gz control_bridge lidar_bridge)
  DRIVER_NAME=probe
else
  GROUP_NAMES=(xrce_agent px4_gz estimation)
  DRIVER_NAME=hover
fi
driver_cmd() {  # $1 = budget in seconds
  if [[ "${MODE}" == probe ]]; then
    DRIVER_CMD=(env "${DDS_ENV[@]}" "${PYTHON}" tools/uavnav/sitl_probe.py --run-dir "${RUN_DIR}" --profile "${PROFILE}"
                --hover-mode "${HOVER_MODE}" --budget-s "$1")
  else
    DRIVER_CMD=(env "${DDS_ENV[@]}" "${PYTHON}" tools/uavnav/sitl_hover.py --run-dir "${RUN_DIR}" --profile "${PROFILE}"
                --spawn-yaw-rad "${SPAWN_YAW_RAD:-0}" --pattern "${PATTERN}" --hover-mode "${HOVER_MODE}"
                --budget-s "$1" ${GNSS_OUTAGE_S:+--gnss-outage-s "${GNSS_OUTAGE_S}"})
  fi
}
if [[ "${MODE}" == probe ]]; then
  VERDICT_CMD=("${PYTHON}" tools/uavnav/sitl_checks.py --probe "${RUN_DIR}")
elif [[ "${PATTERN}" == altitude_sweep ]]; then
  VERDICT_CMD=("${PYTHON}" tools/uavnav/sitl_checks.py --sweep "${RUN_DIR}" --profile "${PROFILE}"
               --spawn-yaw-rad "${SPAWN_YAW_RAD:-0}")
else
  VERDICT_CMD=("${PYTHON}" tools/uavnav/sitl_checks.py --hover "${RUN_DIR}" --profile "${PROFILE}"
               --spawn-yaw-rad "${SPAWN_YAW_RAD:-0}")
fi

if [[ "${DRY_RUN}" == 1 ]]; then
  echo "sitl_hover: DRY RUN (prechecks passed; nothing is started, no directory is created)"
  echo "  run dir     : ${RUN_DIR}/"
  echo "  environment : GZ_IP=${GZ_IP} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ROS_LOG_DIR=${ROS_LOG_DIR} (ROS 2 Jazzy + install/setup.bash)"
  for name in "${GROUP_NAMES[@]}"; do
    declare -n cmd="CMD_${name}"
    printf '  setsid env --default-signal=INT %-14s: %s > %s/%s.log\n' "${name}" "${cmd[*]}" "${RUN_DIR}" "${name}"
    unset -n cmd
  done
  driver_cmd "<${TIMEOUT_S} - elapsed - ${PROBE_MARGIN_S}>"
  printf '  setsid env --default-signal=INT %-14s: %s > %s/%s.log\n' "${DRIVER_NAME}" "${DRIVER_CMD[*]}" "${RUN_DIR}" "${DRIVER_NAME}"
  echo "  bound       : ${TIMEOUT_S} s from the first start; teardown SIGINT -> ${GRACE_INT_S} s -> SIGTERM -> ${GRACE_TERM_S} s -> SIGKILL"
  echo "  verdict     : ${VERDICT_CMD[*]} -> ${RUN_DIR}/summary.json"
  exit 0
fi

###############################################################################
# Run
###############################################################################
mkdir -p "${RUN_DIR}"
log() { echo "sitl_hover: $*" | tee -a "${RUN_DIR}/harness.log"; }

STARTED_NAMES=() STARTED_PIDS=()
TORN_DOWN=0 TEARDOWN_OK=1

start_group() {  # $1 = name, rest = command; the child becomes leader of its own session and process group
  local name="$1"; shift
  printf '%s\n' "$*" >>"${RUN_DIR}/commands.txt"
  # A background job of a non-interactive shell starts with SIGINT ignored; env restores the default so
  # the SIGINT step of the teardown reaches the process (ROS nodes and PX4 shut down cleanly on it).
  setsid env --default-signal=INT "$@" >"${RUN_DIR}/${name}.log" 2>&1 </dev/null 9>&- &
  local pid=$!
  STARTED_NAMES+=("${name}") STARTED_PIDS+=("${pid}")
  # Until setsid has run, the child is still in this script's session: poll up to 2 s for sid == pid.
  local sid="" tries
  for (( tries=0; tries<20; tries++ )); do
    sid="$(ps -o sid= -p "${pid}" 2>/dev/null | tr -d ' ')" || true
    if [[ -z "${sid}" || "${sid}" == "${pid}" ]]; then break; fi  # gone (the main loop reports it) or leader
    sleep 0.1
  done
  if [[ -n "${sid}" && "${sid}" != "${pid}" ]]; then
    log "${name} (pid ${pid}) did not become a session leader within 2 s (sid ${sid}); killing it and stopping"
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pid}" 2>/dev/null || true
    exit 1
  fi
  log "started ${name}: pid=pgid=sid=${pid}: $*"
}

# A session with a live (non-zombie) process. Our direct children stay zombies until reaped by wait,
# and pgrep lists zombies, so this reads the process states.
group_alive() { ps -o stat= -s "$1" 2>/dev/null | awk '$1 !~ /^Z/ { live = 1 } END { exit !live }'; }

signal_group() {  # $1 = signal, $2 = session/group id: the process group, then any session member that left it
  kill "-$1" -- "-$2" 2>/dev/null || true
  local member
  for member in $(pgrep -s "$2" 2>/dev/null); do kill "-$1" "${member}" 2>/dev/null || true; done
}

wait_groups_gone() {  # $1 = seconds; returns 0 when no started session has a live process left
  local end=$((SECONDS + $1)) pid any
  while :; do
    any=0
    for pid in "${STARTED_PIDS[@]}"; do
      if group_alive "${pid}"; then any=1; fi
    done
    if [[ "${any}" == 0 ]]; then return 0; fi
    if (( SECONDS >= end )); then return 1; fi
    sleep 0.5
  done
}

teardown() {
  [[ "${TORN_DOWN}" == 1 ]] && return 0
  trap '' INT TERM  # a teardown is never cut short; it is bounded by the grace periods
  if [[ ${#STARTED_PIDS[@]} -gt 0 ]]; then
    local i sig grace pending pid
    for sig in INT TERM KILL; do
      case "${sig}" in INT) grace="${GRACE_INT_S}" ;; TERM) grace="${GRACE_TERM_S}" ;; *) grace=3 ;; esac
      pending=0
      for (( i=${#STARTED_PIDS[@]}-1; i>=0; i-- )); do  # reverse start order: probe first, Gazebo/PX4 last
        if group_alive "${STARTED_PIDS[i]}"; then
          pending=1
          log "teardown: SIG${sig} to ${STARTED_NAMES[i]} (group ${STARTED_PIDS[i]})"
          signal_group "${sig}" "${STARTED_PIDS[i]}"
        fi
      done
      if [[ "${pending}" == 0 ]] || wait_groups_gone "${grace}"; then break; fi
    done
    for pid in "${STARTED_PIDS[@]}"; do  # reap our direct children; never block on one that still runs
      if ! group_alive "${pid}"; then wait "${pid}" 2>/dev/null || true; fi
    done
    sleep 0.5  # orphaned zombies are reaped by init
    local leftover=""
    for (( i=0; i<${#STARTED_PIDS[@]}; i++ )); do
      if pgrep -s "${STARTED_PIDS[i]}" >/dev/null 2>&1; then
        leftover+="${STARTED_NAMES[i]}: $(ps -o pid=,pgid=,sid=,stat=,args= -s "${STARTED_PIDS[i]}" 2>/dev/null)"$'\n'
      fi
    done
    if [[ -n "${leftover}" ]]; then
      TEARDOWN_OK=0
      log "teardown INCOMPLETE; still running (pid pgid sid stat args):"$'\n'"${leftover}"
    else
      log "teardown complete: pgrep -s finds no process in the ${#STARTED_PIDS[@]} started sessions"
    fi
  fi
  TORN_DOWN=1
}

# shellcheck disable=SC2317  # invoked by the EXIT trap
on_exit() {
  local status=$?
  trap - EXIT INT TERM
  teardown
  if [[ "${TEARDOWN_OK}" == 0 && "${status}" == 0 ]]; then status=1; fi
  exit "${status}"
}
trap on_exit EXIT
trap 'log "interrupted (SIGINT)"; exit 130' INT
trap 'log "terminated (SIGTERM)"; exit 130' TERM

log "${MODE} run dir ${RUN_DIR}; profile ${PROFILE_FILE}; spawn yaw ${SPAWN_YAW_RAD:-0} rad; hover mode ${HOVER_MODE}; GNSS outage ${GNSS_OUTAGE_S:-none} s; bound ${TIMEOUT_S} s"
T0=${SECONDS}
DEADLINE=$((T0 + TIMEOUT_S))
for name in "${GROUP_NAMES[@]}"; do
  declare -n cmd="CMD_${name}"
  start_group "${name}" "${cmd[@]}"
  unset -n cmd
done
budget=$((DEADLINE - SECONDS - PROBE_MARGIN_S))
(( budget > 0 )) || { log "no time left for the ${DRIVER_NAME} driver"; exit 124; }
driver_cmd "${budget}"
start_group "${DRIVER_NAME}" "${DRIVER_CMD[@]}"
PROBE_PID="${STARTED_PIDS[-1]}"

status=0 probe_rc=""
while :; do
  if ! kill -0 "${PROBE_PID}" 2>/dev/null; then
    probe_rc=0; wait "${PROBE_PID}" || probe_rc=$?
    log "${DRIVER_NAME} exited with ${probe_rc} after $((SECONDS - T0)) s"
    break
  fi
  for (( i=0; i<${#STARTED_PIDS[@]}-1; i++ )); do
    if ! kill -0 "${STARTED_PIDS[i]}" 2>/dev/null; then
      log "${STARTED_NAMES[i]} exited before the ${DRIVER_NAME} driver finished (see ${RUN_DIR}/${STARTED_NAMES[i]}.log)"
      status=1
    fi
  done
  [[ "${status}" == 0 ]] || break
  if (( SECONDS >= DEADLINE )); then
    log "hard timeout: ${TIMEOUT_S} s reached"
    status=124
    break
  fi
  sleep 1
done
teardown
trap 'log "interrupted (SIGINT)"; exit 130' INT
trap 'log "terminated (SIGTERM)"; exit 130' TERM
log "evaluating ${RUN_DIR}"
verdict=0
"${VERDICT_CMD[@]}" | tee -a "${RUN_DIR}/harness.log" || verdict=$?
if [[ "${TEARDOWN_OK}" == 0 ]]; then exit 1; fi
if [[ "${status}" != 0 ]]; then exit "${status}"; fi
if [[ "${verdict}" != 0 ]]; then exit "${verdict}"; fi
if [[ "${probe_rc}" != 0 ]]; then
  log "the ${DRIVER_NAME} driver exited with ${probe_rc} (a failed phase): FAIL although the checks passed"
  exit 1
fi
exit 0
