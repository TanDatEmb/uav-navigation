#!/usr/bin/env bash
# =============================================================================
# sim_launch.sh — uav-navigation Gazebo SITL launcher
#
# Starts a complete SITL stack:
#   BG  – MicroXRCE-DDS Agent        (PX4 ↔ ROS 2 bridge)
#   BG  – ros_gz_bridge                (LiDAR PointCloud2 from Gazebo → ROS 2)
#   BG  – obstacle_perception         (PointCloud2 → 2.5D grid → ObstacleDistance for PX4 CP)
#   BG  – obstacle_distance_visualizer (RViz markers from ObstacleDistance)
#   BG  – rosbag2 record               (optional, set RECORD_BAG=1 to enable)
#   WIN – PX4 SITL console             (also launches Gazebo via gz_bridge)
#
# Usage:
#   bash scripts/sim_launch.sh           # headless SITL
#   GZ_GUI=1 bash scripts/sim_launch.sh  # SITL + Gazebo GUI
#
# Environment overrides:
#   PX4_DIR     – PX4 Autopilot repo (default: ~/Dev/Autopilot)
#   ROS_DISTRO  – ROS 2 distro      (default: jazzy)
#   XRCE_PORT   – uXRCE-DDS UDP port (default: 8888)
#   RECORD_BAG  – set 1 to enable high-bandwidth rosbag recording (default: 0)
#   GZ_GUI      – set 1 to open Gazebo GUI
#   ENABLE_OBSTACLE_VIZ – set 0 to disable RViz obstacle-bin visualization
#   ENABLE_EXTERNAL_ODOMETRY – set 1 to publish FAST-LIO odometry to PX4 (default: 0)
# =============================================================================
set -euo pipefail

# ── Path resolution ─────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PX4_DIR="${PX4_DIR:-${HOME}/Dev/Autopilot}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
XRCE_PORT="${XRCE_PORT:-8888}"
RECORD_BAG="${RECORD_BAG:-0}"
GZ_GUI="${GZ_GUI:-0}"
ENABLE_OBSTACLE_VIZ="${ENABLE_OBSTACLE_VIZ:-1}"
ENABLE_RVIZ="${ENABLE_RVIZ:-${GZ_GUI}}"
# Disabled by default while isolating unstable EV fusion from Collision Prevention.
ENABLE_EXTERNAL_ODOMETRY="${ENABLE_EXTERNAL_ODOMETRY:-0}"
SKIP_PX4_BUILD="${SKIP_PX4_BUILD:-0}"
# Map input source for global_mapper node.
# The default pipeline consumes FAST-LIO's registered cloud in lio_world.
# px4_full is an explicit fallback that consumes the raw sensor cloud and uses
# PX4 pose directly; it must never receive an already-registered LIO cloud.
MAP_INPUT_SOURCE="${MAP_INPUT_SOURCE:-lio_world}"
if [[ "${MAP_INPUT_SOURCE}" != "px4_full" && "${MAP_INPUT_SOURCE}" != "lio_world" ]]; then
    echo "ERROR: MAP_INPUT_SOURCE='${MAP_INPUT_SOURCE}' invalid. Use 'px4_full' or 'lio_world'." >&2
    exit 1
fi

if [[ "${MAP_INPUT_SOURCE}" == "lio_world" ]]; then
    GLOBAL_MAP_CLOUD_TOPIC="/lio/cloud_registered"
else
    GLOBAL_MAP_CLOUD_TOPIC="/lidar_360/points"
fi

# ── PX4 paths ──────────────────────────────────────────────────────────────
PX4_BUILD="${PX4_DIR}/build/px4_sitl_default"
PX4_BIN="${PX4_BUILD}/bin/px4"
PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"

# ── Validation ─────────────────────────────────────────────────────────────
if [[ ! -x "${PX4_BIN}" ]]; then
    echo "ERROR: PX4 SITL binary not found: ${PX4_BIN}" >&2
    echo "  Run: cd ${PX4_DIR} && make px4_sitl_default" >&2
    exit 1
fi
if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: gz (Gazebo) not on PATH. Install: sudo apt install gz-harmonic" >&2
    exit 1
fi
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    echo "ERROR: MicroXRCEAgent not in PATH." >&2
    echo "  Install: sudo apt install ros-${ROS_DISTRO}-micro-xrce-dds-agent" >&2
    exit 1
fi

# ── Headless detection ─────────────────────────────────────────────────────
HAS_DISPLAY=0
if [[ -n "${DISPLAY:-}" ]] && command -v xdpyinfo >/dev/null 2>&1; then
    if timeout 1 xdpyinfo >/dev/null 2>&1; then
        HAS_DISPLAY=1
    fi
fi

# ── Pre-launch cleanup ─────────────────────────────────────────────────────
# `make sim` is intentionally idempotent: a previous detached session must not
# leave PX4, Gazebo, XRCE, ROS nodes, or rosbag recorders competing with the new
# session. Use the workspace stop script so recorded process trees are cleaned
# before allocating ports and ROS node names again.
if pgrep -x px4 >/dev/null 2>&1 \
    || pgrep -x MicroXRCEAgent >/dev/null 2>&1 \
    || pgrep -f '^gz sim' >/dev/null 2>&1 \
    || pgrep -f 'ros2 bag record' >/dev/null 2>&1; then
    echo "[sim] Existing SITL session detected; stopping it first..."
    bash "${SCRIPT_DIR}/sim_stop.sh"
fi

# Clean up standalone ROS processes that may predate PID tracking.
for stale_name in global_mapper lio_px4_alignment fast_lio obstacle_perception ros_gz_bridge; do
    pkill -TERM -f "${stale_name}" 2>/dev/null || true
done

# ── PX4 airframe rebuild check ────────────────────────────────────────────
# If the selected airframe is not present in the PX4 build rootfs, we must
# rebuild PX4 so the new airframe is included in the ROMFS.
AIRFRAME_SCRIPT="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_lidar_360"
BUILT_AIRFRAME="${PX4_BUILD}/rootfs/etc/init.d-posix/airframes/4022_gz_x500_lidar_360"
if [[ "${SKIP_PX4_BUILD}" != "1" && -f "${AIRFRAME_SCRIPT}" && ! -f "${BUILT_AIRFRAME}" ]]; then
    echo ""
    echo "[sim] Airframe 4022_gz_x500_lidar_360 not found in PX4 build. Rebuilding PX4..."
    echo "      This happens when the extras have just been applied."
    echo "      Set SKIP_PX4_BUILD=1 to skip this check."
    (cd "${PX4_DIR}" && PX4_AIRFRAME=4022_gz_x500_lidar_360 make px4_sitl_default)
    if [[ ! -f "${BUILT_AIRFRAME}" ]]; then
        echo "ERROR: PX4 rebuild failed; airframe still missing: ${BUILT_AIRFRAME}" >&2
        exit 1
    fi
    echo "[sim] PX4 rebuild complete."
fi

# ── Session logging ────────────────────────────────────────────────────────
SESSION_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${WS_DIR}/log/sim/session_${SESSION_ID}"

# Clean old sessions (keep last 5)
SIM_LOG_BASE="${WS_DIR}/log/sim"
if [[ -d "${SIM_LOG_BASE}" ]]; then
    find "${SIM_LOG_BASE}" -maxdepth 1 -name 'session_*' -type d \
        | sort -r | tail -n +6 | xargs -r rm -rf 2>/dev/null || true
    rm -f "${SIM_LOG_BASE}/latest"
fi
mkdir -p "${LOG_DIR}"
ln -sfn "${LOG_DIR}" "${WS_DIR}/log/sim/latest"

# ── ROS source helper ────────────────────────────────────────────────────────
ROS_SOURCE="${LOG_DIR}/ros_source.sh"
cat > "${ROS_SOURCE}" << ROSEOF
#!/usr/bin/env bash
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
source "${WS_DIR}/install/setup.bash" 2>/dev/null || true
set -u
ROSEOF

# ── Background launcher ────────────────────────────────────────────────────
make_bg() {
    local name="$1"
    local delay_s="$2"
    local body; body="$(cat)"

    local ts="${LOG_DIR}/bg_${name}.sh"
    local tl="${LOG_DIR}/bg_${name}.log"

    cat > "${ts}" << BGEOF
#!/usr/bin/env bash
source "${ROS_SOURCE}"
echo "=== [${name}] started \$(date) ===" >> "${tl}"
[[ ${delay_s} -gt 0 ]] && sleep ${delay_s}
${body}
echo "=== [${name}] ended \$(date) ===" >> "${tl}"
BGEOF
    chmod +x "${ts}"
    bash "${ts}" >> "${tl}" 2>&1 &
    echo $! > "${LOG_DIR}/bg_${name}.pid"
    echo "  [bg] ${name}  (delay=${delay_s}s)  log: ${tl}"
}

# ── xterm launcher ─────────────────────────────────────────────────────────
make_win() {
    local name="$1"
    local delay_s="$2"
    local geometry="${3:-140x35}"
    local body; body="$(cat)"

    local ts="${LOG_DIR}/win_${name}.sh"
    local tl="${LOG_DIR}/win_${name}.log"

    cat > "${ts}" << WINEOF
#!/usr/bin/env bash
source "${ROS_SOURCE}"
echo "=== [${name}] started \$(date) ==="
[[ ${delay_s} -gt 0 ]] && echo "Waiting ${delay_s}s..." && sleep ${delay_s}
${body}
echo ""
echo "=== [${name}] exited \$(date). Press Enter to close. ==="
read -r _x
WINEOF
    chmod +x "${ts}"

    if [[ "${HAS_DISPLAY}" == "1" ]] && command -v xterm >/dev/null 2>&1; then
        xterm -T "[uav-nav] ${name}" -fa 'Monospace' -fs 10 \
              -geometry "${geometry}" \
              -e "bash '${ts}' 2>&1 | tee -a '${tl}'; echo ''; read -r _x" &
        echo $! > "${LOG_DIR}/win_${name}.pid"
        echo "  [win] ${name}  (delay=${delay_s}s)"
    else
        bash "${ts}" >> "${tl}" 2>&1 &
        echo $! > "${LOG_DIR}/win_${name}.pid"
        echo "  [bg-win] ${name}  (delay=${delay_s}s)  log: ${tl}"
    fi
}

# ════════════════════════════════════════════════════════════════════════════
# BACKGROUND PROCESSES
# ════════════════════════════════════════════════════════════════════════════
echo ""
echo "Starting background processes..."

# ── MicroXRCE-DDS Agent ───────────────────────────────────────────────────
make_bg "xrce-dds-agent" 0 << BGEOF
MicroXRCEAgent udp4 -p ${XRCE_PORT}
BGEOF

# ── ros_gz_bridge (LiDAR PointCloud2 + sim /clock → ROS 2) ───────────────
# Gazebo Harmonic MID-360 topics from the standalone lidar_mid360 model.
GZ_LIDAR_POINTS="/world/obstacle_course/model/x500_lidar_360_0/link/mid360_link/sensor/lidar/scan/points"
GZ_LIDAR_IMU="/world/obstacle_course/model/x500_lidar_360_0/link/mid360_link/sensor/mid360_imu/imu"
GZ_WORLD_CLOCK="/world/obstacle_course/clock"
make_bg "gz-bridge" 10 << BGEOF
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:\${GZ_SIM_RESOURCE_PATH:-}"
ros2 run ros_gz_bridge parameter_bridge \
  "${GZ_LIDAR_POINTS}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "${GZ_LIDAR_IMU}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  "${GZ_WORLD_CLOCK}@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "${GZ_LIDAR_POINTS}:=/lidar_360/points" \
  -r "${GZ_LIDAR_IMU}:=/imu/out" \
  -r "${GZ_WORLD_CLOCK}:=/clock"
BGEOF

# ── obstacle_distance visualizer node (debug-only) ────────────────────────
if [[ "${ENABLE_OBSTACLE_VIZ}" == "1" ]]; then
make_bg "obstacle-viz" 16 << BGEOF
ros2 run px4_navigation obstacle_distance_visualizer_node --ros-args -p use_sim_time:=true
BGEOF
else
    echo "  [skip] obstacle-viz disabled (ENABLE_OBSTACLE_VIZ=0)"
fi

# ── FAST-LIO2 node (15-DOF IESKF LiDAR-Inertial Odometry) ───────────────
make_bg "fast-lio" 13 << BGEOF
ros2 launch fast_lio fast_lio_sim.launch.py \
    use_sim_time:=true \
    lidar_topic:=/lidar_360/points \
    imu_topic:=/imu/out
BGEOF

# ── LIO-PX4 alignment bridge (LIO world → PX4 NED) ───────────────────────
# Keep this opt-in during the isolation test. FAST-LIO still runs and publishes
# diagnostics, but PX4 receives no external vehicle odometry when disabled.
if [[ "${ENABLE_EXTERNAL_ODOMETRY}" == "1" ]]; then
make_bg "lio-px4-alignment" 14 << BGEOF
ros2 launch px4_mapping lio_px4_alignment.launch.py \
    use_sim_time:=true
BGEOF
else
    echo "  [isolation] external odometry publication to PX4 disabled"
fi

# ── global_mapper node (sparse global map in map_ned) ─────────────────────
make_bg "global-mapper" 16 << BGEOF
ros2 run px4_mapping global_mapper \
  --ros-args \
    -r __node:=global_mapper \
  --params-file "${WS_DIR}/src/px4_mapping/config/defaults.yaml" \
  -p use_sim_time:=true \
    -p cloud_topic:=${GLOBAL_MAP_CLOUD_TOPIC} \
    -p map_topic:=/mapping/global \
    -p lio_odom_topic:=/lio/odometry \
    -p input_source:=${MAP_INPUT_SOURCE} \
    -p use_lio_buffer:=false
BGEOF

# ── obstacle_perception node ───────────────────────────────────────────────
make_bg "obstacle-perception" 14 << BGEOF
ros2 run px4_navigation obstacle_perception \
  --ros-args \
    -r __node:=obstacle_perception \
  --params-file "${WS_DIR}/src/px4_navigation/config/obstacle_perception.yaml" \
  -p use_sim_time:=true \
  -p input_cloud_topic:=/lidar_360/points \
  -p vehicle_odom_topic:=/fmu/out/vehicle_odometry \
  -p obstacle_distance_topic:=/fmu/in/obstacle_distance \
    -p local_virtual_scan_topic:=/perception/scan_1d \
  -p grid_markers_topic:=/visualization/grid_2d5/markers \
  -p min_distance_cloud_topic:=/visualization/grid_2d5/min_distance
BGEOF

# ── rosbag2 (optional) ────────────────────────────────────────────────────
if [[ "${RECORD_BAG}" == "1" ]]; then
make_bg "rosbag" 20 << BGEOF
mkdir -p "${LOG_DIR}/rosbag"
ros2 bag record --output "${LOG_DIR}/rosbag/flight_data" \
  --max-cache-size 40000000 \
    --qos-profile-overrides-path "${WS_DIR}/config/rosbag_qos_overrides.yaml" \
  --include-unpublished-topics \
  --topics \
  /lidar_360/points \
  /livox/lidar/pointcloud \
  /imu/out \
  /lio/odometry \
  /lio/path \
  /lio/cloud_registered \
  /fmu/in/vehicle_visual_odometry \
  /fmu/in/obstacle_distance \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_status \
  /fmu/out/vehicle_status_v1 \
  /fmu/out/vehicle_imu \
  /visualization/grid_2d5/markers \
  /visualization/grid_2d5/min_distance \
  /perception/scan_1d \
  /tf \
  /tf_static \
  /clock
BGEOF
fi

# ── RViz2 and visualization TFs (optional) ─────────────────────────────────
RVIZ2_BIN="/opt/ros/${ROS_DISTRO}/bin/rviz2"
if [[ "${ENABLE_RVIZ}" == "1" && -x "${RVIZ2_BIN}" ]]; then
# Gazebo publishes the raw cloud in mid360_lidar_frame. Connect it to the
# project sensor frame used by RViz and obstacle diagnostics.
make_bg "static-tf-lidar" 18 << BGEOF
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id lidar_sensor_link \
  --child-frame-id mid360_lidar_frame
BGEOF

# FAST-LIO publishes lio_world -> mid360_imu. This calibrated static transform
# closes the chain to the LiDAR/RViz sensor frame.
make_bg "static-tf-extrinsic" 18 << BGEOF
ros2 run tf2_ros static_transform_publisher \
  --x -0.011 --y -0.023 --z 0.044 --roll 0 --pitch 0 --yaw 0 \
  --frame-id mid360_imu \
  --child-frame-id lidar_sensor_link
BGEOF

make_bg "rviz" 20 << BGEOF
"${RVIZ2_BIN}" -d "${WS_DIR}/assets/rviz/uav_navigation.rviz" 2>&1
BGEOF
else
    echo "  [skip] RViz disabled or unavailable (ENABLE_RVIZ=${ENABLE_RVIZ})"
fi

# ════════════════════════════════════════════════════════════════════════════
# PX4 SITL — this also spawns the Gazebo world + drone model
# (always launches Gazebo GUI)
# ════════════════════════════════════════════════════════════════════════════
echo ""
echo "Starting PX4 SITL..."
echo "  Model: x500_lidar_360"
echo "  World: obstacle_course"
echo ""

make_win "px4-sitl" 4 "140x35" << WINEOF
# PX4 selects the airframe by the full autostart model name (including gz_).
# The selected airframe then maps it to the Gazebo asset x500_lidar_360.
export PX4_SYS_AUTOSTART=4022
export PX4_SIM_MODEL=gz_x500_lidar_360
export PX4_GZ_WORLD=obstacle_course
export PX4_GZ_MODEL_POSE="0,0,0,0,0,0"
export PX4_GZ_MODELS="${PX4_GZ_MODELS}"
export PX4_GZ_WORLDS="${PX4_GZ_WORLDS}"
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:\${GZ_SIM_RESOURCE_PATH:-}"
export GZ_IP=127.0.0.1

# Configure PX4 Gazebo GUI mode based on GZ_GUI.
if [[ "${GZ_GUI}" == "1" ]]; then
    export HEADLESS=""
    echo "Gazebo GUI enabled (HEADLESS cleared)"
else
    export HEADLESS=1
    echo "Gazebo GUI disabled (HEADLESS=1)"
fi

cd "${PX4_BUILD}/rootfs"
"${PX4_BIN}" -d
WINEOF

# ── Ensure Gazebo GUI client is running ────────────────────────────────────
# PX4's px4-rc.gzsim only spawns the GUI client when it starts a fresh world.
# If a world is already running (e.g. from a previous session), the GUI is not
# reopened. We therefore launch a dedicated GUI client here when requested.
if [[ "${GZ_GUI}" == "1" ]]; then
make_bg "gz-gui" 20 << BGEOF
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:\${GZ_SIM_RESOURCE_PATH:-}"
export GZ_IP=127.0.0.1

# Wait for the Gazebo server world to appear
WORLD_NAME=""
for i in \$(seq 1 30); do
    WORLD_NAME=\$(gz topic -l 2>/dev/null | grep -m 1 -e "^/world/.*/clock" | sed 's/\/world\///g; s/\/clock//g')
    [[ -n "\${WORLD_NAME}" ]] && break
    sleep 1
done

if [[ -z "\${WORLD_NAME}" ]]; then
    echo "WARNING: Gazebo world not detected, skipping GUI client launch" >&2
    exit 0
fi

# Avoid duplicate GUI clients. Gazebo Harmonic's process name is 'ruby',
# so we match the full command line instead of the process name.
if pgrep -a -f "^gz sim -g" >/dev/null 2>&1; then
    echo "Gazebo GUI client already running"
else
    echo "Starting Gazebo GUI client for world: \${WORLD_NAME}"
    gz sim -g &
fi
BGEOF
fi

# ── Summary ────────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  uav-navigation SITL launched"
echo "  Session      : ${SESSION_ID}"
echo "  Log dir      : ${LOG_DIR}/"
echo "  GUI          : ${GZ_GUI}"
echo "  Map mode     : ${MAP_INPUT_SOURCE}"
echo ""
echo "  Background logs:"
echo "    xrce-dds     : ${LOG_DIR}/bg_xrce-dds-agent.log"
echo "    gz-bridge    : ${LOG_DIR}/bg_gz-bridge.log"

echo "    global-map   : ${LOG_DIR}/bg_global-mapper.log"
echo "    obstacle     : ${LOG_DIR}/bg_obstacle-perception.log"
if [[ "${RECORD_BAG}" == "1" ]]; then
    echo "    rosbag       : ${LOG_DIR}/bg_rosbag.log → rosbag/flight_data/"
fi
echo ""
echo "Quick commands:"
echo "  tail -f ${LOG_DIR}/win_px4-sitl.log          # Watch PX4"
echo "  tail -f ${LOG_DIR}/bg_obstacle-perception.log # Watch obstacle perception"
echo "  ros2 topic echo /fmu/in/obstacle_distance      # Verify CP data"
echo "  gz topic -l                                    # List Gazebo topics"
echo ""
echo "  bash scripts/sim_stop.sh                       # Stop everything"
echo "============================================================"