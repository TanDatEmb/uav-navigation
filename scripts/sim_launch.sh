#!/usr/bin/env bash
# =============================================================================
# sim_launch.sh — uav-navigation Gazebo SITL launcher
#
# Starts a complete SITL stack:
#   BG  – MicroXRCE-DDS Agent        (PX4 ↔ ROS 2 bridge)
#   BG  – ros_gz_bridge                (LiDAR PointCloud2 from Gazebo → ROS 2)
#   BG  – obstacle_perception         (PointCloud2 → 2.5D grid → ObstacleDistance for PX4 CP)
#   BG  – obstacle_distance_visualizer (RViz markers from ObstacleDistance)
#   BG  – rosbag2 record               (optional, set SKIP_BAG=1 to disable)
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
#   SKIP_BAG    – set 1 to skip rosbag recording
#   GZ_GUI      – set 1 to open Gazebo GUI
#   ENABLE_OBSTACLE_VIZ – set 1 to run RViz obstacle marker visualizer
# =============================================================================
set -euo pipefail

# ── Path resolution ─────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PX4_DIR="${PX4_DIR:-${HOME}/Dev/Autopilot}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
XRCE_PORT="${XRCE_PORT:-8888}"
SKIP_BAG="${SKIP_BAG:-0}"
GZ_GUI="${GZ_GUI:-0}"
ENABLE_OBSTACLE_VIZ="${ENABLE_OBSTACLE_VIZ:-0}"
# Map input source for voxel_map node.
# Production: px4_full (per Topic Contract in docs/architecture.md).
# Debug only: lio_world (frame=camera_init, NOT for quality assessment).
MAP_INPUT_SOURCE="${MAP_INPUT_SOURCE:-px4_full}"
if [[ "${MAP_INPUT_SOURCE}" != "px4_full" && "${MAP_INPUT_SOURCE}" != "lio_world" ]]; then
    echo "ERROR: MAP_INPUT_SOURCE='${MAP_INPUT_SOURCE}' invalid. Use 'px4_full' or 'lio_world'." >&2
    exit 1
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

# ── Pre-launch stale node cleanup ─────────────────────────────────────────
# Previous sessions may leave node executables running even after sim_stop,
# which causes duplicate publishers and misleading QoS discovery warnings.
for stale_name in world_bridge_node cloud_preprocessor_node voxel_map_node \
          ned_transform_node fast_lio2_node voxmap_manager_node \
          obstacle_perception_node livox_mid360_processor_node; do
    pkill -9 -f "${stale_name}" 2>/dev/null || true
done

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
# Gazebo Harmonic gpu_lidar publishes PointCloud2 on:
#   /world/.../model/.../link/lidar_sensor_link/sensor/lidar/scan/points
GZ_LIDAR_POINTS="/world/obstacle_course/model/x500_lidar_360_0/link/lidar_sensor_link/sensor/lidar/scan/points"
GZ_WORLD_CLOCK="/world/obstacle_course/clock"
make_bg "gz-bridge" 10 << BGEOF
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:\${GZ_SIM_RESOURCE_PATH:-}"
ros2 run ros_gz_bridge parameter_bridge \
  "${GZ_LIDAR_POINTS}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "${GZ_WORLD_CLOCK}@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "${GZ_LIDAR_POINTS}:=/lidar_360/points" \
  -r "${GZ_WORLD_CLOCK}:=/clock"
BGEOF

# ── obstacle_distance visualizer node (debug-only) ────────────────────────
if [[ "${ENABLE_OBSTACLE_VIZ}" == "1" ]]; then
make_bg "obstacle-viz" 16 << BGEOF
ros2 run px4_navigation obstacle_distance_visualizer_node --ros-args -p use_sim_time:=true
BGEOF
else
    echo "  [skip] obstacle-viz disabled by default (set ENABLE_OBSTACLE_VIZ=1 to enable)"
fi

# ── FAST-LIO2 adapter (core contract topics) ──────────────────────────────
make_bg "fast-lio2" 13 << BGEOF
ros2 run px4_mapping cloud_preprocessor_node \
    --ros-args \
        -r __node:=cloud_preprocessor \
    --params-file "${WS_DIR}/src/px4_mapping/config/defaults.yaml" \
    -p use_sim_time:=true \
    -p input_cloud_topic:=/lidar_360/points \
    -p px4_odom_topic:=/fmu/out/vehicle_odometry \
    -p vehicle_imu_topic:=/fmu/out/vehicle_imu \
    -p use_imu_fusion:=false \
        -p output_cloud_topic:=/livox/l1/cloud \
        -p output_odom_topic:=/livox/l1/odometry
BGEOF

# ── world_bridge node (L1 camera_init → map_ned world cloud) ──────────────
make_bg "ned-transform" 14 << BGEOF
ros2 run px4_mapping world_bridge_node \
  --ros-args \
    -r __node:=world_bridge \
  --params-file "${WS_DIR}/src/px4_mapping/config/defaults.yaml" \
  -p use_sim_time:=true \
    -p input_cloud_topic:=/livox/l1/cloud \
    -p output_cloud_topic:=/livox/world/cloud \
    -p lio_odom_topic:=/livox/l1/odometry \
  -p px4_odom_topic:=/fmu/out/vehicle_odometry \
  -p publish_visual_odometry_to_px4:=true \
  -p visual_odom_topic:=/fmu/in/vehicle_visual_odometry
BGEOF

# ── voxel_map node (sparse global map in map_ned) ─────────────────────────
make_bg "voxmap-manager" 16 << BGEOF
ros2 run px4_mapping voxel_map_node \
  --ros-args \
    -r __node:=voxel_map \
  --params-file "${WS_DIR}/src/px4_mapping/config/defaults.yaml" \
  -p use_sim_time:=true \
    -p cloud_topic:=/livox/world/cloud \
    -p map_topic:=/livox/map/global \
    -p lio_odom_topic:=/livox/l1/odometry \
  -p input_source:=${MAP_INPUT_SOURCE}
BGEOF

# ── obstacle_perception node ───────────────────────────────────────────────
make_bg "livox-proc" 14 << BGEOF
ros2 run px4_navigation obstacle_perception_node \
  --ros-args \
    -r __node:=obstacle_perception \
  --params-file "${WS_DIR}/src/px4_navigation/config/livox_mid360_processor.yaml" \
  -p use_sim_time:=true \
  -p input_cloud_topic:=/lidar_360/points \
  -p vehicle_odom_topic:=/fmu/out/vehicle_odometry \
  -p obstacle_distance_topic:=/fmu/in/obstacle_distance \
    -p local_virtual_scan_topic:=/livox/perception/scan_1d \
  -p grid_markers_topic:=/livox/grid_2d5/markers \
  -p min_distance_cloud_topic:=/livox/grid_2d5/min_distance
BGEOF

# ── rosbag2 (optional) ────────────────────────────────────────────────────
if [[ "${SKIP_BAG}" != "1" ]]; then
make_bg "rosbag" 20 << BGEOF
mkdir -p "${LOG_DIR}/rosbag"
ros2 bag record --output "${LOG_DIR}/rosbag/flight_data" \
  --max-cache-size 40000000 \
    --qos-profile-overrides-path "${WS_DIR}/config/rosbag_qos_overrides.yaml" \
  --include-unpublished-topics \
  --topics \
  /lidar_360/points \
  /livox/grid_2d5/markers \
  /livox/grid_2d5/min_distance \
    /livox/perception/scan_1d \
    /livox/l1/cloud \
    /livox/l1/odometry \
    /livox/world/cloud \
    /livox/map/global \
    /local_virtual_scan \
    /livox_processed \
    /livox_processed_ned \
    /livox_map \
  /fmu/in/obstacle_distance \
  /fmu/in/vehicle_visual_odometry \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_status \
  /fmu/out/vehicle_status_v1 \
    /odometry
BGEOF
fi

# ── RViz2 (optional) ────────────────────────────────────────────────────────
RVIZ2_BIN="/opt/ros/${ROS_DISTRO}/bin/rviz2"
if [[ -x "${RVIZ2_BIN}" ]]; then
# Publish static TF frames so RViz can display the point cloud and markers properly.
make_bg "static-tf" 18 << BGEOF
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id lidar_sensor_link \
  --child-frame-id x500_lidar_360_0/lidar_sensor_link/lidar

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map_ned \
  --child-frame-id base_link
BGEOF

make_bg "rviz" 20 << BGEOF
"${RVIZ2_BIN}" -d "${WS_DIR}/assets/rviz/uav_navigation.rviz" 2>&1
BGEOF
else
    echo "  [skip] rviz2 not installed. Install: sudo apt install ros-${ROS_DISTRO}-rviz2"
fi

# ════════════════════════════════════════════════════════════════════════════
# PX4 SITL — this also spawns the Gazebo world + drone model
# (always launches Gazebo GUI)
# ════════════════════════════════════════════════════════════════════════════
echo ""
echo "Starting PX4 SITL..."

make_win "px4-sitl" 4 "140x35" << WINEOF
export PX4_SIMULATOR=gz
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
echo "    fast-lio2    : ${LOG_DIR}/bg_fast-lio2.log"
echo "    livox-proc   : ${LOG_DIR}/bg_livox-proc.log"
if [[ "${SKIP_BAG}" != "1" ]]; then
    echo "    rosbag       : ${LOG_DIR}/bg_rosbag.log → rosbag/flight_data/"
fi
echo ""
echo "Quick commands:"
echo "  tail -f ${LOG_DIR}/win_px4-sitl.log          # Watch PX4"
echo "  tail -f ${LOG_DIR}/bg_livox-proc.log        # Watch publisher"
echo "  ros2 topic echo /fmu/in/obstacle_distance      # Verify CP data"
echo "  gz topic -l                                    # List Gazebo topics"
echo ""
echo "  bash scripts/sim_stop.sh                       # Stop everything"
echo "============================================================"