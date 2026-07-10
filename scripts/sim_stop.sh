#!/usr/bin/env bash
# =============================================================================
# sim_stop.sh — Stop all uav-navigation SITL processes
#
# Strategy:
#   1. Collect PIDs recorded in every session directory under log/sim/.
#   2. Recursively kill each recorded PID and all of its descendants.
#   3. Kill well-known orphan processes by exact process name / command line,
#      using patterns that do not match a normal shell running this script.
#   4. SIGTERM first, then SIGKILL for survivors.
# =============================================================================
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${WS_DIR}/log/sim"
MY_PID="$$"

echo "Stopping uav-navigation SITL processes..."

# ── Recursive kill helper ───────────────────────────────────────────────────
# Kill children first so the parent does not respawn them before we get there.
kill_tree() {
    local pid="$1"
    local sig="$2"
    [[ -z "${pid}" ]] && return 0
    [[ "${pid}" == "${MY_PID}" ]] && return 0
    for child in $(pgrep -P "${pid}" 2>/dev/null || true); do
        kill_tree "${child}" "${sig}"
    done
    kill -"${sig}" "${pid}" 2>/dev/null || true
}

# ── 1. Collect PIDs from all session directories ────────────────────────────
PIDS=()
if [[ -d "${LOG_DIR}" ]]; then
    for pidfile in "${LOG_DIR}"/*/bg_*.pid "${LOG_DIR}"/*/win_*.pid; do
        [[ -f "${pidfile}" ]] || continue
        pid=$(cat "${pidfile}" 2>/dev/null | tr -d '[:space:]')
        [[ -n "${pid}" ]] && PIDS+=("${pid}")
    done
fi

# ── 2. Add well-known orphan processes ───────────────────────────────────────
# Exact process-name matches (comm field). These never match a shell running
# this script.
for name in rviz2 px4 MicroXRCEAgent parameter_bridge obstacle_distance_publisher_node obstacle_distance_visualizer_node obstacle_perception static_transform_publisher lidar_odometry localization_bridge global_mapper xterm; do
    for pid in $(pgrep -x "${name}" 2>/dev/null || true); do
        [[ "${pid}" == "${MY_PID}" ]] && continue
        [[ -n "${pid}" ]] && PIDS+=("${pid}")
    done
done

# ros2 bag record runs under a Python interpreter, so we match its cmdline.
# The command line of this shell does not contain "ros2 bag record".
for pid in $(pgrep -f "ros2 bag record" 2>/dev/null || true); do
    [[ "${pid}" == "${MY_PID}" ]] && continue
    [[ -n "${pid}" ]] && PIDS+=("${pid}")
done

# Gazebo Harmonic server / GUI client. The gz binary is wrapped by Ruby,
# so its process name is "ruby" and its cmdline starts with "gz sim".
for pid in $(pgrep -f "^gz sim" 2>/dev/null || true); do
    [[ "${pid}" == "${MY_PID}" ]] && continue
    [[ -n "${pid}" ]] && PIDS+=("${pid}")
done

# ── 3. Deduplicate and kill ─────────────────────────────────────────────────
if [[ ${#PIDS[@]} -gt 0 ]]; then
    UNIQUE_PIDS=$(printf '%s\n' "${PIDS[@]}" | sort -u | grep -v '^$')
    if [[ -z "${UNIQUE_PIDS}" ]]; then
        echo "  No processes found."
    else
        pid_count=$(echo "${UNIQUE_PIDS}" | wc -l | tr -d '[:space:]')
        echo "  Killing ${pid_count} recorded/orphan process tree(s)"
        while IFS= read -r pid; do
            kill_tree "${pid}" "TERM"
        done <<< "${UNIQUE_PIDS}"

        sleep 1

        ALIVE=()
        while IFS= read -r pid; do
            [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && ALIVE+=("${pid}")
        done <<< "${UNIQUE_PIDS}"

        if [[ ${#ALIVE[@]} -gt 0 ]]; then
            echo "  SIGKILL survivors: ${ALIVE[*]}"
            for pid in "${ALIVE[@]}"; do
                kill_tree "${pid}" "KILL"
            done
        fi
    fi
else
    echo "  No processes found."
fi

# ── 4. Free XRCE UDP port ───────────────────────────────────────────────────
if command -v fuser >/dev/null 2>&1; then
    fuser -k 8888/udp 2>/dev/null || true
fi

# ── 5. Verify clean ─────────────────────────────────────────────────────────
sleep 0.5
remaining=0
for name in rviz2 px4 MicroXRCEAgent parameter_bridge obstacle_distance_publisher_node obstacle_distance_visualizer_node obstacle_perception static_transform_publisher lidar_odometry localization_bridge global_mapper xterm; do
    count=$(pgrep -x "${name}" 2>/dev/null | wc -l)
    remaining=$((remaining + count))
done
remaining=$((remaining + $(pgrep -f "^gz sim" 2>/dev/null | wc -l)))
remaining=$((remaining + $(pgrep -f "ros2 bag record" 2>/dev/null | wc -l)))

echo ""
echo "=========================================="
if [[ "${remaining}" -eq 0 ]]; then
    echo "  All processes terminated cleanly"
else
    echo "  WARNING: ${remaining} process(es) still running"
fi
echo "=========================================="
