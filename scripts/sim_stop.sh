#!/usr/bin/env bash
# =============================================================================
# sim_stop.sh — Stop all uav-navigation SITL processes (fast, clean)
#
# Kills: xterm windows, PX4 SITL, Gazebo, ros_gz_bridge, obstacle publisher,
#        MicroXRCEAgent, rosbag2 — all at once, no sequential sleep loops.
# =============================================================================
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${WS_DIR}/log/sim/latest"

echo "Stopping uav-navigation SITL processes..."

# ── 1. Kill xterm windows (tagged [uav-nav]) ────────────────────────────────
# This is the main fix: old script didn't kill xterm windows.
xterm_pids=$(pgrep -f "xterm.*uav-nav" 2>/dev/null || true)
if [[ -n "${xterm_pids}" ]]; then
    echo "  kill: xterm windows (PIDs: ${xterm_pids})"
    echo "${xterm_pids}" | xargs kill -9 2>/dev/null || true
fi

# ── 2. Kill all SITL processes by pattern — all at once ─────────────────────
# SIGINT first (graceful for rosbag flush), then SIGKILL after 1s.
pkill -INT -f "ros2 bag record" 2>/dev/null || true
pkill -INT -f "obstacle_distance_publisher" 2>/dev/null || true
pkill -INT -f "parameter_bridge" 2>/dev/null || true
pkill -INT -f "px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -INT -f "gz sim" 2>/dev/null || true
pkill -INT -f "MicroXRCEAgent" 2>/dev/null || true

# Brief wait for graceful shutdown
sleep 1

# Force kill survivors
pkill -KILL -f "ros2 bag record" 2>/dev/null || true
pkill -KILL -f "obstacle_distance_publisher" 2>/dev/null || true
pkill -KILL -f "parameter_bridge" 2>/dev/null || true
pkill -KILL -f "px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -KILL -f "gz sim" 2>/dev/null || true
pkill -KILL -f "rviz2" 2>/dev/null || true
pkill -KILL -f "MicroXRCEAgent" 2>/dev/null || true

# Also kill xterm windows if still alive
pkill -KILL -f "xterm.*uav-nav" 2>/dev/null || true

# ── 3. Free XRCE UDP port ──────────────────────────────────────────────────
if command -v fuser >/dev/null 2>&1; then
    fuser -k 8888/udp 2>/dev/null || true
fi

# ── 4. Verify clean (exclude this script/grep from count) ───────────────────
sleep 0.5
remaining=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|obstacle_distance_publisher|parameter_bridge|uav-nav" 2>/dev/null | grep -v "$$" | wc -l || echo 0)
remaining=$(echo "${remaining}" | tr -d '[:space:]')

echo ""
echo "=========================================="
if [[ "${remaining:-0}" -eq 0 ]]; then
    echo "  All processes terminated cleanly"
else
    echo "  WARNING: ${remaining} process(es) still running"
    echo "  Try: pkill -9 -f 'px4|gz sim|MicroXRCE'"
fi
echo "=========================================="