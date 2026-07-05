#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[run_sim] Sourcing ROS 2 environment..."
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
# shellcheck source=/dev/null
source "${WORKSPACE_DIR}/install/setup.bash" 2>/dev/null || true

echo "[run_sim] Launching SITL navigation stack (placeholder)."
# Replace with actual launch file when ready.
# ros2 launch uav_navigation navigation_stack.launch.py use_sim_time:=true

echo "[run_sim] Done (placeholder)."
