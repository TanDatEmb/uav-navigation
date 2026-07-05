#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[build-core] Sourcing ROS 2 environment..."
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

cd "${WORKSPACE_DIR}"

echo "[build-core] Cleaning previous build artefacts..."
rm -rf build install log

echo "[build-core] Building core packages (safe: workers=1, jobs=1)..."
GZ_VERSION="${GZ_VERSION:-harmonic}" \
MAKEFLAGS=-j1 \
colcon build \
    --parallel-workers 1 \
    --executor sequential \
    --event-handlers console_direct+ \
    --symlink-install \
    --packages-up-to px4_common px4_mapping px4_navigation \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[build-core] Sourcing install..."
# shellcheck source=/dev/null
source install/setup.bash

echo "[build-core] Done."
