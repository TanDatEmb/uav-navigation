#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"

echo "[build] Sourcing ROS 2 environment..."
if [ -f /opt/ros/jazzy/setup.bash ]; then
    # Source in a way that survives broken user-local argcomplete scripts.
    # shellcheck source=/dev/null
    if ! source /opt/ros/jazzy/setup.bash 2>/dev/null; then
        echo "[build] Warning: ROS setup script returned non-zero (argcomplete/shebang issue?); continuing..."
    fi
else
    echo "[build] Warning: /opt/ros/jazzy/setup.bash not found; assuming ROS 2 is already sourced"
fi

echo "[build] Verifying ROS build Python dependencies on ${ROS_PYTHON}..."
if ! "${ROS_PYTHON}" -c "import catkin_pkg, em, lark, importlib_resources" 2>/dev/null; then
    echo "[build] ERROR: ${ROS_PYTHON} is missing ROS build dependencies."
    echo "[build] Install with: sudo apt install python3-catkin-pkg python3-empy python3-lark python3-importlib-resources"
    exit 1
fi

cd "${WORKSPACE_DIR}"

echo "[build] Cleaning previous build artefacts..."
rm -rf build install log

echo "[build] Building uav-navigation workspace (safe: workers=1, jobs=1)..."
GZ_VERSION="${GZ_VERSION:-harmonic}" \
MAKEFLAGS=-j1 \
colcon build \
    --base-paths src \
    --parallel-workers 1 \
    --executor sequential \
    --event-handlers console_direct+ \
    --symlink-install \
    --packages-up-to px4_ros2_utils px4_navigation_common px4_mapping px4_navigation \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DPython3_EXECUTABLE="${ROS_PYTHON}" \
        -DPYTHON_EXECUTABLE="${ROS_PYTHON}" \
        -DPython3_FIND_VIRTUALENV=STANDARD

echo "[build] Sourcing install..."
if [ -f install/setup.bash ]; then
    # shellcheck source=/dev/null
    if ! source install/setup.bash 2>/dev/null; then
        echo "[build] Warning: install/setup.bash returned non-zero; build artifacts are still ready."
        echo "[build] You can source it manually: source install/setup.bash"
    fi
else
    echo "[build] Warning: install/setup.bash not found"
fi

echo "[build] Done."
