#!/usr/bin/env bash
# =============================================================================
# apply_px4_extras.sh
#
# Apply uav-navigation's PX4 Autopilot extras onto a clean PX4 release/1.17
# checkout so that SITL with the Livox MID-360 model can run.
#
# Usage:
#   bash tools/apply_px4_extras.sh [PX4_DIR]
#
# Default PX4_DIR: ${HOME}/Dev/Autopilot
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTRAS_DIR="${SCRIPT_DIR}/vendor/px4_autopilot_extras"
PX4_DIR="${1:-${HOME}/Dev/Autopilot}"
GZ_COMMIT="606c099"

if [[ ! -d "${PX4_DIR}/.git" ]]; then
    echo "ERROR: ${PX4_DIR} is not a git repository"
    exit 1
fi

if [[ ! -d "${EXTRAS_DIR}" ]]; then
    echo "ERROR: extras directory not found: ${EXTRAS_DIR}"
    exit 1
fi

echo "Applying PX4 extras to ${PX4_DIR}..."

# 1. Airframe
echo "  [1/5] Copy airframe 4022_gz_x500_lidar_360"
cp "${EXTRAS_DIR}/airframes/4022_gz_x500_lidar_360" \
   "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"

# 2. Patch CMakeLists
echo "  [2/5] Patch airframes/CMakeLists.txt"
cd "${PX4_DIR}"
if grep -q "4022_gz_x500_lidar_360" "ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"; then
    echo "       (airframe already registered in CMakeLists.txt)"
else
    patch -p1 --forward -i "${EXTRAS_DIR}/airframes_CMakeLists.patch" || {
        echo "ERROR: failed to patch CMakeLists.txt"
        exit 1
    }
fi

# 3. Checkout submodule gz to the experimental commit FIRST.
#    This gives us the baseline obstacle_course world and the original
#    x500_lidar_360 model. We then overwrite them with the refactored
#    standalone-lidar versions in steps 4-5.
echo "  [3/5] Checkout Tools/simulation/gz to ${GZ_COMMIT}"
cd "${PX4_DIR}/Tools/simulation/gz"
if git rev-parse --verify "${GZ_COMMIT}" >/dev/null 2>&1; then
    git checkout "${GZ_COMMIT}"
else
    echo "  [submodule] WARNING: commit ${GZ_COMMIT} not found in local gz repo."
    echo "              You may need to fetch or manually checkout the experimental branch."
fi

# 4. Copy Gazebo models (overwrite the embedded-lidar version with the refactored one)
echo "  [4/5] Copy Gazebo models"
cd "${PX4_DIR}"
cp -r "${EXTRAS_DIR}/gz_models/lidar_mid360" \
   "${PX4_DIR}/Tools/simulation/gz/models/"
cp -r "${EXTRAS_DIR}/gz_models/map_tree" \
   "${PX4_DIR}/Tools/simulation/gz/models/"
cp -r "${EXTRAS_DIR}/gz_models/x500_lidar_360" \
   "${PX4_DIR}/Tools/simulation/gz/models/"

# 5. Copy Gazebo world (should already exist at 606c099; overwrite to be safe)
echo "  [5/5] Copy Gazebo world"
cp "${EXTRAS_DIR}/gz_worlds/obstacle_course.sdf" \
   "${PX4_DIR}/Tools/simulation/gz/worlds/"

cd "${PX4_DIR}"
echo ""
echo "Done. Verify with:"
echo "  cd ${PX4_DIR} && git status --short"
echo "Expected:"
echo "  A  ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_lidar_360"
echo "  M  ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
echo "   M Tools/simulation/gz"
