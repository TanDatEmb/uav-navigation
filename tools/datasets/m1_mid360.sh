#!/usr/bin/env bash
set -euo pipefail

action=${1:?action required}
dataset=${2:-swarm_lio2_mutual_avoidance_uav1}
run=${3:-swarm_lio2_mutual_avoidance_uav1_release_run_20260728}
root="data/external/mid360/${dataset}"
report="reports/m1_dataset/${run}"
bag="${root}/original/mutual_avoidance_uav1.bag"
converted="${root}/converted/mutual_avoidance_uav1"
file_id=1DVs5DGqGsBW-oNL9mj7bKIqIODFuQ2D7

case "$action" in
  download)
    mkdir -p "${root}/original" "${root}/converted"
    if [[ ! -f "$bag" ]]; then
      python3 -m venv .cache/m1-gdown
      .cache/m1-gdown/bin/pip install --quiet gdown
      .cache/m1-gdown/bin/gdown "$file_id" -O "$bag"
    fi
    echo "43f25ac10deb11f8eed4febe33574b7c7bbc67171b5a93268931e827ad40cf24  $bag" | sha256sum --check
    ;;
  sanitize)
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-up-to fast_lio_ros --build-base build-asan \
      --install-base install-asan --cmake-args -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
      -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
    echo "Build complete. Dataset sanitizer replay remains blocked by the synchronization defect documented in ${report}."
    exit 1
    ;;
  run)
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-up-to fast_lio_ros --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    echo "Use the checked production-node orchestration documented in docs/verification/m1_mid360_dataset_validation_report.md"
    ;;
  reference)
    echo "BLOCKED: ROS1 Noetic/container runtime is unavailable; see ${report}/comparison.md" >&2
    exit 1
    ;;
  compare)
    python3 tools/datasets/export_m1_run.py --bag "${report}/production_outputs_valid" --output "$report"
    ;;
  view)
    test -f "${report}/map_full.pcd"
    if command -v pcl_viewer >/dev/null; then pcl_viewer "${report}/map_full.pcd"; else xdg-open "${report}/map_xy.png"; fi
    ;;
  *) echo "unknown action: $action" >&2; exit 2 ;;
esac
