#!/usr/bin/env bash
# One-time (idempotent) host setup for a fresh clone.
#
# The workspace runs on the system Python (/usr/bin/python3, 3.12) because ROS 2
# Jazzy is installed for it; no virtualenv is used. This script checks the host,
# initialises submodules and installs missing system dependencies.
#
#   tools/setup.sh            check, then install missing dependencies (uses sudo)
#   tools/setup.sh --check    check only; change nothing, exit 1 if anything is missing
set -uo pipefail

CHECK_ONLY=0
case "${1:-}" in
  "") ;;
  --check) CHECK_ONLY=1 ;;
  -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
  *) echo "usage: $0 [--check]" >&2; exit 64 ;;
esac

cd "$(dirname "$0")/.."
ROS_SETUP=/opt/ros/jazzy/setup.bash
PROBLEMS=0
problem() { echo "MISSING: $*" >&2; PROBLEMS=$((PROBLEMS + 1)); }
ok() { echo "ok:      $*"; }

# 1. Host platform and interpreter -------------------------------------------
if [ -n "${VIRTUAL_ENV:-}" ]; then
  problem "a virtualenv is active ($VIRTUAL_ENV); run 'deactivate' (make refuses to run inside one)"
fi
# shellcheck disable=SC1091
. /etc/os-release 2>/dev/null || true
if [ "${VERSION_ID:-}" = "24.04" ]; then ok "Ubuntu 24.04"; else problem "Ubuntu 24.04 is required for ROS 2 Jazzy (found ${PRETTY_NAME:-unknown})"; fi
if [ -x /usr/bin/python3 ] && /usr/bin/python3 -c 'import sys; sys.exit(sys.version_info[:2] != (3, 12))'; then
  ok "/usr/bin/python3 is Python 3.12"
else
  problem "/usr/bin/python3 must be Python 3.12"
fi

# 2. ROS 2 Jazzy and build tools ---------------------------------------------
if [ -f "$ROS_SETUP" ]; then ok "ROS 2 Jazzy ($ROS_SETUP)"; else problem "ROS 2 Jazzy not found; follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"; fi
for tool in colcon rosdep git make; do
  if command -v "$tool" >/dev/null; then ok "$tool"; else problem "$tool (sudo apt install ${tool/colcon/python3-colcon-common-extensions})"; fi
done
if command -v gz >/dev/null; then ok "Gazebo (gz)"; else problem "Gazebo Harmonic (installed by ros-jazzy-ros-gz via rosdep)"; fi

# 3. Submodules ----------------------------------------------------------------
if [ -d .git ] && [ "$CHECK_ONLY" = 0 ]; then
  git submodule update --init --recursive || problem "git submodule update failed"
fi
for submodule in src/external/px4_msgs src/external/px4_ros2_interface_lib; do
  if [ -f "$submodule/package.xml" ] || [ -f "$submodule/px4_ros2_cpp/package.xml" ]; then ok "$submodule"; else problem "$submodule (run: git submodule update --init --recursive)"; fi
done

# 4. System dependencies (PyYAML for tools, and every package.xml dependency) ---
if /usr/bin/python3 -c 'import yaml' 2>/dev/null; then ok "python3-yaml"; else
  problem "python3-yaml"
  [ "$CHECK_ONLY" = 0 ] && sudo apt-get install -y python3-yaml && PROBLEMS=$((PROBLEMS - 1))
fi
if [ -f "$ROS_SETUP" ] && command -v rosdep >/dev/null; then
  # ROS setup scripts are not nounset-safe.
  set +u
  # shellcheck disable=SC1090
  . "$ROS_SETUP"
  set -u
  [ -d /etc/ros/rosdep/sources.list.d ] || { [ "$CHECK_ONLY" = 0 ] && sudo rosdep init; }
  [ "$CHECK_ONLY" = 0 ] && rosdep update >/dev/null
  if rosdep check --from-paths src --ignore-src --rosdistro jazzy >/dev/null 2>&1; then
    ok "rosdep: all package dependencies satisfied"
  elif [ "$CHECK_ONLY" = 1 ]; then
    problem "ROS package dependencies (run tools/setup.sh to install them)"
  else
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y -r || problem "rosdep install failed"
  fi
fi

# 5. Optional PX4 checkout (SITL workflows only) -------------------------------
PX4_DIR="${PX4_DIR:-$HOME/Dev/Autopilot}"
if [ -d "$PX4_DIR" ]; then ok "PX4 checkout $PX4_DIR"; else echo "note:    no PX4 checkout at $PX4_DIR (needed only for sim/run; set PX4_DIR)"; fi

if [ "$PROBLEMS" -gt 0 ]; then
  echo "setup: $PROBLEMS problem(s) found" >&2
  exit 1
fi
echo "setup complete. Next: make build && make test"
