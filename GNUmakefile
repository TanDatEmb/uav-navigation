# =============================================================================
# GNUmakefile — uav-navigation colcon build + SITL targets
#
# Usage:
#   make              # build all packages
#   make sim          # start SITL with Gazebo GUI
#   make sim-stop     # stop all SITL processes
#   make test         # run unit tests
#   make clean        # remove build artefacts
# =============================================================================

PARALLEL_WORKERS ?= 2
MAKE_JOBS        ?= 2
COLCON_FLAGS    ?= --cmake-args -DCMAKE_BUILD_TYPE=Release

ALL_PACKAGES := px4_common px4_mapping px4_navigation px4_ros_com

# ── Default: build all ──────────────────────────────────────────────────────
.DEFAULT_GOAL := build

# ─────────────────────────────────────────────────────────────────────────────
# Build
# ─────────────────────────────────────────────────────────────────────────────
build:
	@MAKEFLAGS=-j$(MAKE_JOBS) \
		colcon build \
		--parallel-workers $(PARALLEL_WORKERS) \
		--event-handlers console_direct+ \
		--symlink-install \
		--packages-up-to $(ALL_PACKAGES) \
		$(COLCON_FLAGS)

build-nav:
	@MAKEFLAGS=-j$(MAKE_JOBS) \
		colcon build \
		--parallel-workers $(PARALLEL_WORKERS) \
		--event-handlers console_direct+ \
		--symlink-install \
		--packages-select px4_navigation \
		$(COLCON_FLAGS)

# ─────────────────────────────────────────────────────────────────────────────
# Test
# ─────────────────────────────────────────────────────────────────────────────
test:
	@colcon test --packages-select $(ALL_PACKAGES) --event-handlers console_direct+
	@colcon test-result --verbose

# ─────────────────────────────────────────────────────────────────────────────
# SITL simulation (always with Gazebo GUI)
#   Requires: PX4 built at ~/Dev/Autopilot (make px4_sitl_default)
#   Requires: MicroXRCEAgent on PATH
# ─────────────────────────────────────────────────────────────────────────────
sim:
	@GZ_GUI=1 bash scripts/sim_launch.sh

sim-stop:
	@bash scripts/sim_stop.sh

# ─────────────────────────────────────────────────────────────────────────────
# Clean
# ─────────────────────────────────────────────────────────────────────────────
clean:
	@rm -rf build install log
	@echo "All build artefacts removed."

.PHONY: build build-nav test sim sim-stop clean