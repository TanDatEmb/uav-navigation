# =============================================================================
# GNUmakefile – colcon build for uav-navigation workspace
#
# Resource-constrained build variables (override on Raspberry Pi 5 / laptop):
#   make build-safe PARALLEL_WORKERS=1 MAKE_JOBS=1
#
# Targets:
#   build-safe      : full constrained build of all packages
#   build-core-safe : build only core packages + dependencies
#   build-test      : build with BUILD_TESTING=ON
#   test            : run unit tests
#   tidy            : remove old sessions and temp files
#   clean           : remove all build artefacts
# =============================================================================

PARALLEL_WORKERS ?= 1
MAKE_JOBS        ?= 1
GZ_VERSION       ?= harmonic
COLCON_FLAGS     ?= --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Core packages: minimum flight stack.
CORE_PACKAGES := px4_common px4_mapping px4_navigation

# All packages: full stack including bridge and visualization.
ALL_PACKAGES  := px4_common px4_mapping px4_navigation px4_ros_com px4_visualization

# ── Default target ─────────────────────────────────────────────────────────
.DEFAULT_GOAL := build-safe

# ─────────────────────────────────────────────────────────────────────────────
# Build with explicit resource limits (suitable for embedded targets or CI).
# Packages live under src/ following standard ROS 2 workspace layout.
# ─────────────────────────────────────────────────────────────────────────────
build-safe:
	@echo "Building with constrained resources: workers=$(PARALLEL_WORKERS), make_jobs=$(MAKE_JOBS), gz=$(GZ_VERSION)"
	@GZ_VERSION=$(GZ_VERSION) MAKEFLAGS=-j$(MAKE_JOBS) \
		colcon build \
		--base-paths src \
		--parallel-workers $(PARALLEL_WORKERS) \
		--executor sequential \
		--event-handlers console_direct+ \
		--symlink-install \
		--packages-up-to $(ALL_PACKAGES) \
		$(COLCON_FLAGS)

# ─────────────────────────────────────────────────────────────────────────────
# Build only core packages + their dependencies.
# ─────────────────────────────────────────────────────────────────────────────
build-core-safe:
	@echo "Building core packages with constrained resources: workers=$(PARALLEL_WORKERS), make_jobs=$(MAKE_JOBS)"
	@GZ_VERSION=$(GZ_VERSION) MAKEFLAGS=-j$(MAKE_JOBS) \
		colcon build \
		--base-paths src \
		--parallel-workers $(PARALLEL_WORKERS) \
		--executor sequential \
		--event-handlers console_direct+ \
		--symlink-install \
		--packages-up-to $(CORE_PACKAGES) \
		$(COLCON_FLAGS)

# ─────────────────────────────────────────────────────────────────────────────
# Build with testing enabled.
# ─────────────────────────────────────────────────────────────────────────────
build-test:
	@echo "Building with BUILD_TESTING=ON: workers=$(PARALLEL_WORKERS), make_jobs=$(MAKE_JOBS)"
	@GZ_VERSION=$(GZ_VERSION) MAKEFLAGS=-j$(MAKE_JOBS) \
		colcon build \
		--base-paths src \
		--parallel-workers $(PARALLEL_WORKERS) \
		--executor sequential \
		--event-handlers console_direct+ \
		--packages-up-to $(ALL_PACKAGES) \
		--cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON

# ─────────────────────────────────────────────────────────────────────────────
# Run unit tests (requires build-test first).
# ─────────────────────────────────────────────────────────────────────────────
test:
	@colcon test \
		--base-paths src \
		--packages-select $(CORE_PACKAGES) \
		--event-handlers console_direct+
	@colcon test-result --verbose

# ─────────────────────────────────────────────────────────────────────────────
# Tidy: remove temp files and old build logs.
#   make tidy             → remove temporary files
#   make tidy KEEP=5      → reserved for future session cleanup
# ─────────────────────────────────────────────────────────────────────────────
KEEP ?= 1
tidy:
	@find . -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
	@find . -name '*.pyc' -delete 2>/dev/null || true
	@find . -name '*~' -delete 2>/dev/null || true
	@echo "Tidied temporary files."

# ─────────────────────────────────────────────────────────────────────────────
# Remove ALL build artefacts (build + install + logs).
# ─────────────────────────────────────────────────────────────────────────────
clean:
	@rm -rf build install log
	@find . -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
	@find . -name '*.pyc' -delete 2>/dev/null || true
	@echo "All build artefacts removed."

.PHONY: build-safe build-core-safe build-test test tidy clean
