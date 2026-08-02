SHELL := /bin/bash
DATASET ?=
RATE ?= 1.0
REPLAY_TIMEOUT ?= 900
REPLAY_READINESS_TIMEOUT ?= 30
REPLAY_DRAIN_TIMEOUT ?= 120
REPEAT ?= 1
MAX_LIDAR ?= 0
DRY_RUN ?= 0
ENABLE_RVIZ ?= 1
MODE ?= release
PACKAGES ?=
PARALLEL_WORKERS ?= 1
MAKE_JOBS ?= 1
GZ_VERSION ?=
COLCON_FLAGS ?=
DATASET_TOOL := python3 tools/data.py
ROS_ENV := source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
BUILD_TOOL := PARALLEL_WORKERS="$(PARALLEL_WORKERS)" MAKE_JOBS="$(MAKE_JOBS)" GZ_VERSION="$(GZ_VERSION)" COLCON_FLAGS="$(COLCON_FLAGS)" python3 tools/runtime/build.py --mode "$(MODE)"

.PHONY: help build build-safe test test-tools check vendor-check deps-px4-sync deps-px4-verify clean clean-artifacts data-list data-fetch data-check data-smoke data-run data-replay data-replay-stop replay-stop data-cleanup data-view data-report data-test runtime-repro px4-ingress-build px4-ingress-test px4-ingress-check px4-ingress-sitl px4-ingress-smoke

help:
	@$(DATASET_TOOL) --help

build:
	@$(BUILD_TOOL) build $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

build-safe:
	@BUILD_BASE="$(CURDIR)/build-safe" INSTALL_BASE="$(CURDIR)/install-safe" LOG_BASE="$(CURDIR)/log-safe" \
		PARALLEL_WORKERS="$(PARALLEL_WORKERS)" MAKE_JOBS="$(MAKE_JOBS)" \
		GZ_VERSION="$(GZ_VERSION)" COLCON_FLAGS="$(COLCON_FLAGS)" \
		$(MAKE) build

test:
	@$(BUILD_TOOL) test $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)
	@$(MAKE) test-tools

test-tools:
	@python3 -m unittest discover -s tools/tests -p 'test_*.py' -v
	@python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v
	@python3 -m unittest discover -s tools/simulation/tests -p 'test_*.py' -v

check:
	@$(BUILD_TOOL) check

vendor-check:
	@python3 tools/vendor_check.py

deps-px4-sync:
	@python3 tools/runtime/px4_deps.py sync

deps-px4-verify:
	@python3 tools/runtime/px4_deps.py verify

clean:
	@$(DATASET_TOOL) clean

clean-artifacts:
	@$(DATASET_TOOL) clean-artifacts

data-list:
	@$(DATASET_TOOL) list

data-fetch:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) fetch --dataset "$(DATASET)"

data-check:
	@$(DATASET_TOOL) check $(if $(strip $(DATASET)),--dataset "$(DATASET)",)

data-smoke:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) smoke --dataset "$(DATASET)"

data-run:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) run --dataset "$(DATASET)"

data-replay:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@trap 'python3 tools/runtime/cleanup_replay.py >/dev/null 2>&1 || true' EXIT; \
		$(ROS_ENV) $(DATASET_TOOL) replay --dataset "$(DATASET)" --rate "$(RATE)" \
		--replay-timeout "$(REPLAY_TIMEOUT)" \
		--readiness-timeout "$(REPLAY_READINESS_TIMEOUT)" \
		--drain-timeout "$(REPLAY_DRAIN_TIMEOUT)" \
		$(if $(filter 1 true yes,$(ENABLE_RVIZ)),--enable-rviz,)

data-replay-stop:
	@python3 tools/runtime/cleanup_replay.py $(if $(filter 1 true yes,$(DRY_RUN)),--dry-run,)

replay-stop: data-replay-stop

data-cleanup:
	@python3 tools/runtime/cleanup_replay.py $(if $(filter 1 true yes,$(DRY_RUN)),--dry-run,)

data-view:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) view --dataset "$(DATASET)"

data-report:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) report --dataset "$(DATASET)"

data-test:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) test --dataset "$(DATASET)"

runtime-repro:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) python3 tools/runtime/repro.py --dataset "$(DATASET)" \
		--repeat "$(REPEAT)" --max-lidar "$(MAX_LIDAR)" --mode "$(MODE)"

px4-ingress-build:
	@$(ROS_ENV) python3 tools/runtime/px4_ingress.py build

px4-ingress-test:
	@$(ROS_ENV) python3 tools/runtime/px4_ingress.py test

px4-ingress-check:
	@$(ROS_ENV) python3 tools/runtime/px4_ingress.py check

px4-ingress-sitl:
	@$(ROS_ENV) python3 tools/runtime/px4_ingress.py sitl

px4-ingress-smoke:
	@$(ROS_ENV) python3 tools/runtime/px4_ingress.py smoke

# BEGIN PX4 MID360 SIMULATION WORKFLOW
# PX4 MID-360 simulation workflow
PX4_DIR ?= $(HOME)/Dev/Autopilot
GZ_GUI ?= 1
SESSION_ROOT ?= $(CURDIR)/.artifacts/simulation
KEEP_SESSIONS ?= 10
OBSERVER_SAMPLE_HZ ?= 2
POINTCLOUD_SAMPLE_EVERY ?= 10
AUTO_SNAPSHOT ?= 1
PUBLISH_LOCAL_MAP ?=

.PHONY: sim-px4-mid360 sim-px4-mid360-headless sim-px4-mid360-check sim-px4-mid360-stop sim-px4-mid360-report sim-px4-mid360-clean sim-px4-mid360-test sim-px4-mid360-latest sim-px4-mid360-reset

sim-px4-mid360:
	@PX4_DIR="$(PX4_DIR)" GZ_GUI="$(GZ_GUI)" SESSION_ROOT="$(SESSION_ROOT)" \
		OBSERVER_SAMPLE_HZ="$(OBSERVER_SAMPLE_HZ)" POINTCLOUD_SAMPLE_EVERY="$(POINTCLOUD_SAMPLE_EVERY)" \
		AUTO_SNAPSHOT="$(AUTO_SNAPSHOT)" ENABLE_RVIZ="$(ENABLE_RVIZ)" PUBLISH_LOCAL_MAP="$(PUBLISH_LOCAL_MAP)" \
		bash tools/simulation/start_px4_mid360_session.sh

sim-px4-mid360-headless:
	@PX4_DIR="$(PX4_DIR)" GZ_GUI=0 SESSION_ROOT="$(SESSION_ROOT)" ENABLE_RVIZ=0 \
		OBSERVER_SAMPLE_HZ="$(OBSERVER_SAMPLE_HZ)" POINTCLOUD_SAMPLE_EVERY="$(POINTCLOUD_SAMPLE_EVERY)" \
		AUTO_SNAPSHOT="$(AUTO_SNAPSHOT)" PUBLISH_LOCAL_MAP="$(PUBLISH_LOCAL_MAP)" \
		bash tools/simulation/start_px4_mid360_session.sh

sim-px4-mid360-check:
	@$(ROS_ENV) bash tools/simulation/verify_px4_mid360.sh
	@$(ROS_ENV) timeout 8 ros2 topic hz /lidar/imu || true
	@$(ROS_ENV) timeout 8 ros2 topic hz /lidar/points || true
	@$(ROS_ENV) timeout 8 ros2 topic hz /lio/odometry_corrected || true
	@$(ROS_ENV) timeout 8 ros2 topic hz /lio/registered_points || true
	@$(ROS_ENV) ros2 topic echo /lio/diagnostics --once || true

sim-px4-mid360-stop:
	@SESSION_ROOT="$(SESSION_ROOT)" bash tools/simulation/stop_px4_mid360_session.sh

sim-px4-mid360-report:
	@python3 tools/simulation/report_generator.py \
		--session "$(SESSION_ROOT)/latest"
	@cat "$(SESSION_ROOT)/latest/REPORT.md"

sim-px4-mid360-clean:
	@python3 tools/simulation/session_manager.py clean --root "$(SESSION_ROOT)" --keep "$(KEEP_SESSIONS)"

sim-px4-mid360-test:
	@$(ROS_ENV) python3 -m unittest discover -s tools/simulation/tests -p 'test_*.py' -v

sim-px4-mid360-reset:
	@SESSION_ROOT="$(SESSION_ROOT)" bash tools/simulation/stop_px4_mid360_session.sh || true
	@$(MAKE) sim-px4-mid360

sim-px4-mid360-latest:
	@readlink -f "$(SESSION_ROOT)/latest"
# END PX4 MID360 SIMULATION WORKFLOW
