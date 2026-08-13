SHELL := /bin/bash

DATASET ?=
RATE ?= 1.0
FRONTIER_DEBUG ?= 0
PX4_DIR ?= $(HOME)/Dev/Autopilot
# Keep legacy runtime environments from re-enabling the old RViz sidecar.
NO_RVIZ_ENV = export ENABLE_RVIZ=0 RVIZ_ENABLE=0 DISABLE_RVIZ=1 NAVIGATION_NO_RVIZ=1;
ROS_ENV = $(NO_RVIZ_ENV) source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
BUILD_ENV = PARALLEL_WORKERS="$${PARALLEL_WORKERS:-1}" MAKE_JOBS="$${MAKE_JOBS:-1}" GZ_VERSION="$${GZ_VERSION:-}" COLCON_FLAGS="$${COLCON_FLAGS:-}"
FRONTIER_DEBUG_ARG = $(if $(filter 1 true yes,$(FRONTIER_DEBUG)),--frontier-debug,)

.PHONY: help build test replay dataset-check sim-check sim status stop clean

help:
	@echo "uav-navigation runtime commands"
	@echo "  make build                                 static ROS build; no runtime data required"
	@echo "  make test                                  unit/integration tests; not a runtime verdict"
	@echo "  make replay DATASET=<name> RATE=1.0       dataset replay alias; always launches RViz"
	@echo "  make dataset-check DATASET=<name> RATE=1.0 full dataset pipeline; PX4 not required"
	@echo "  make sim-check                             headless PX4/Gazebo + offboard acceptance"
	@echo "  make sim                                   interactive PX4/Gazebo/RViz session; no auto flight"
	@echo "  make status                                live state for the latest session"
	@echo "  make stop                                  stop all workspace-owned runtime session process groups"
	@echo "  make clean                                 remove build, logs, artifacts and caches"
	@echo "Artifacts: .artifacts/runtime/<workflow>-*/REPORT.md and report.json"
	@echo "PASS requires samples, freshness, validity, cleanup, and workflow-specific acceptance."

build:
	@$(ROS_ENV) $(BUILD_ENV) python3 tools/runtime/build.py build

test:
	@$(ROS_ENV) $(BUILD_ENV) python3 tools/runtime/build.py test
	@python3 -m unittest discover -s tools/tests -p 'test_*.py' -v
	@python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v

replay:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) python3 tools/runtime/runner.py dataset-check --dataset "$(DATASET)" --rate "$(RATE)" --rviz $(FRONTIER_DEBUG_ARG)

dataset-check:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) python3 tools/runtime/runner.py dataset-check --dataset "$(DATASET)" --rate "$(RATE)"

sim-check:
	@$(ROS_ENV) export PX4_DIR="$(PX4_DIR)"; python3 tools/runtime/runner.py sim-check

sim:
	@$(ROS_ENV) export PX4_DIR="$(PX4_DIR)"; python3 tools/runtime/runner.py sim

status:
	@$(ROS_ENV) python3 tools/runtime/runner.py status

stop:
	@$(ROS_ENV) python3 tools/runtime/runner.py stop

clean:
	@python3 tools/runtime/runner.py clean
