SHELL := /bin/bash
PYTHON := /usr/bin/python3
# ROS Jazzy's rclpy/ament modules are installed for the system interpreter.
# Do not let an activated venv or PYTHONHOME silently select another runtime.
CANONICAL_PYTHON_ENV = env -u VIRTUAL_ENV -u PYTHONHOME

DATASET ?=
RATE ?= 1.0
DATASET_SHADOW_GOAL_M ?= 5.0
DATASET_ROS_DOMAIN_ID ?=
FRONTIER_DEBUG ?= 0
PX4_DIR ?= $(HOME)/Dev/Autopilot
BUILD_PX4_ROS2_EXAMPLES ?= 0
# Keep legacy runtime environments from re-enabling the old RViz sidecar.
NO_RVIZ_ENV = export ENABLE_RVIZ=0 RVIZ_ENABLE=0 DISABLE_RVIZ=1 NAVIGATION_NO_RVIZ=1;
ROS_ENV = $(NO_RVIZ_ENV) source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
ROS_GUI_ENV = export ENABLE_RVIZ=1 RVIZ_ENABLE=1 DISABLE_RVIZ=0 NAVIGATION_NO_RVIZ=0; source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
BUILD_ENV = PARALLEL_WORKERS="$${PARALLEL_WORKERS:-1}" MAKE_JOBS="$${MAKE_JOBS:-1}" GZ_VERSION="$${GZ_VERSION:-}" BUILD_PX4_ROS2_EXAMPLES="$${BUILD_PX4_ROS2_EXAMPLES:-$(BUILD_PX4_ROS2_EXAMPLES)}" COLCON_FLAGS="$${COLCON_FLAGS:-}"
FRONTIER_DEBUG_ARG = $(if $(filter 1 true yes,$(FRONTIER_DEBUG)),--frontier-debug,)
DATASET_SHADOW_GOAL_ARG = --shadow-planning-goal-distance-m $(DATASET_SHADOW_GOAL_M)
DATASET_ROS_DOMAIN_ARG = $(if $(strip $(DATASET_ROS_DOMAIN_ID)),--ros-domain-id $(DATASET_ROS_DOMAIN_ID),)
MAP_PROFILE ?=
MAP_SCENE ?= sanity_open
TEST_CASE ?= positive
MOTION_PRESET ?= nominal
MAP_SEED ?= 0
MANUAL_TAKEOFF ?= 0
SPEED_CAP_MPS ?=
MAP_PROFILE_ARG = $(if $(strip $(MAP_PROFILE)),--map-profile $(MAP_PROFILE),)
MAP_SCENE_ARG = $(if $(strip $(MAP_PROFILE)),,--map-scene $(MAP_SCENE))
TEST_CASE_ARG = --test-case $(TEST_CASE)
MOTION_PRESET_ARG = --motion-preset $(MOTION_PRESET)
MAP_SEED_ARG = --map-seed $(MAP_SEED)
MANUAL_TAKEOFF_ARG = $(if $(filter 1 true yes,$(MANUAL_TAKEOFF)),--manual-takeoff,)
SPEED_CAP_MPS_ARG = $(if $(strip $(SPEED_CAP_MPS)),--speed-cap-mps $(SPEED_CAP_MPS),)

.PHONY: help build test replay dataset-check sim-check external-mode-check external-mode-gui external-mode sim status stop clean

help:
	@echo "uav-navigation runtime commands"
	@echo "  make build                                 static ROS build; no runtime data required"
	@echo "  BUILD_PX4_ROS2_EXAMPLES=1 make build       include upstream PX4 ROS 2 examples (default: off)"
	@echo "  make test                                  unit/integration tests; not a runtime verdict"
	@echo "  make replay DATASET=<name> RATE=1.0       dataset replay alias; always launches RViz"
	@echo "  make dataset-check DATASET=<name> RATE=1.0 full dataset + bounded shadow planning; PX4 not required"
	@echo "  DATASET_SHADOW_GOAL_M=0 make dataset-check ...  mapping-only replay without a synthetic goal"
	@echo "  make sim-check                             headless PX4/Gazebo + offboard acceptance"
	@echo "  make external-mode-check                  headless PX4/Gazebo + PX4 External Mode acceptance"
	@echo "  make external-mode-gui                   GUI PX4/Gazebo + RViz + External Mode mission"
	@echo "  MANUAL_TAKEOFF=1 make external-mode-gui  wait for manual Takeoff/arm; harness sends no ARM/TAKEOFF"
	@echo "  make external-mode                       alias for external-mode-gui"
	@echo "  MAP_SCENE=sanity_open|structured_obstacle|long_route|tunnel|clutter|planner_negative|navigation_generalization"
	@echo "  TEST_CASE=positive|degenerate|detour|no_path|comprehensive  MOTION_PRESET=nominal|slow|fast"
	@echo "  SPEED_CAP_MPS=<number>                      temporary speed cap for one mission run"
	@echo "  MAP_PROFILE=<legacy alias> (optional; overrides MAP_SCENE)"
	@echo "  make sim                                   interactive PX4/Gazebo/RViz session; no auto flight"
	@echo "  Map sweep: make build; then MAP_SCENE=<scene> TEST_CASE=positive SPEED_CAP_MPS=5 make external-mode-check"
	@echo "  make status                                live state for the latest session"
	@echo "  make stop                                  stop all workspace-owned runtime session process groups"
	@echo "  Runtime guard                              one workspace-owned simulation; concurrent starts are rejected"
	@echo "  make clean                                 keep build/install; remove stale variants, logs and caches"
	@echo "Artifacts: shared Git root .artifacts/runtime/<workflow>-*/REPORT.html and report.json"
	@echo "PASS requires samples, freshness, validity, cleanup, and workflow-specific acceptance."

build:
	@$(ROS_ENV) $(BUILD_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/build.py build

test:
	@$(ROS_ENV) $(BUILD_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/build.py test
	@$(ROS_ENV) $(BUILD_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/build.py check
	@$(CANONICAL_PYTHON_ENV) $(PYTHON) -m unittest discover -s tools/tests -p 'test_*.py' -v
	@$(CANONICAL_PYTHON_ENV) $(PYTHON) -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v

replay:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py dataset-check --dataset "$(DATASET)" --rate "$(RATE)" --rviz $(FRONTIER_DEBUG_ARG) $(DATASET_SHADOW_GOAL_ARG) $(DATASET_ROS_DOMAIN_ARG)

dataset-check:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py dataset-check --dataset "$(DATASET)" --rate "$(RATE)" $(DATASET_SHADOW_GOAL_ARG) $(DATASET_ROS_DOMAIN_ARG)

sim-check:
	@$(ROS_ENV) export PX4_DIR="$(PX4_DIR)"; $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py sim-check

external-mode-check:
	@$(ROS_ENV) export PX4_DIR="$(PX4_DIR)"; $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py external-mode-check $(MAP_PROFILE_ARG) $(MAP_SCENE_ARG) $(TEST_CASE_ARG) $(MOTION_PRESET_ARG) $(MAP_SEED_ARG) $(SPEED_CAP_MPS_ARG)

sim:
	@$(ROS_ENV) export PX4_DIR="$(PX4_DIR)"; $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py sim

external-mode-gui:
	@$(ROS_GUI_ENV) export PX4_DIR="$(PX4_DIR)"; $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py external-mode-gui $(MAP_PROFILE_ARG) $(MAP_SCENE_ARG) $(TEST_CASE_ARG) $(MOTION_PRESET_ARG) $(MAP_SEED_ARG) $(MANUAL_TAKEOFF_ARG) $(SPEED_CAP_MPS_ARG)

external-mode: external-mode-gui

status:
	@$(ROS_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py status

stop:
	@$(ROS_ENV) $(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py stop

clean:
	@$(CANONICAL_PYTHON_ENV) $(PYTHON) tools/runtime/runner.py clean
