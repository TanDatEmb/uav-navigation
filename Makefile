SHELL := /bin/bash
DATASET ?=
RATE ?= 1.0
REPEAT ?= 1
MAX_LIDAR ?= 0
DATA_ROOT ?= data
MODE ?= release
PACKAGES ?=
DATASET_TOOL := python3 tools/dev/dataset.py --data-root "$(DATA_ROOT)"
ROS_ENV := source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
BUILD_TOOL := python3 tools/runtime/build.py --mode "$(MODE)"

.PHONY: help build test check clean clean-artifacts dataset-inspect dataset-smoke dataset-run dataset-ros dataset-view runtime-repro

help:
	@$(DATASET_TOOL) help

build:
	@$(BUILD_TOOL) build $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

test:
	@$(BUILD_TOOL) test $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

check:
	@$(BUILD_TOOL) check

clean:
	@$(DATASET_TOOL) clean

clean-artifacts:
	@$(DATASET_TOOL) clean-artifacts

dataset-inspect:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) inspect --dataset "$(DATASET)"

dataset-smoke:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) smoke --dataset "$(DATASET)"

dataset-run:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) run --dataset "$(DATASET)"

dataset-ros:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) ros --dataset "$(DATASET)" --rate "$(RATE)"

dataset-view:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) view --dataset "$(DATASET)"

runtime-repro:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) python3 tools/runtime/repro.py --dataset "$(DATASET)" \
		--repeat "$(REPEAT)" --max-lidar "$(MAX_LIDAR)" --mode "$(MODE)"
