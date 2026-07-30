SHELL := /bin/bash
DATASET ?=
RATE ?= 1.0
REPEAT ?= 1
MAX_LIDAR ?= 0
DRY_RUN ?= 0
MODE ?= release
PACKAGES ?=
DATASET_TOOL := python3 tools/data.py
ROS_ENV := source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
BUILD_TOOL := python3 tools/runtime/build.py --mode "$(MODE)"

.PHONY: help build test check clean clean-artifacts vendor-check data-list data-fetch data-check data-smoke data-run data-replay data-cleanup data-view data-report data-test runtime-repro

help:
	@$(DATASET_TOOL) --help

build:
	@$(BUILD_TOOL) build $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

test:
	@$(BUILD_TOOL) test $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

check:
	@$(BUILD_TOOL) check

vendor-check:
	@python3 tools/vendor_check.py

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
	@$(ROS_ENV) $(DATASET_TOOL) replay --dataset "$(DATASET)" --rate "$(RATE)"

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
