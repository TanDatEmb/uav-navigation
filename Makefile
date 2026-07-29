SHELL := /bin/bash
DATASET ?=
RATE ?= 1.0
REPEAT ?= 1
MAX_LIDAR ?= 0
SET ?=
KEEP_ARCHIVE ?= 1
DATA_ROOT ?= data
MODE ?= release
PACKAGES ?=
DATASET_TOOL := python3 tools/data.py
LEGACY_DATASET_TOOL := python3 tools/dev/dataset.py --data-root "$(DATA_ROOT)"
ROS_ENV := source /opt/ros/jazzy/setup.bash; if test -f install/setup.bash; then source install/setup.bash; fi;
BUILD_TOOL := python3 tools/runtime/build.py --mode "$(MODE)"

.PHONY: help build test check clean clean-artifacts data-list data-get data-check data-prepare data-rm data-info data-smoke data-run data-replay data-view data-matrix runtime-repro

help:
	@$(DATASET_TOOL) --help

build:
	@$(BUILD_TOOL) build $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

test:
	@$(BUILD_TOOL) test $(if $(strip $(PACKAGES)),--packages $(PACKAGES),)

check:
	@$(BUILD_TOOL) check

clean:
	@$(LEGACY_DATASET_TOOL) clean

clean-artifacts:
	@$(LEGACY_DATASET_TOOL) clean-artifacts

data-list:
	@$(DATASET_TOOL) list

data-get:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) get --dataset "$(DATASET)"

data-check:
	@$(DATASET_TOOL) check $(if $(strip $(DATASET)),--dataset "$(DATASET)",)

data-prepare:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) prepare --dataset "$(DATASET)" --keep-archive "$(KEEP_ARCHIVE)"

data-rm:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) rm --dataset "$(DATASET)"

data-info:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) info --dataset "$(DATASET)"

data-smoke:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) smoke --dataset "$(DATASET)"

data-run:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) run --dataset "$(DATASET)"

data-replay:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) replay --dataset "$(DATASET)" --rate "$(RATE)"

data-view:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) view --dataset "$(DATASET)"

data-matrix:
	@test -n "$(SET)" || { echo "SET is required" >&2; exit 64; }
	@$(ROS_ENV) $(DATASET_TOOL) matrix --case "$(SET)"

runtime-repro:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(ROS_ENV) python3 tools/runtime/repro.py --dataset "$(DATASET)" \
		--repeat "$(REPEAT)" --max-lidar "$(MAX_LIDAR)" --mode "$(MODE)"
