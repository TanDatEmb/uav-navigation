SHELL := /bin/bash
DATASET ?=
RATE ?= 1.0
DATA_ROOT ?= data
DATASET_TOOL := python3 tools/dev/dataset.py --data-root "$(DATA_ROOT)"

.PHONY: help build test check clean clean-artifacts dataset-inspect dataset-smoke dataset-run dataset-ros dataset-view

help:
	@$(DATASET_TOOL) help

build:
	@source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

test:
	@source /opt/ros/jazzy/setup.bash && colcon test --event-handlers console_cohesion+

check:
	@source /opt/ros/jazzy/setup.bash && colcon test-result --verbose

clean:
	@$(DATASET_TOOL) clean

clean-artifacts:
	@$(DATASET_TOOL) clean-artifacts

dataset-inspect:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) inspect --dataset "$(DATASET)"

dataset-smoke:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) smoke --dataset "$(DATASET)"

dataset-run:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) run --dataset "$(DATASET)"

dataset-ros:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) ros --dataset "$(DATASET)" --rate "$(RATE)"

dataset-view:
	@test -n "$(DATASET)" || { echo "DATASET is required" >&2; exit 64; }
	@$(DATASET_TOOL) view --dataset "$(DATASET)"
