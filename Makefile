DATASET ?= swarm_lio2_mutual_avoidance_uav1
RUN ?= swarm_lio2_mutual_avoidance_uav1_release_run_20260728
DATA_ROOT := data/external/mid360/$(DATASET)
INSPECTION := reports/m1_dataset/$(DATASET)_inspection_20260728
PARAMS := src/navigation_estimator/fast_lio_ros/config/mid360_mutual_avoidance_uav1.yaml

.PHONY: dataset-download dataset-inspect dataset-convert dataset-sanitize dataset-run dataset-reference dataset-compare dataset-view

dataset-download:
	tools/datasets/m1_mid360.sh download "$(DATASET)"

dataset-inspect:
	python3 tools/datasets/m1_mid360_bag.py inspect --bag "$(DATA_ROOT)/original/mutual_avoidance_uav1.bag" --output "$(INSPECTION)"

dataset-convert:
	python3 tools/datasets/m1_mid360_bag.py convert --bag "$(DATA_ROOT)/original/mutual_avoidance_uav1.bag" --output "$(DATA_ROOT)/converted/mutual_avoidance_uav1" --livox-source src/navigation_estimator/livox_ros_driver2_interface

dataset-sanitize:
	tools/datasets/m1_mid360.sh sanitize "$(DATASET)"

dataset-run:
	tools/datasets/m1_mid360.sh run "$(DATASET)" "$(RUN)"

dataset-reference:
	tools/datasets/m1_mid360.sh reference "$(DATASET)" "$(RUN)"

dataset-compare:
	tools/datasets/m1_mid360.sh compare "$(DATASET)" "$(RUN)"

dataset-view:
	tools/datasets/m1_mid360.sh view "$(RUN)"
