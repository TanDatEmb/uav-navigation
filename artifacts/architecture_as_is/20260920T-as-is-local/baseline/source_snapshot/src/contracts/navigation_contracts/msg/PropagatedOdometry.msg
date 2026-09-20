# Product-owned propagated state contract.
# The nested odometry header is the only source timestamp and frame metadata.
# localization_epoch invalidates state across estimator public-frame resets;
# sequence is strictly increasing within one epoch.
nav_msgs/Odometry odometry
uint64 localization_epoch
uint64 sequence
