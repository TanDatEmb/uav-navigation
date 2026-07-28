# Bringup configuration boundary

Estimator parameters are owned by `fast_lio_ros/config` so there is one schema
for its node. This package only composes that configuration with the sensor-frame
publisher. Replace all calibration placeholders in the selected estimator YAML
and URDF arguments before operating on real data.
