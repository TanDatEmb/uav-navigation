# Repository layout

```text
src/navigation_estimator/
  ikfom_vendor/       pinned dependency provenance
  fast_lio_core/      ROS-independent estimator
  fast_lio_ros/       ROS boundary and output publishing
  fast_lio_tools/     offline use of the same pipeline
src/navigation_bringup/  launch/configuration composition
src/uav_description/     sensor-frame source of truth
src/uav_simulation/      Gazebo Harmonic simulation assets
docs/                    architecture, interfaces, verification, ADRs
```

Future world-model, planner, safety, and PX4 packages are intentionally absent.
Configuration names, frame names, and static transforms have one documented
source of truth; do not introduce duplicate conversion helpers.
