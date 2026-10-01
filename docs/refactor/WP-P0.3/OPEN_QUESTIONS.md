# WP-P0.3 — Open questions

1. **I2 planner API:** owner decision 2026-09-30 P03-I2: defer ambient planner API removal to W3-B5. A future wave must first verify `PlanningRequest` ownership and port product-path tests before deleting symbols.

2. **G1 environment invocation:** calling `tools/gate.sh` without sourcing `/opt/ros/jazzy/setup.bash` makes `/usr/bin/python3` miss `ament_package`; the sourced ROS gate passed. Should the repository gate wrapper source Jazzy itself, or should CI keep the environment contract explicit?

3. **Runtime qualification scope:** A1 has no new SITL/replay evidence. Runtime qualification remains deferred to the wave-level representative evidence plan.
