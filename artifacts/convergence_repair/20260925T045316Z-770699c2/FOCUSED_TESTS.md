# Focused tests

- `navigation_planning_backend/test_planner_config`: 105/105 passed; includes steady A/J round-down, no-extra-correction interior, cancellation, support, non-steady and overspeed contracts.
- `navigation_planning/test_planning_contracts`: 26/26 passed; includes request predecessor value validation and substitution/identity tests.
- `navigation_planning_backend/test_planner_facade`: 43/43 passed.
- Runtime terminal monitor: 60/60 passed.
- Fresh selected CTest run after final product commit: 7 packages, 19/19 CTest targets passed (`navigation_planning`, `navigation_planning_backend`, `navigation_runtime`, `navigation_execution`, `navigation_contracts`, `navigation_mission`, `px4_navigation_external_mode`).
- Full pinned workspace CTest is not reported PASS: the earlier 35-package run had failures in 13 pinned `px4_ros2_interface_lib`/example targets (external dependency lint and FMU wait failures). Product-relevant selected CTest is green; full workspace gate remains externally blocked.
