---
name: build-and-test
description: Build and test uav-navigation with the repository Makefile and Wave 3 gate
---

Use the system Python and repository Make targets; do not invoke an activated
virtual environment or call colcon directly for normal verification.

- `make setup` prepares the host when required.
- `make build` compiles the ROS workspace.
- `make test` runs build/test checks and the Python unit suites.
- `tools/gate.sh static|python|ros|all` runs the Wave 3 gate; `PACKAGES=` may
  narrow the ROS scope for a task, but the report must state the scope.
- `make replay`/`make dataset-check` require recorded data; `make sim-check`
  and `make run` require PX4/Gazebo and leave processes behind.
- Run `make status`/`make stop` after interrupted simulation runs.

Tests are not flight qualification. Missing or partial runtime evidence is
reported as `NOT_EVALUABLE`, never promoted to PASS.
