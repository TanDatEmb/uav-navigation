---
name: build-and-test
description: Build and test uav-navigation with the repository Makefile and the rebuild gate
---

Use the system Python and repository Make targets; do not invoke an activated
virtual environment for normal verification.

- `make setup` prepares the host when required.
- `make build` runs `colcon build --packages-up-to $(PKGS)`; `PKGS` defaults to
  `uavnav_core uavnav_interfaces`. Override it with `make build PKGS="..."`.
- `make test` runs `colcon test` for `PKGS`, then `colcon test-result --verbose`.
- `make gate` runs `tools/uavnav/gate.sh all`. The script also takes
  `static|python|ros`, and its last line is `GATE_RESULT=PASS|FAIL`.
- `make clean` only prints what to remove (`log/`); `build/` and `install/` are
  a shared incremental build and are never deleted.
- Machine limit: builds already use `-j3` and 2 colcon workers; do not raise
  them, and run at most 3 agents in parallel.

Tests are not flight qualification. Missing or partial runtime evidence is
reported as `NOT_EVALUABLE`, never promoted to PASS.
