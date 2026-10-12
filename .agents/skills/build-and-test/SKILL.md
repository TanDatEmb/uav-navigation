---
name: build-and-test
description: Build and test uav-navigation with the repository Makefile and the rebuild gate
---

Use the system Python and repository Make targets; do not invoke an activated
virtual environment for normal verification.

- `make setup` prepares the host when required.
- `make build` runs `colcon build --packages-up-to $(PKGS)`; `PKGS` defaults to
  `uavnav_core uavnav_interfaces fast_lio_core uavnav_lio_core uavnav_px4_bridge`. Override it with
  `make build PKGS="..."`. `COLCON_EXTRA="--cmake-clean-cache"` adds colcon
  arguments (use it after a package moves); the Makefile sets the colcon
  arguments, never call bare `colcon`.
- `make test` first deletes the stale result XMLs of `PKGS` under `build/<pkg>`
  (only those; `build/` outputs, `install/` and other packages stay), then runs
  `colcon test` for `PKGS` and `colcon test-result --verbose` per package. A bad
  package name stops it before anything runs.
- `make sanitize` builds and tests `uavnav_core` and `uavnav_lio_core` under
  ThreadSanitizer (tests run under `setarch -R`, ASLR off), then rebuilds them
  normally, also on Ctrl-C (exit 130). `fast_lio_core` and `uavnav_px4_bridge`
  are not instrumented. Run nothing heavy meanwhile. A second Ctrl-C during
  that restore leaves a sanitized CMake cache (`UAVNAV_SANITIZE:STRING=thread`)
  that a plain `make build` keeps: run `make sanitize` again, or
  `make build PKGS="uavnav_core uavnav_lio_core" COLCON_EXTRA=--cmake-clean-cache`.
- `make gate` runs `tools/uavnav/gate.sh all`. The script also takes
  `static|python|ros`, and its last line is `GATE_RESULT=PASS|FAIL`.
- `make clean` only prints what to remove (`log/`); `build/` and `install/` are
  a shared incremental build and are never deleted.
- Machine limit: builds already use `-j3` and 2 colcon workers; do not raise
  them, and run at most 3 agents in parallel.

Tests are not flight qualification. Missing or partial runtime evidence is
reported as `NOT_EVALUABLE`, never promoted to PASS.
