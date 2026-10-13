---
name: build-and-test
description: Build and test uav-navigation with the repository Makefile and the rebuild gate
---

Use the system Python and repository Make targets; do not invoke an activated
virtual environment for normal verification.

- `make setup` prepares the host when required.
- `make build` runs `colcon build --packages-up-to $(PKGS)`; `PKGS` defaults to
  `uavnav_core uavnav_interfaces uavnav_ros fast_lio_core uavnav_lio_core uavnav_lio uavnav_px4_bridge`.
  Override it with
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
- `make sitl-probe PROFILE=gps|no_gps` (optional `HOVER_MODE=hold`, `DRY_RUN=1`)
  runs `tools/uavnav/sitl_hover.sh --probe`: PX4 SITL + Gazebo (headless) +
  MicroXRCEAgent (UDP 8888) + gz bridges + `tools/uavnav/sitl_probe.py`, each in
  its own process group, bounded at 240 s, always torn down (Ctrl-C is safe;
  never `kill -9` the script). It exits 2 while a `colcon` build or test or
  another SITL runs (`sitl_guard.py`, matched on argv), so never start it
  during a build. The verdict is
  `log/sitl/<UTC stamp>-<profile>-probe/summary.json` (`sitl_checks.py`: seven
  checks plus every probe phase ok). It is
  owner machine time and never part of `make test` or `make gate`; `DRY_RUN=1`
  runs only the prechecks and prints the commands.
- `make sitl-build` builds `uavnav_bringup uav_simulation uav_description
  uavnav_lio uavnav_px4_bridge` (and their dependencies). `make sitl-hover
  PROFILE=gps|no_gps` (optional `SPAWN_YAW_RAD=<rad>`, `HOVER_MODE=hold`,
  `DRY_RUN=1`; `PATTERN=altitude_sweep` comes with S1b Task 13) runs
  `tools/uavnav/sitl_hover.sh --hover`: the probe's PX4/Gazebo/XRCE plus
  `ros2 launch uavnav_bringup s1_estimation.launch.py` (gz bridges, `lio`,
  `px4_bridge`) and `tools/uavnav/sitl_hover.py` (wait for TRACKING, VALID,
  EV fused; arm; climb 2.5 m; 60 s hover; land), bounded at 300 s, same
  refusals and teardown as the probe. Verdict:
  `log/sitl/<UTC stamp>-<profile>-hover[-yaw<rad>]/summary.json`
  (`sitl_checks.evaluate_hover`, fourteen checks plus every phase ok). On a
  failure read `python3 -m tools.uavnav.events <run>/events_lio.jsonl
  --timeline` (and `events_px4_bridge.jsonl`) first. Owner machine time; never
  part of `make test` or `make gate`. Both SITL modes give their ROS 2
  processes `FASTRTPS_DEFAULT_PROFILES_FILE=config/ros/fastdds_sitl.xml` (16 MB
  shared-memory segment; a 645 KB cloud is otherwise lost ~15 % of the time).
  Start a ROS tool that must see the clouds with the same variable.
- PX4 profiles (`config/px4/profiles/*.yaml`) are validated by
  `tools/uavnav/px4_profile.py`; an unknown key or a value other than the
  decided one (D20, D28, D33/O23) refuses start.
- Machine limit: builds already use `-j3` and 2 colcon workers; do not raise
  them, and run at most 3 agents in parallel.

Tests are not flight qualification. Missing or partial runtime evidence is
reported as `NOT_EVALUABLE`, never promoted to PASS.
