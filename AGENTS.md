# AGENTS.md — uav-navigation

## Project Context

This is a clean-room ROS 2 Jazzy + PX4 autopilot navigation stack for autonomous UAV in GPS-denied and cluttered environments.

It is the successor to the legacy `Mapping_and_Navigation_for_PX4_UAV` prototype. Lessons from that codebase are enforced here from the first commit.

## Working Conventions

- Read `docs/conventions.md` before editing source code.
- All comments in English.
- Classes `PascalCase`; functions `snake_case()`; member variables `snake_case_`.
- Every new member variable must have a matching parameter declaration and load in the same commit.
- No magic numbers in logic — use named constants or ROS 2 parameters.
- Single setpoint / command publish per control cycle.
- Remove debug logs before final commits.

## Critical Rules from the Legacy Project

1. **Declare → Load → Use**: new header members require `declare_parameter` + `get_parameter` in the constructor.
2. **Refactor protocol**: when changing function signatures, create an old→new mapping table and verify units, semantics, and ranges.
3. **Frame convention**: all transforms must be documented in `docs/architecture.md`. No implicit frame changes.
4. **Safety first**: failsafe paths must be explicit and logged.
5. **Pre-commit checklist**:
   - [ ] new member has declare + load parameter
   - [ ] compute_* receives correct input type and unit
   - [ ] no Vietnamese comments
   - [ ] no leftover debug logs
   - [ ] single publish path per cycle

## Repository Layout

Standard ROS 2 workspace structure:

```
src/
  px4_msgs/            # upstream PX4 uORB message definitions (submodule)
  px4_common/          # shared math, geometry, parameter helpers
  px4_mapping/         # odometry, local map
  px4_navigation/      # planner, controller, state machine
  px4_ros_com/         # ROS 2 ↔ PX4 bridge, transforms
  px4_visualization/   # RViz, plotting, bag helpers
config/              # global YAML parameters
launch/              # top-level launch files
docs/                # architecture, conventions
tools/               # helper scripts
tests/               # integration tests
```

## Build

```bash
cd /home/letandat/Dev/uav-navigation
./tools/build.sh
```

## Commit Policy — highest priority rule

- **NEVER commit without explicit human approval.** This is the highest-priority
  working rule for this repository. No exceptions.
- Do **not** assume a bug is fixed and commit on your own. The fix must be
  reviewed and explicitly approved first.
- Do **not** mark functionality as "working" before the human has verified it.
- Keep commits small and focused on one concern.
- Use English commit messages.
- If in doubt, ask before committing.
