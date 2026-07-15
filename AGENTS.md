# AGENTS.md — uav-navigation

## Project Context

This is a clean-room ROS 2 Jazzy + PX4 autopilot navigation stack for autonomous UAV in GPS-denied and cluttered environments.

It is the successor to the legacy `Mapping_and_Navigation_for_PX4_UAV` prototype. Lessons from that codebase are enforced here from the first commit.

## Working Conventions

- Read `docs/conventions.md` before editing source code.
- All source comments, commit messages, and first-party documentation in English.
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
  fast_lio/                  # MID-360 LiDAR-inertial odometry
  px4_mapping/               # global map and PX4 odometry boundaries
  px4_nav_common/            # project-specific mapping/navigation contracts
  px4_navigation/            # obstacle perception and planning libraries
  px4_msgs/                  # upstream PX4 messages (submodule)
  px4_ros2_interface_lib/    # upstream PX4 ROS 2 interface (submodule)
  px4_ros2_utils/            # generic PX4/ROS 2 utilities (submodule)
assets/                      # RViz configuration
config/                      # cross-package runtime parameters
docs/                        # architecture and engineering contracts
scripts/                     # SITL orchestration and analysis
tests/                       # workspace integration tests
tools/                       # build, formatting, and PX4 helper scripts
vendor/px4_autopilot_extras/ # reproducible PX4/Gazebo additions
```

`px4_visualization` and the local `px4_ros_com` package were removed. Keep
visualization assets under `assets/` and use `px4_ros2_utils` only for generic
PX4/ROS 2 boundary helpers.

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

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **uav-navigation** (6494 symbols, 12310 relationships, 155 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> If any GitNexus tool warns the index is stale, run `npx gitnexus analyze` in terminal first.

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `gitnexus_impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `gitnexus_detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `gitnexus_query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `gitnexus_context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `gitnexus_impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename` which understands the call graph.
- NEVER commit changes without running `gitnexus_detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/uav-navigation/context` | Codebase overview, check index freshness |
| `gitnexus://repo/uav-navigation/clusters` | All functional areas |
| `gitnexus://repo/uav-navigation/processes` | All execution flows |
| `gitnexus://repo/uav-navigation/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
