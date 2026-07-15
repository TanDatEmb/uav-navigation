# Coding Conventions — uav-navigation

## Package Types and Submodules

- `px4_msgs`: MUST be a Git submodule pointing to `PX4/px4_msgs`. Message definitions must match the PX4 firmware version used in simulation/flight.
- `px4_ros2_utils` (submodule): external C++20 header-first PX4 ↔ ROS 2 utility library (`TanDatEmb/px4_ros2_utils`). Provides frame transforms, time constants, math, parameters, QoS, and geometry bridges. Do NOT submodule the upstream `PX4/px4_ros_com`; it is ROS 1 legacy, weakly maintained for ROS 2 Jazzy, and brings dependency conflicts.
- `px4_nav_common` (this workspace): project-specific contracts built on top of `px4_ros2_utils` and `px4_msgs`.
- Header-only packages: use `INTERFACE` library type with proper `install(TARGETS ...)` and `ament_export_targets`.
- Compiled C++ packages: use `SHARED` library type with explicit `CMAKE_CXX_STANDARD 20`.
- `buildtool_depend` must be `ament_cmake` (not `ament_cmake_ros`) for consistency unless a package genuinely requires ROS-specific CMake macros.

## C++ Style

Primary reference: ROS 2 C++ Style Guide.
Supplement: PX4 Style Guide for safety-critical and autopilot-adjacent code.

| Item                  | Rule                               |
| --------------------- | ---------------------------------- |
| Classes / structs     | `PascalCase`                       |
| Functions / methods   | `snake_case()`                     |
| Member variables      | `snake_case_`                      |
| Local variables       | `snake_case`                       |
| Parameters            | `snake_case`                       |
| Namespaces            | `snake_case`                       |
| File names            | `snake_case.cpp`, `snake_case.hpp` |
| `constexpr` constants | `kPascalCase` (PX4 preferred)      |
| `#define` constants   | `SCREAMING_SNAKE_CASE`             |
| Indentation           | 4 spaces                           |
| Braces                | attach (K&R)                       |
| Line length           | ≤ 100 characters                   |

## Comments and Documentation

- All first-party source comments, commit messages, and repository documentation
  are written in English. Standard technical names, symbols, and units are kept
  unchanged.
- Use Doxygen style for public APIs: `/** ... */`.
- No commented-out debug code in final commits.
- Explain **why**, not what, when intent is non-obvious.
- Do not track one-time review, audit, status-snapshot, or remediation reports.
  Promote durable findings into the relevant architecture/design document and
  delete the temporary report.
- Each engineering contract has one authoritative document; update it instead of
  creating overlapping notes.

## Parameters

- Every new class member must have a matching `declare_parameter` + `get_parameter` call in the same commit.
- Prefer typed ROS 2 parameters with descriptors and bounds.
- No magic numbers in logic; use named constants or YAML parameters.

## Control Flow

- Single setpoint / command publish per control cycle.
- Handle invalid `dt`, missing data, or stale input early with explicit returns or errors.
- Failsafe paths must be obvious and documented.

## Frame Conventions

- **NED** (`map_ned`): North-East-Down, PX4 local world frame.
  - X: North positive, Y: East positive, Z: Down positive.
- **ENU**: East-North-Up, ROS default world frame.
  - Conversion from ENU to NED point: `(x, y, z) → (y, x, -z)`.
  - Conversion from NED to ENU point: `(x, y, z) → (y, x, -z)` (involution).
- **Aircraft body frame** (`aircraft`, FRD): Forward-Right-Down, PX4 body frame.
- **Base link body frame** (`base_link`, FLU): Forward-Left-Up, ROS REP-103 body frame.
- **FRD↔FLU rotation**: `diag(1, -1, -1)` applied as a passive frame rotation.
- Generic frame conversion must use `px4_ros2_utils::frame` directly (for
  example `enu_to_ned`, `enu_to_ned_pose`, and `flu_to_frd`). Do not duplicate
  these operations with local sign flips or matrices.

## Quaternion Conventions

- **Eigen internal storage**: `q.coeffs()` returns `[x, y, z, w]`.
- **Eigen constructor**: `Eigen::Quaterniond(w, x, y, z)`.
- **PX4 storage**: arrays are `[w, x, y, z]`.
- **Conversion helpers**:
  - `EigenQuatToArray()` → `[w, x, y, z]`.
  - `ArrayToEigenQuat()` expects `[w, x, y, z]`.
- **Euler angles**: ZYX intrinsic sequence (yaw, pitch, roll) unless
  explicitly documented otherwise.
- **Transform direction**: treat frame conversion as a coordinate
  representation change. Use the complete pose helpers for world+body changes;
  do not derive orientation conversion by negating Euler angles.

## Time Conventions

- The system has **two explicit clock domains**:
  - ROS messages and internal freshness/timeout logic use the node-owned ROS 2
    clock (`this->now()`, `rclcpp::Time`, and `header.stamp`).
  - PX4 message fields `timestamp` and `timestamp_sample` use microseconds in
    the PX4 boot-time domain.
- At every PX4 boundary, use `px4_ros2_utils::time::Timesync`:
  - PX4 input → ROS processing: `Timesync::toROS(px4_timestamp_us)`.
  - ROS measurement/publication → PX4 output: `Timesync::toPX4(ros_time)`.
- `timestamp` means publication time. `timestamp_sample` means measurement
  acquisition time. They may differ and must not be collapsed into one value.
- Never cast `this->now().nanoseconds() / 1000` directly into a PX4 timestamp,
  and never use a PX4 integer timestamp directly as a ROS header stamp.
- `Timesync` must receive `node->get_clock()`; utilities must not construct a
  separate clock. External mode requires a valid `TimesyncStatus`. Simulation
  mode is only for the documented zero-offset `/clock` configuration.
- Failed conversion and PX4 timestamp zero are invalid: drop/defer the sample;
  do not guess or fall back to another clock.
- Internal pose buffers store one documented ROS nanosecond domain, enforce
  strict monotonic timestamps, and track non-monotonic/overflow/miss counters.

## Logging and Monitoring Conventions

- Use `RCLCPP_INFO/WARN/ERROR` in C++ nodes; prefer `RCLCPP_*_THROTTLE` for
  high-frequency messages.
- Log lifecycle events (node start, parameter load, mode transitions) at
  `INFO`.
- Log recoverable faults at `WARN` with a counter if they repeat.
- Log safety-critical or unrecoverable faults at `ERROR`.
- Never run a node in silence: every executable must emit at least one
  startup log line and publish a periodic health/status topic.
- Python tooling (`recorder.py`, `visualize.py`) keeps CSV schemas stable
  or documents breaking changes explicitly.

The authoritative runtime topic contract is documented in `docs/architecture.md`.
The authoritative frame definitions and boundary rules are documented in
`docs/frame_contract.md`.

## Formatting and Linting

- `.clang-format` enforces the style defined above.
- `.clang-tidy` catches common C++ issues.
- `CPPLINT.cfg` enforces Google-style lint checks.

Run formatting before committing:

```bash
./tools/format.sh
```

## Pre-commit Checklist

- [ ] New members have declare + load parameter.
- [ ] `compute_*` functions receive correct input type and unit.
- [ ] No Vietnamese comments.
- [ ] No leftover debug logs.
- [ ] Single publish path per cycle.
- [ ] No direct ROS↔PX4 timestamp cast; all boundary timestamps use `Timesync`.
- [ ] Documentation is English-only and contains no temporary review artifact.
- [ ] `clang-format` passes.
