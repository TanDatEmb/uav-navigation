# Coding Conventions — uav-navigation

## Package Types and Submodules

- `px4_msgs`: MUST be a Git submodule pointing to `PX4/px4_msgs`. Message definitions must match the PX4 firmware version used in simulation/flight.
- `px4_ros_com` (this workspace): self-developed bridge/transforms/offboard helpers. Do NOT submodule the upstream `PX4/px4_ros_com`; it is ROS 1 legacy, weakly maintained for ROS 2 Jazzy, and brings dependency conflicts. Re-implement only the helpers we need on top of `px4_common` math and `px4_msgs`.
- Header-only packages: use `INTERFACE` library type with proper `install(TARGETS ...)` and `ament_export_targets`.
- Compiled C++ packages: use `SHARED` library type with explicit `CMAKE_CXX_STANDARD 17`.
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

- All source comments in English.
- Use Doxygen style for public APIs: `/** ... */`.
- No commented-out debug code in final commits.
- Explain **why**, not what, when intent is non-obvious.

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
- Every transform function name must state source and destination frames
  unambiguously (e.g. `EnuToNed`, `QuaternionAircraftToBaselink`).

## Quaternion Conventions

- **Eigen internal storage**: `q.coeffs()` returns `[x, y, z, w]`.
- **Eigen constructor**: `Eigen::Quaterniond(w, x, y, z)`.
- **PX4 storage**: arrays are `[w, x, y, z]`.
- **Conversion helpers**:
  - `EigenQuatToArray()` → `[w, x, y, z]`.
  - `ArrayToEigenQuat()` expects `[w, x, y, z]`.
- **Euler angles**: ZYX intrinsic sequence (yaw, pitch, roll) unless
  explicitly documented otherwise.
- **Transform direction**: all frame transforms are **passive** (rotate the
  coordinate frame, not the vector), implemented as
  `q_rotation * q * q_rotation.conjugate()` for quaternions.

## Time Conventions

- **Single clock source**: ROS 2 clock (`rclcpp::Clock`, `this->now()`,
  `header.stamp`).
- **PX4 boundary**: MicroXRCE-DDS agent translates `timestamp_sample`
  (PX4 wall-clock microseconds) into ROS 2 time. Do not maintain a manual
  offset inside nodes.
- **Internal messages**: use `std::chrono::nanoseconds` or `rclcpp::Time`.
  Avoid mixing `double seconds` and `int64_t nanoseconds` in the same buffer.
- **Pose buffers**: enforce strict monotonic timestamps and track
  non-monotonic / overflow / miss counters for diagnostics.

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

## Frame Conventions

Frame conventions will be documented in `docs/architecture.md` once finalized. Every transform must have a named, versioned convention.

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
- [ ] `clang-format` passes.
