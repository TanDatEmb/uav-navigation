# Critical Assessment of Migration Review Reports

This document contains a second-pass critique of the four review reports
(`review_feature_parity.md`, `review_transform_conventions.md`,
`review_timestamp_sync.md`, `review_logging_monitoring.md`) and of the
legacy code itself. The goal is to catch inaccuracies and silent risks
before execution.

---

## 1. Timestamp and Time-System Review — Corrections

### 1.1 Legacy does NOT subscribe `TimesyncStatus`
The legacy `ned_transform.cpp` does **not** subscribe the PX4
`TimesyncStatus` topic. It extracts PX4 wall-clock time from the
`timestamp_sample` field of `/fmu/out/vehicle_odometry`:

```cpp
const double px4_wall_sec = static_cast<double>(msg->timestamp_sample) * 1e-6;
```

This is a different (and simpler) strategy than maintaining a full
TimesyncStatus-based offset. The original timestamp review report was
imprecise on this point.

### 1.2 Offset capture is conditional and tied to sim_time flow
The legacy code refuses to lock the offset until ROS time has advanced
past 1 second:

```cpp
if (now_sec < 1.0) return;
px4_to_ros_offset_sec_.store(now_sec - px4_wall_sec, ...);
```

This prevents the offset from being captured while `/clock` has not yet
reached the node. A naïve migration that removes this guard would
silently produce bad timestamps in simulation.

### 1.3 Reverse conversion for EV output
When publishing `vehicle_visual_odometry` back to PX4, the code converts
ROS/sim time back to PX4 wall-clock microseconds:

```cpp
const double sample_px4_sec = lio_sample.t_sec - offset_sec;
const double now_px4_sec    = this->now().seconds() - offset_sec;
ev.timestamp        = secondsToUsec(now_px4_sec);
ev.timestamp_sample = secondsToUsec(sample_px4_sec);
```

Any new timestamp utility must support **both directions** (PX4→ROS and
ROS→PX4), not just a one-way conversion.

### 1.4 `voxmap_node.cpp` uses nanosecond timestamps, not seconds
The `LioBuffer` class stores `int64_t t_ns` and enforces a strict
monotonic push invariant:

```cpp
if (!buf_.empty() && s.t_ns <= buf_.back().t_ns) {
    non_monotonic_count_.fetch_add(1, ...);
    return;
}
```

It also drops samples that are older than a 5-second sliding window and
tracks overflow/lookup-miss counters for diagnostics. A new
`px4_common::time::PoseBuffer` must replicate these invariants or the
mapping pipeline will silently produce stale/invalid raycast origins.

### 1.5 Strategic decision still needed
The precision-landing codebase recently adopted the **PX4 Scenario A**
assumption: ROS 2 nodes use OS clock and the uXRCE-DDS agent handles
cross-time translation automatically. The legacy navigation repo,
however, explicitly manages a PX4-to-ROS offset because it timestamps
scan data from FAST-LIO2 and must align it with PX4 odometry.

Before migration, decide:

- **Option A (uXRCE agent sync)**: Simpler, matches precision landing,
  but requires verifying that the agent actually aligns
  `timestamp_sample` with ROS `header.stamp` under Gazebo sim-time.
- **Option B (manual offset)**: Matches legacy exactly, adds complexity,
  and is vulnerable to RTF drift.

Do not mix the two strategies within the same node.

---

## 2. Frame and Quaternion Conventions — Corrections and Risks

### 2.1 Eigen constructor order is `(w, x, y, z)`, internal storage is `[x, y, z, w]`
The transform review report conflated these two orders. This is a
common source of silent quaternion bugs.

- `Eigen::Quaterniond(w, x, y, z)` is the **constructor**.
- `q.coeffs()` returns `[x, y, z, w]`.
- `q.x()`, `q.y()`, `q.z()`, `q.w()` return the individual components.

Legacy `ned_transform.cpp` constructs from PX4 `VehicleOdometry.q` as:

```cpp
p.q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
```

PX4 stores quaternions as `[w, x, y, z]`, so this is correct. Any wrapper
that exposes `ArrayToEigenQuat(array)` must document that the input is
`[w, x, y, z]`.

### 2.2 `ned_transform.cpp` ENU↔NED point transform matches the new repo
Legacy static path:

```cpp
static Eigen::Matrix3d lioWorldToNedMatrix() {
    return (Eigen::Matrix3d() <<
        0.0, 1.0,  0.0,
        1.0, 0.0,  0.0,
        0.0, 0.0, -1.0).finished();
}
```

This is exactly `(x, y, z) → (y, x, -z)`, which matches the new
`px4_common::math::EnuToNed()` implementation. Cross-verified.

### 2.3 Aircraft↔baselink is implemented in legacy but missing in new repo
Legacy defines:

```cpp
const Eigen::Matrix3d C_FRD_FLU = (Eigen::Matrix3d() <<
    1.0,  0.0,  0.0,
    0.0, -1.0,  0.0,
    0.0,  0.0, -1.0).finished();
```

This is the Forward-Right-Down (aircraft) ↔ Forward-Left-Up (base_link)
rotation used by PX4 and ROS REP-103. The new repo currently has only a
placeholder comment for this in `frame_transforms.hpp`. It must be
implemented before any external-vision or odometry bridge node is
migrated, otherwise orientations published to PX4 will be wrong.

### 2.4 `px4_to_ros_orientation` and `ros_to_px4_orientation` are two-step transforms
The legacy `frame_transforms.h` defines:

```cpp
px4_to_ros_orientation(q) = baselink_to_aircraft(ned_to_enu_orientation(q));
ros_to_px4_orientation(q) = aircraft_to_baselink(enu_to_ned_orientation(q));
```

A migrated bridge node must follow the same composition or document why it
deviates. The current new `frame_transforms.hpp` only wraps NED↔ENU and
covariance aliases; it does not yet implement these two-step helpers.

### 2.5 Yaw extraction in legacy is frame-specific
Legacy `ned_transform.cpp` extracts yaw from a NED rotation matrix as:

```cpp
return std::atan2(R(1, 0), R(0, 0));
```

This is correct for NED yaw (rotation about Down). The new repo's
`QuaternionGetYaw` uses the generic Eigen `atan2(2*(w*z + x*y),
1 - 2*(y^2 + z^2))`. Both are equivalent for ZYX yaw, but the NED matrix
form is less sensitive to gimbal-lock edge cases in the navigation
controller. Keep both forms for different contexts.

---

## 3. Feature Parity Review — Additions

### 3.1 `px4_msgs` is a source submodule
The original feature-parity report stated `px4_msgs` was an installed
package. Verified via `git submodule status`:

```
ff7ae284c4b9cb1c39d182e9f1a1343b3817011e src/px4_msgs (heads/main)
```

This is correct and should remain so.

### 3.2 `ned_transform.cpp` is more than a coordinate bridge
It also publishes FAST-LIO2 external vision (`vehicle_visual_odometry`)
to PX4 with an initial yaw/translation alignment phase:

```cpp
const double yaw_delta = wrapPi(yawFromNedRotation(q_px4) - yawFromNedRotation(q_raw));
visual_align_q_ = yawOnlyNedRotation(yaw_delta);
visual_align_t_ = t_px4 - visual_align_q_ * p_raw;
```

A migration must decide whether to preserve this EV alignment behavior or
replace it with a simpler `px4_ros_com` offboard/EV publisher.

### 3.3 `voxmap_node.cpp` has production details not yet in the new repo
- Pose buffer with nanosecond monotonic invariant and diagnostic counters.
- Point-cloud field introspection (x/y/z/intensity offsets).
- 15-meter radius distance-based eviction.
- Per-frame timing instrumentation.

These are not optional optimizations; they affect correctness and
debuggability.

### 3.4 `navigation3d_controller.cpp` has message-version detection
It builds topic names dynamically based on `px4_msgs` message version:

```cpp
topic.vehicle_local_position = "/fmu/out/vehicle_local_position" + getMessageNameVersion<VehicleLocalPosition>();
```

This is needed because PX4 message topics gain `_vN` suffixes when
message definitions change. The new repo must either replicate this or
hardcode version-specific topic names and verify them per `px4_msgs`
commit.

---

## 4. Logging and Monitoring Review — Additions

### 4.1 Distinguish C++ node logging from Python tooling
The logging review was generic. The legacy repo actually has two layers:

- **C++ nodes**: `RCLCPP_INFO/WARN/ERROR`, throttled logs, per-frame timing
  logs, mode-transition logs.
- **Python tools**: `recorder.py` (flight data recorder) and
  `visualize.py` (HTML replay generator).

A logging policy should address both layers separately.

### 4.2 `navigation3d_controller.cpp` already emits structured runtime logs
Examples observed:

```cpp
RCLCPP_INFO(get_logger(), "Subscribing to: %s", pos_topic.c_str());
RCLCPP_INFO(get_logger(), "[NED] F%lu: %zu pts | interp %.2fms tf %.2fms pub %.2fms", ...);
RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, ...);
```

The new repo currently has no nodes, so there is nowhere to log yet.
Logging policy becomes actionable only after the first node migration.

### 4.3 `trajectory_logger.cpp` writes CSV with a fixed schema
The schema includes planned path and executed state columns. A new
logger should keep the same column order or explicitly document breaking
changes, because downstream `visualize.py` parses these CSV files.

---

## 5. Silent Risks to Address Before Execution

| Risk | Why it is silent | Mitigation |
|------|------------------|------------|
| Quaternion order confusion | Constructor vs `coeffs()` | Add `static_assert`-style tests; document `wxyz` in every array helper name |
| Aircraft↔baselink missing | New repo has placeholder only | Implement and unit-test before EV/odom migration |
| Timestamp offset not captured | Sim-time and wall-clock diverge | Choose Option A or B and add monotonic/RTF guards |
| Pose buffer non-monotonic drops | `LioBuffer` drops out-of-order samples without loud warnings | Keep atomic counters and publish diagnostics |
| Message-version topic mismatch | `px4_msgs` updates change `/fmu/out/...` suffixes | Replicate version detection or pin topic names per submodule commit |
| Callback-group/QoS mismatches | Wrong QoS causes dropped PX4 messages | Preserve best_effort QoS and separate I/O + compute groups |
| Frame ID strings | `map_ned`, `camera_init`, `base_link`, `aircraft` | Define constants in `px4_common` and enforce via tests |

---

## 6. Recommended Pre-Execution Checklist

1. Decide timestamp strategy (manual offset vs uXRCE agent sync) and
   document it in `docs/architecture.md`.
2. Implement aircraft↔baselink and ECEF helpers in `px4_ros_com` with
   unit tests.
3. Add quaternion-order tests that explicitly fail if convention changes.
4. Define frame-ID string constants in `px4_common`.
5. Decide whether to preserve FAST-LIO2 EV alignment behavior from
   `ned_transform.cpp`.
6. Verify `px4_msgs` submodule version matches the PX4 firmware version
   that will be used in SITL/HIL.
7. Add a logging policy document before migrating the first executable
   node.

---

*Assessment written after reading legacy `ned_transform.cpp`,
`voxmap_node.cpp`, `navigation3d_controller.cpp`, and the four review
reports.*
