# Frame Contract v1.0 - UAV Navigation

**Date:** 2026-07-14
**Status:** Phase 1 - Foundation
**Scope:** PX4 + FAST-LIO2 integration for MID-360

---

## 1. Frame Definitions

### 1.1 PX4 Frames (NED Convention)

| Frame | Origin | Axes | Description |
|-------|--------|------|-------------|
| `map_ned` | Local EKF origin | x=North, y=East, z=Down | PX4 local world frame |
| `vehicle_frd` | Vehicle CG | x=Forward, y=Right, z=Down | PX4 body frame |

### 1.2 ROS Standard Frames

| Frame | Origin | Axes | Description |
|-------|--------|------|-------------|
| `base_link` | Vehicle CG | x=Forward, y=Left, z=Up | ROS REP-103 body (FLU) |

### 1.3 FAST-LIO2 Frames

| Frame | Origin | Axes | Description |
|-------|--------|------|-------------|
| `lio_world` | Initialization point | Gravity-aligned, arbitrary yaw | FAST-LIO world |
| `mid360_imu` | IMU chip | x=Forward, y=Left, z=Up | MID-360 IMU (FLU) |
| `mid360_lidar` | Optical center | x=Forward, y=Left, z=Up | MID-360 LiDAR (FLU) |

---

## 2. Transform Naming Convention

### 2.1 Mathematical Notation

```
^A T_B  = Pose of frame B expressed in frame A
       = Transform that takes a point from B to A

p_A = ^A T_B * p_B
```

### 2.2 Code Naming

```cpp
// Full SE(3) transforms
Sophus::SE3d T_map_ned_vehicle_frd;
Sophus::SE3d T_vehicle_frd_mid360_imu;
Sophus::SE3d T_mid360_imu_mid360_lidar;
Sophus::SE3d T_lio_world_mid360_imu;
Sophus::SE3d T_map_ned_lio_world;  // Alignment transform

// Rotation and translation separated (if needed)
Eigen::Quaterniond R_map_ned_vehicle_frd;
Eigen::Vector3d p_map_ned_vehicle_frd;

// Point notation
Eigen::Vector3d p_map_ned;      // Point in map_ned
Eigen::Vector3d p_lio_world;    // Point in lio_world
Eigen::Vector3d p_mid360_lidar; // Point in mid360_lidar
```

### 2.3 Invalid Names (FORBIDDEN)

```cpp
// These names are BANNED
R_total           // Ambiguous
C_FRD_FLU         // Hardcoded matrix
t_world           // No frame reference
extrinsic_T       // Ambiguous direction
lidar_offset      // Ambiguous
body_pose         // Ambiguous
sensor_pose       // Ambiguous
T_lidar_in_imu    // Wrong: should be T_mid360_imu_mid360_lidar
```

---

## 3. Transform Chains

### 3.1 LiDAR Point → LIO World

```
p_lio_world = T_lio_world_mid360_imu * T_mid360_imu_mid360_lidar * p_mid360_lidar
```

### 3.2 LiDAR Point → map_ned (via alignment)

```
p_map_ned = T_map_ned_lio_world * p_lio_world
          = T_map_ned_lio_world * T_lio_world_mid360_imu * T_mid360_imu_mid360_lidar * p_mid360_lidar
```

### 3.3 FAST-LIO IMU Pose → PX4 EV

```
T_map_ned_vehicle_frd = T_map_ned_lio_world * T_lio_world_mid360_imu * T_vehicle_frd_mid360_imu.inverse()
```

### 3.4 Alignment Capture

At time t_0:
```
T_map_ned_lio_world = T_map_ned_vehicle_frd(t_0) * T_vehicle_frd_mid360_imu * T_lio_world_mid360_imu(t_0).inverse()
```

---

## 4. YAML Configuration Format

### 4.1 Transform Specification

```yaml
transforms:
  # Convention: parent_T_child
  # All translations in meters, rotations as quaternions [w, x, y, z]

  vehicle_frd_T_mid360_imu:
    translation_m: [0.0, 0.0, 0.0]  # [x, y, z]
    quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]  # Identity
    # Alternative: rotation_rpy_deg: [0, 0, 0]  # Optional

  mid360_imu_T_mid360_lidar:
    translation_m: [-0.011, -0.02329, 0.04412]
    quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
```

### 4.2 Loading Code Pattern

```cpp
// Load transform from YAML
Sophus::SE3d loadTransform(const YAML::Node& node) {
    std::vector<double> t = node["translation_m"].as<std::vector<double>>();
    std::vector<double> q = node["quaternion_wxyz"].as<std::vector<double>>();

    Eigen::Vector3d translation(t[0], t[1], t[2]);
    Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
    rotation.normalize();

    return Sophus::SE3d(rotation, translation);
}
```

---

## 5. Frame Validation

### 5.1 Header Frame ID Checks

```cpp
// REQUIRED: All cloud messages must have valid frame_id
void validateFrameId(const std::string& frame_id) {
    static const std::set<std::string> valid_frames = {
        "map_ned",
        "vehicle_frd",
        "base_link",
        "lio_world",
        "mid360_imu",
        "mid360_lidar"
    };

    if (valid_frames.find(frame_id) == valid_frames.end()) {
        throw std::invalid_argument("Invalid frame_id: " + frame_id);
    }
}
```

### 5.2 Transform Algebra Tests

```cpp
// Unit test: T_AB * T_BC = T_AC
TEST(FrameContract, TransformChain) {
    Sophus::SE3d T_AB = ...;
    Sophus::SE3d T_BC = ...;
    Sophus::SE3d T_AC = ...;

    EXPECT_TRUE(T_AB * T_BC.isApprox(T_AC));
}

// Unit test: T_AB.inverse() * T_AB = Identity
TEST(FrameContract, Inverse) {
    Sophus::SE3d T_AB = ...;
    EXPECT_TRUE((T_AB.inverse() * T_AB).isApprox(Sophus::SE3d()));
}
```

---

## 6. Migration Checklist

### 6.1 Remove Old Code

- [ ] Delete `LioWorldToNedMatrix()` hardcoded function
- [ ] Remove `C_FRD_FLU` from LIO pipeline
- [ ] Replace `extrinsic_T` with full transform spec
- [ ] Rename `T_lidar_in_imu_` → `T_mid360_imu_mid360_lidar_`
- [ ] Remove all `R_total`, `t_world` variables

### 6.2 Add New Code

- [ ] Frame enum/validation class
- [ ] Transform loader from YAML
- [ ] SE3 wrapper with frame checking
- [ ] Unit tests for transform algebra

### 6.3 Update Config

- [ ] Replace old extrinsic configs
- [ ] Add `transforms:` section to all YAMLs
- [ ] Document frame conventions in README

---

## 7. SDF Model Updates

### 7.1 Unified Origin

```xml
<!-- BEFORE (wrong): 3 different origins -->
<link name="lidar_sensor_link">
  <pose>0 0 0.28 0 0 0</pose>
<sensor>
  <pose>0 0 0.03 0 0 0</pose>  <!-- 3cm offset -->

<!-- AFTER (correct): Single origin -->
<link name="mid360_link">
  <pose>0 0 0 0 0 0</pose>
<sensor name="mid360_lidar">
  <gz_frame_id>mid360_lidar_frame</gz_frame_id>
  <pose>0 0 0 0 0 0</pose>
```

### 7.2 IMU Frame

```xml
<link name="mid360_imu_frame">
  <pose>0 0 0 0 0 0</pose>  <!-- Same as LiDAR for MID-360 -->
</link>
<joint name="mid360_imu_joint" type="fixed">
  <parent>mid360_link</parent>
  <child>mid360_imu_frame</child>
</joint>
```

---

## 8. References

- [1] PX4 ECL EKF: https://docs.px4.io/main/en/advanced_config/estimators.html
- [2] FAST-LIO2: Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry"
- [3] Sophus: https://github.com/strasdat/Sophus
- [4] ROS REP-103: https://www.ros.org/reps/rep-0103.html

---

**Next:** Phase 2 - SDF model fixes
