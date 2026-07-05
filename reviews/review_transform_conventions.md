# Coordinate-Frame and Quaternion Convention Review

## 1. Quaternion Convention in Legacy Repository

### Order and Storage
The legacy repository (`/home/letandat/Dev/Mapping_and_Navigation_for_PX4_UAV/`) uses the Eigen library quaternion convention:
- **Eigen `Quaterniond` constructor order**: `w, x, y, z` — this is the order passed to `Eigen::Quaterniond(w, x, y, z)`.
- **Eigen internal storage (`coeffs()`)**: `x, y, z, w` — this is only relevant when accessing raw coefficients, not normal usage.
- **Storage arrays**: When converting to `std::array<float, 4>`, the order is `[w, x, y, z]` (PX4 convention)
- Function `eigen_quat_to_array()` explicitly stores as `[q.w(), q.x(), q.y(), q.z()]`

> ⚠️ **Critical**: Always distinguish between the Eigen constructor `(w, x, y, z)` and the internal `coeffs()` storage `[x, y, z, w]`. Mistaking these two orders is a common source of silent quaternion bugs.

### Passive vs Active Transformations
The legacy codebase uses passive transformations for coordinate frame conversions:
- `NED_ENU_Q` quaternion represents a passive rotation from NED to ENU frames
- Aircraft to base_link conversion uses a +π rotation around X axis

### Euler Angle Convention
- Euler angles are interpreted as ZYX (yaw, pitch, roll) sequence
- Function `quaternion_from_euler()` uses `Eigen::AngleAxisd` in order: Z, Y, X
- Function `quaternion_to_euler()` uses `eulerAngles(2, 1, 0).reverse()` which corresponds to ZYX

## 2. Quaternion Convention in New Repository

### Order and Storage
The new repository (`/home/letandat/Dev/uav-navigation/`) maintains consistency with the legacy approach:
- **Eigen `Quaterniond` constructor order**: `w, x, y, z`.
- **Eigen internal storage (`coeffs()`)**: `x, y, z, w`.
- Storage arrays: Explicit functions for conversion between Eigen quaternions and PX4 format arrays
- `EigenQuatToArray()` stores as `[q.w(), q.x(), q.y(), q.z()]` (PX4 wxyz order)
- `ArrayToEigenQuat()` converts from PX4 wxyz order to Eigen quaternion

### Passive vs Active Transformations
The new repository also uses passive transformations:
- `QuaternionEnuToNed()` and `QuaternionNedToEnu()` perform passive frame rotations
- The transformation is implemented as a conjugation: `q_rotation * q * q_rotation.conjugate()`
- The rotation quaternion represents a 180° rotation about the (1,1,0) axis

### Euler Angle Convention
- Euler angles follow the same ZYX convention as the legacy repository
- `EulerToQuaternion()` uses the same Eigen::AngleAxisd sequence: Z, Y, X
- `QuaternionToEuler()` uses the same conversion approach

## 3. Side-by-Side Comparison of Key Transform Functions

| Transform Function | Legacy Repository | New Repository | Notes |
|-------------------|-------------------|----------------|-------|
| **NED ↔ ENU Point** | `transform_static_frame(vec, StaticTF::NED_TO_ENU)` | `EnuToNed(const Eigen::Vector3d&)` | New repository uses direct coordinate swapping instead of matrix transforms |
| | Uses reflection matrices for efficiency | Simple coordinate swap: `(y, x, -z)` | New approach is more efficient |
| **NED ↔ ENU Quaternion** | `ned_to_enu_orientation(q)` | `QuaternionEnuToNed(q)` | Both perform the same passive transformation |
| | Uses precomputed `NED_ENU_Q` | Uses explicit rotation quaternion | Same mathematical result |
| **Body Frame Conversions** | `AIRCRAFT_TO_BASELINK` transforms | Not explicitly implemented | Legacy has aircraft(FRD) ↔ baselink(FLU) transforms |
| **Euler ↔ Quaternion** | `quaternion_from_euler()` | `EulerToQuaternion()` | Both use ZYX rotation order |
| | `quaternion_to_euler()` | `QuaternionToEuler()` | Same conversion approach |

### Detailed Function Comparison

**Point Transforms:**
- Legacy: Uses reflection matrices `NED_ENU_REFLECTION_XY` and `NED_ENU_REFLECTION_Z` for NED↔ENU
- New: Direct coordinate swapping `return Eigen::Vector3d(enu.y(), enu.x(), -enu.z())`

**Quaternion Transforms:**
- Legacy: `NED_ENU_Q * q` for orientation transformation
- New: `q_rotation * q_enu * q_rotation.conjugate()` with explicit rotation quaternion

**Euler Conversions:**
- Legacy: `Eigen::AngleAxisd(euler.z(), Z) * Eigen::AngleAxisd(euler.y(), Y) * Eigen::AngleAxisd(euler.x(), X)`
- New: Same implementation but in `px4_common::math` namespace

## 4. Potential Mismatches and Ambiguities

### 1. Coordinate System Definitions
- **Legacy**: Aircraft frame = Forward-Right-Down, base_link = Forward-Left-Up
- **New**: The aircraft to base_link conversion is not explicitly implemented in the new repository

### 2. Quaternion Storage Conventions
Both repositories use the same convention:
- Eigen internal: x, y, z, w
- PX4 storage: w, x, y, z
- This consistency prevents mismatches

### 3. Euler Angle Sequences
Both use ZYX (yaw-pitch-roll) sequence, ensuring compatibility.

### 4. Transform Direction Ambiguities
- Legacy: Uses enum `StaticTF` with values like `NED_TO_ENU`
- New: Uses function names like `EnuToNed()` which might be confusing (function name vs. actual transform)

### 5. Missing Functionality
The new repository is missing some of the body frame conversion functions that exist in the legacy repository:
- Aircraft to base_link transforms
- ECEF to ENU transforms with map origin support

## 5. Recommendations

### Convention Adoption
1. **Adopt the new repository's approach** for point transforms as it's more efficient:
   - Direct coordinate swapping instead of matrix multiplication
   - Clearer function names (`EnuToNed()` vs. `transform_static_frame(vec, StaticTF::NED_TO_ENU)`)

2. **Maintain the same quaternion storage convention**:
   - Continue using Eigen's internal x,y,z,w order
   - Continue using PX4's w,x,y,z storage format
   - This ensures compatibility with existing PX4 systems

### Helper Functions and Naming
1. **Add missing body frame conversion functions**:
   ```cpp
   // Add to new repository
   inline Eigen::Quaterniond QuaternionAircraftToBaselink(const Eigen::Quaterniond& q_aircraft);
   inline Eigen::Quaterniond QuaternionBaselinkToAircraft(const Eigen::Quaterniond& q_baselink);
   ```

2. **Standardize function naming**:
   - Use consistent naming like `SourceTypeToDestinationType()` 
   - Avoid confusion between function names and actual transform direction

3. **Add ECEF transformation support**:
   - Implement the ECEF to ENU transforms that exist in the legacy repository
   - Include map origin support for geographic coordinate conversions

### Documentation Updates
1. **Document the coordinate frame conventions clearly**:
   - NED: North-East-Down (PX4 default)
   - ENU: East-North-Up (ROS default)
   - Aircraft: Forward-Right-Down
   - base_link: Forward-Left-Up (ROS REP-103)

2. **Clarify passive vs active transformations**:
   - Specify that all transformations are passive (frame conversions)
   - Document what each quaternion transformation represents

3. **Add explicit notes about Euler angle conventions**:
   - ZYX rotation order (yaw, pitch, roll)
   - Intrinsic vs extrinsic rotations

### Unit Tests to Add
1. **Body frame conversion tests**:
   ```cpp
   TEST(FrameTransforms, AircraftToBaselinkRoundTrip) {
       // Test round-trip conversion
   }
   ```

2. **ECEF transformation tests**:
   ```cpp
   TEST(FrameTransforms, EcefToEnuTransform) {
       // Test ECEF to ENU conversions with map origin
   }
   ```

3. **Edge case tests**:
   - Test with zero quaternions
   - Test with identity quaternions
   - Test with pure yaw/pitch/roll quaternions

4. **Performance comparison tests**:
   - Benchmark the new direct coordinate swapping vs. legacy matrix approach
   - Validate that results are numerically equivalent

### Integration Recommendations
1. **Maintain API compatibility** where possible with legacy code
2. **Provide clear migration path** for users moving from legacy to new repository
3. **Add deprecation warnings** for any legacy functions that are replaced
4. **Ensure consistent error handling** across all transform functions

This analysis shows that the new repository has a cleaner, more efficient implementation while maintaining compatibility with the legacy quaternion and coordinate conventions.