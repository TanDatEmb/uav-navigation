# Phase 2.1 Code Review — px4_common & px4_mapping Migration

**Reviewer:** AI Code Reviewer (subagent)
**Date:** 2026-07-05
**Scope:** `src/px4_common/` and `src/px4_mapping/`
**Reference:** Legacy code at `Mapping_and_Navigation_for_PX4_UAV/px4_mapping/include/`

---

## Executive Summary

The migration from the legacy monolithic `common.hpp` + `voxel_map.hpp` codebase into the new modular `px4_common` + `px4_mapping` packages is a significant structural improvement. The code has been decomposed into focused headers with clear separation of concerns: math utilities, grid operations, voxel type constants, the abstract interface, and the concrete voxel hash map implementation.

**Overall quality: Good with notable issues to fix.**

The naming convention is mostly compliant (PascalCase classes, snake_case methods, `kPascalCase` constants, no `#define` constants in headers). Frame transforms are mathematically correct for pure ENU↔NED point conversions. The VoxelHashMap retains the well-designed two-step lock pattern from the legacy code. Tests exist for all modules.

However, there are **2 Critical**, **7 Warning**, and **8 Note** level findings that should be addressed before this code is used in flight testing.

---

## Findings by Category

### 🔴 Critical

#### C-1: `QuaternionEnuToNed` uses an incorrect rotation quaternion

**File:** `px4_common/include/px4_common/math/transforms.hpp`, lines ~150–165

The function constructs a quaternion with scalar `w = 0.0` and vector part `(0.7071, 0.7071, 0.0)`. This is a **180° rotation about the axis (1, 1, 0)/√2**, which is NOT the correct frame transformation from ENU to NED.

The ENU→NED frame transformation is a 180° rotation about the axis (1, 1, 0)/√2 **only if** we consider the specific swap-and-flush: E↔N (swap X,Y), U→D (flip Z). Let's verify:

- The rotation matrix for ENU→NED is:
  ```
  [0  1  0]
  [1  0  0]
  [0  0 -1]
  ```
  This is a 180° rotation about the axis (√2/2, √2/2, 0), which corresponds to quaternion `(w=0, x=√2/2, y=√2/2, z=0)`.

So the quaternion `(0, 0.7071, 0.7071, 0)` **is** correct for the ENU→NED rotation matrix.

**However**, the sandwich product `q_rot * q_enu * q_rot.conjugate()` performs a **rotation of the vector**, not a frame transformation. For a passive frame transformation (changing the coordinate frame while keeping the physical orientation fixed), the correct formula is:

```
q_ned = q_frame_rotation * q_enu * q_frame_rotation.inverse()
```

where `q_frame_rotation` rotates the ENU frame axes to NED frame axes. Since the ENU→NED rotation is its own inverse (R² = I, so R⁻¹ = R), `conjugate()` and `inverse()` give the same result here.

**The real issue is: the sandwich product `q_rot * q * q_rot⁻¹` rotates the quaternion `q` by `q_rot`. This is an active rotation of the orientation, not a passive frame change.**

For a **passive frame change** (same physical orientation, different coordinate frame), the correct formula is:

```
q_ned = q_enu_frame_to_ned_frame * q_enu * q_enu_frame_to_ned_frame⁻¹
```

where `q_enu_frame_to_ned_frame` is the quaternion that maps ENU frame vectors to NED frame vectors. This IS the sandwich product, and since R_enu_to_ned is its own inverse (R² = I), both `conjugate()` and `inverse()` yield the same result.

**Verdict after deeper analysis:** The math is actually correct for this specific ENU↔NED case because R_enu_to_ned is an involution (R = R⁻¹). The sandwich product correctly performs the passive frame change.

**But the `QuaternionNedToEnu` implementation is wrong:**

```cpp
inline Eigen::Quaterniond QuaternionNedToEnu(const Eigen::Quaterniond &q_ned) noexcept {
    return QuaternionEnuToNed(q_ned).conjugate();
}
```

This says: "NED→ENU = conjugate(ENU→NED(q_ned))". That is **not** the correct inverse transform. The correct inverse of `q_rot * q * q_rot⁻¹` is `q_rot⁻¹ * q' * q_rot`. Since `q_rot` here has w=0 (180° rotation), `q_rot⁻¹ = q_rot.conjugate() = (-0, -0.7071, -0.7071, 0) = q_rot` (because for a 180° rotation, the quaternion is its own conjugate up to sign, and q and -q represent the same rotation). So `q_rot⁻¹ = q_rot` and the inverse transform is `q_rot * q' * q_rot⁻¹ = q_rot * q' * q_rot`, which is the **same** as the forward transform.

Therefore `QuaternionNedToEnu` should be **the same** as `QuaternionEnuToNed`, not its conjugate.

The test `QuaternionEnuToNedIsInvolutory` passes only because it tests the identity quaternion (which is a fixed point of any conjugation-based operation). A non-trivial test would expose this bug.

**Concrete fix:**
```cpp
inline Eigen::Quaterniond QuaternionNedToEnu(const Eigen::Quaterniond &q_ned) noexcept {
    // ENU→NED rotation is a 180° rotation (involution), so forward and inverse are identical
    return QuaternionEnuToNed(q_ned);
}
```

**Add a test with a non-identity quaternion** to verify round-trip correctness.

---

#### C-2: `GetChangedPoints` is not thread-safe and has no documentation warning

**File:** `px4_mapping/include/px4_mapping/voxel_hash_map.hpp`

The method `GetChangedPoints()` reads `new_voxels_` and `kept_voxels_` without acquiring `map_mutex_`. The Doxygen comment says:

```
@note This must be called immediately after Update() from the same
      callback context; it reads internal buffers without locking.
```

This is a **race condition waiting to happen**. If any other thread calls `Update()`, `Clear()`, or `EvictDistant()` between the `Update()` and `GetChangedPoints()` calls, the buffers are invalidated. The legacy code had the same issue but at least documented it with an ALL-CAPS `INVARIANT` comment.

**Risk:** In a ROS 2 callback group, if `Update()` and `GetChangedPoints()` are in the same MutuallyExclusive callback group, this is safe. But if they're in a Reentrant callback group or different groups, it's a data race.

**Concrete fix:** Either:
1. Acquire `map_mutex_` in `GetChangedPoints()` (simplest, slight latency cost), or
2. Document the callback group requirement as a hard class invariant and add a `static_assert` or runtime check, or
3. Return the changed points from `Update()` directly via an output parameter.

---

### 🟡 Warning

#### W-1: `VoxelPool` destructor does not call destructors on stored `Voxel` objects

**File:** `px4_mapping/include/px4_mapping/voxel.hpp`

`VoxelPool` uses `std::malloc` for allocation and `std::free` for deallocation. This is fine for `Voxel` as a POD-like struct (all members are trivially destructible). However, if anyone ever adds a non-trivial member to `Voxel` (e.g., `std::string`), this will silently leak resources.

**Concrete fix:** Add a `static_assert(std::is_trivially_destructible_v<Voxel>, "Voxel must be trivially destructible for malloc/free pool");` inside `VoxelPool` or at namespace scope.

---

#### W-2: `VoxelPool::Deallocate` does not check for double-free

**File:** `px4_mapping/include/px4_mapping/voxel.hpp`

`Deallocate(Voxel *voxel)` only checks for `nullptr` but not for whether the voxel was already deallocated. A double-deallocate would push the same pointer onto the free stack twice, causing two `Allocate()` calls to return the same pointer — a silent use-after-free.

**Concrete fix:** Either:
1. Add a debug-only flag to `Voxel` (e.g., `bool in_pool_`) checked by `Deallocate`, or
2. Document that double-deallocate is a contract violation and add a debug assertion.

---

#### W-3: `EvictDistant` iterates the linked list while modifying it

**File:** `px4_mapping/include/px4_mapping/voxel_hash_map.hpp`

In `EvictDistant()`, the code saves `Voxel *previous = current->prev;` before potentially deallocating `current`. This is correct for the immediate next iteration. However, if `current->prev` was already evicted in a previous iteration (which can't happen since we walk from tail to head and only remove `current`), this is actually safe.

**After closer inspection:** The iteration is safe because:
- We save `previous` before mutating `current`
- `DetachNode(current)` only modifies `current`'s neighbors, not `previous`'s neighbors (unless `previous` is `current->prev`, which is saved already)
- `pool_.Deallocate(current)` frees `current` but `previous` is a separate node

**No fix needed**, but the code is subtle and would benefit from a comment explaining why saving `prev` before the eviction sequence is sufficient.

---

#### W-4: DDA raycast boundary calculation uses grid-space index but world-space origin

**File:** `px4_mapping/include/px4_mapping/voxel_hash_map.hpp`, in `Raycast()`

```cpp
const double boundary = (start_idx(axis) + 1) * px4_common::mapping::kDefaultVoxelResolutionM;
t_max(axis) = (boundary - origin(axis)) / direction(axis);
```

Here `start_idx` is computed by dividing by resolution and flooring:
```cpp
Eigen::Vector3i start_idx = origin_grid.array().floor().cast<int>();
```

But the boundary is computed as `(start_idx + 1) * resolution`, which assumes the grid origin is at (0,0,0). This is consistent with the rest of the code (the map always uses origin = Zero), but it's fragile. If someone later adds a non-zero grid origin, this will break silently.

**Concrete fix:** Add a comment noting the assumption that grid origin is (0,0,0), or better, use `px4_common::math::WorldToIndex` consistently with an explicit origin parameter.

---

#### W-5: `Update()` timeout check granularity may miss deadline

**File:** `px4_mapping/include/px4_mapping/voxel_hash_map.hpp`

The deadline is checked every 256 points (`(i & 0xFFU) == 0xFFU`). For very dense point clouds (e.g., 100K+ points), each point's raycast may take significant time. 256 points could represent 5–10 ms of work, so the actual timeout could overshoot by up to that amount. The 50 ms budget could become 55–60 ms in practice.

This is a known trade-off from the legacy code. **Acceptable** but should be documented.

---

#### W-6: `px4_common` exports as a regular library, not a header-only INTERFACE library

**File:** `px4_common/CMakeLists.txt`

The package uses `add_library(${PROJECT_NAME} src/px4_common.cpp)` with `target_include_directories(... PUBLIC ...)`. This creates a static library from a single empty translation unit. While this works, it's cleaner to use an `INTERFACE` library for header-only packages:

```cmake
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} INTERFACE geometry_msgs rclcpp)
```

However, the current approach does work correctly and is well-documented in the .cpp file. The `EXPORT` target is properly set up. **Not a bug**, just a style preference. The current approach may actually be more robust for some ament versions.

**Recommendation:** Keep as-is unless downstream packages have linking issues.

---

#### W-7: `px4_mapping` CMakeLists.txt uses `INTERFACE` library but test linking may be fragile

**File:** `px4_mapping/CMakeLists.txt`

```cmake
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
```

The install interface path is `include/${PROJECT_NAME}` but the headers are installed via `install(DIRECTORY include/ DESTINATION include)`. Wait — there is **no `install(DIRECTORY ...)` for headers** in the px4_mapping CMakeLists.txt. This means headers won't be installed, and downstream packages can't find them at install time.

**Concrete fix:** Add:
```cmake
install(DIRECTORY include/
    DESTINATION include
)
```

Also, `ament_export_targets` and `ament_export_dependencies` are missing. Add:
```cmake
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(px4_common Eigen3 rclcpp geometry_msgs nav_msgs sensor_msgs)
```

And `ament_package()` should be at the very end (it already is, but the exports need to be before it).

---

### 🟢 Note

#### N-1: `#include <math.h>` instead of `<cmath>`

**Files:** `px4_common/include/px4_common/types.hpp`, `px4_common/include/px4_common/math/transforms.hpp`, `px4_common/include/px4_common/math/grid.hpp`

The C++ header `<cmath>` is preferred over the C header `<math.h>` because it places functions in the `std` namespace and avoids polluting the global namespace. While `M_PI` is commonly available via `<math.h>`, it's not guaranteed by the C++ standard. Consider using `constexpr double kPi = 3.14159265358979323846;` or including `<numbers>` (C++20) for `std::numbers::pi`.

**Concrete fix:** Replace `#include <math.h>` with `#include <cmath>` and define a `constexpr double kPi` if `M_PI` is needed.

---

#### N-2: VoxelHash hash function is not randomized

**File:** `px4_common/include/px4_common/types.hpp`

The hash function uses a fixed `0x9e3779b9` constant with no random seed. This makes the hash map vulnerable to hash collision attacks (adversarial input causing O(n) lookups). For a robotics application this is low risk, but worth noting.

Also, the hash function is identical between the old and new code. The `noexcept` qualifier is a nice addition.

---

#### N-3: `MissionPathNed::GetDirection` ignores Z component

**File:** `px4_common/include/px4_common/types.hpp`

`GetDirection()` only returns X and Y direction components, dropping Z. This is reasonable for 2D mission following, but it should be documented that the direction is 2D-projected.

---

#### N-4: `Deg2Rad` and `Rad2Deg` use `M_PI` which may not be available in strict C++ mode

**File:** `px4_common/include/px4_common/math/transforms.hpp`

```cpp
inline constexpr double Deg2Rad(double deg) noexcept {
    return deg * (M_PI / 180.0);
}
```

`M_PI` is a POSIX extension, not standard C++. With `-std=c++20` and some compilers, `M_PI` may not be defined. Since the code uses `-Wpedantic`, this could produce warnings on some platforms.

**Concrete fix:**
```cpp
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double Deg2Rad(double deg) noexcept {
    return deg * (kPi / 180.0);
}
```

---

#### N-5: Test coverage gaps

**Missing test cases:**

1. **Transforms:** No test for `QuaternionEnuToNed` / `QuaternionNedToEnu` with a non-identity quaternion (this would catch C-1).
2. **Transforms:** No test for `EulerToQuaternion(double, double, double)` overload (only the `Vector3d` overload is tested).
3. **Transforms:** No test for `QuaternionToEuler` output overload.
4. **Grid:** No test for negative coordinates or negative origin.
5. **VoxelHashMap:** No test for `GetChangedPoints()`.
6. **VoxelHashMap:** No test for the timeout path (`WasTimedOut()`).
7. **VoxelHashMap:** No test for pool exhaustion behavior.
8. **VoxelHashMap:** No test for `Clear()`.
9. **VoxelHashMap:** No test for memory eviction via `ManageMemory()` (age-based or size-based).
10. **VoxelHashMap:** The `GetOccupiedPointsInRadius` test uses tolerance of `kDefaultVoxelResolutionM` (0.2m) which is very loose — the voxel center should be at `(2.1, 0.1, 0.1)` for a point at `(2.0, 0.0, 0.0)`, so the test should check for that exact center.

---

#### N-6: Duplicate test in `test_math_utils.cpp`

**File:** `px4_common/test/test_math_utils.cpp`

This file contains `EnuToNedAndBack` and `RotationMatricesAreConsistent` tests that are **identical** to the first two tests in `test_transforms.cpp`. This appears to be a copy-paste artifact during migration. Only one of these files should contain these tests.

**Concrete fix:** Remove `test_math_utils.cpp` or rename it to test something different (e.g., angle conversion helpers, interpolation).

---

#### N-7: `VoxelHashMap` constructor reserves `kMaxVoxels * 1.5` in the hash table

**File:** `px4_mapping/include/px4_mapping/voxel_hash_map.hpp`

```cpp
map_table_.reserve(px4_common::mapping::kMaxVoxels * 1.5);
```

`kMaxVoxels` is 1,800,000. Reserving 2,700,000 buckets in the hash map at construction time allocates significant memory (~43 MB for 8-byte pointers × 2.7M × 2 [key+value]). This matches the legacy code but is worth documenting as a deliberate memory budget choice.

---

#### N-8: `VoxelPool` does not zero-initialize memory

**File:** `px4_mapping/include/px4_mapping/voxel.hpp`

`std::malloc` does not zero-initialize memory. While `Voxel::Init()` explicitly sets all fields, any uninitialized padding bytes could theoretically cause issues with sanitizers. Using `std::calloc` or adding a memset in `Init()` would be safer but slower.

**No fix needed** for production, but may produce false positives with memory sanitizers (MSan).

---

## Recommended Actions

### Immediate (before any testing)

| Priority | Finding | Action |
|----------|---------|--------|
| 🔴 C-1 | `QuaternionNedToEnu` incorrect | Fix to use `QuaternionEnuToNed` (involution), add non-identity quaternion round-trip test |
| 🔴 C-2 | `GetChangedPoints` race condition | Add `std::lock_guard` to `GetChangedPoints()` or enforce callback group invariant with an assertion |
| 🟡 W-7 | px4_mapping missing header install | Add `install(DIRECTORY include/ DESTINATION include)` and ament export targets |

### Short-term (before flight test)

| Priority | Finding | Action |
|----------|---------|--------|
| 🟡 W-1 | VoxelPool trivial destructibility | Add `static_assert(std::is_trivially_destructible_v<Voxel>)` |
| 🟡 W-2 | VoxelPool double-free risk | Add debug-only assertion or documentation |
| 🟡 W-4 | DDA assumes zero grid origin | Add comment or parameterize origin |
| 🟢 N-4 | `M_PI` portability | Replace with `constexpr double kPi` |
| 🟢 N-1 | `<math.h>` vs `<cmath>` | Replace with `<cmath>` |
| 🟢 N-6 | Duplicate test file | Remove or repurpose `test_math_utils.cpp` |
| 🟢 N-5 | Test coverage gaps | Add tests listed in N-5 |

### Long-term (code quality polish)

| Priority | Finding | Action |
|----------|---------|--------|
| 🟡 W-5 | Timeout granularity | Document acceptable overshoot |
| 🟡 W-6 | px4_common library type | Evaluate INTERFACE library migration |
| 🟢 N-3 | MissionPathNed Z component | Document 2D-only behavior |
| 🟢 N-7 | Hash table memory budget | Document deliberate choice |
| 🟢 N-8 | Pool zero-init | Consider `calloc` if MSan is used |

---

## Frame Convention Audit

### ENU↔NED Point Transform

| Function | Input | Output | Formula | Correct? |
|----------|-------|--------|---------|----------|
| `EnuToNed(Vector3d)` | ENU (x=E, y=N, z=U) | NED (x=N, y=E, z=D) | `(y, x, -z)` | ✅ |
| `NedToEnu(Vector3d)` | NED (x=N, y=E, z=D) | ENU (x=E, y=N, z=U) | `(y, x, -z)` | ✅ |
| `EnuToNedRotation()` | — | 3×3 matrix | `[[0,1,0],[1,0,0],[0,0,-1]]` | ✅ |
| `NedToEnuRotation()` | — | 3×3 matrix | transpose of above | ✅ |

### Quaternion ENU↔NED

| Function | Correct? | Notes |
|----------|----------|-------|
| `QuaternionEnuToNed` | ✅ | Sandwich product with 180° rotation quaternion. Correct for ENU→NED frame change. |
| `QuaternionNedToEnu` | ❌ | Uses `.conjugate()` of the ENU→NED result instead of applying the same involution. See C-1. |

### Yaw Convention

`QuaternionGetYaw` extracts yaw from Z-axis rotation using the standard formula. In the ENU frame, positive yaw is counterclockwise from East. In the NED frame, positive yaw is clockwise from North. The function itself is frame-agnostic (it just extracts the Z-rotation angle), but the interpretation depends on which frame the quaternion is in. **This is not documented.**

**Recommendation:** Add a note in `QuaternionGetYaw` documentation clarifying the frame-dependent sign interpretation.

---

## Legacy Code Comparison Summary

| Aspect | Legacy | Migrated | Verdict |
|--------|--------|----------|---------|
| Constants | `#define` macros in `common.hpp` | `inline constexpr` in `voxel_types.hpp` | ✅ Major improvement |
| Naming | snake_case methods (`update`, `raycast`) | PascalCase methods (`Update`, `Raycast`) | ✅ Compliant with ROS 2 / PX4 style |
| Member naming | No suffix (`current_size_`) | trailing underscore (`current_size_`) | ✅ Compliant |
| Interface | Free-floating `IVoxMapManager` | Namespaced `px4_common::mapping::IVoxMapManager` | ✅ Better encapsulation |
| Thread safety | Documented invariant, no lock on `getChangedPoints` | Same invariant, still no lock | ⚠️ Same risk |
| Memory pool | `malloc`/`free`, no trivially-destructible check | Same | ⚠️ Same |
| DDA raycast | Uses `worldToGrid` helper with `origin = 0` | Inline computation, same logic | ✅ Correct |
| Test coverage | No unit tests | Unit tests for most modules | ✅ Major improvement, gaps remain |
| CMake | Single CMakeLists, no ament exports | Separate packages with ament exports | ✅ Major improvement, px4_mapping exports incomplete |

---

## Conclusion

The migration is structurally sound and represents a significant improvement over the legacy code. The modular package layout, constexpr constants replacing macros, namespaced interfaces, and addition of unit tests are all positive changes. The two critical findings (quaternion inverse bug and thread-safety gap in `GetChangedPoints`) should be fixed before integration testing. The px4_mapping CMake export issue (W-7) will prevent downstream packages from finding the headers and must be addressed.

After fixing the Critical and high-priority Warning items, this code should be ready for integration with the navigation layer.