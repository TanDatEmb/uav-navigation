# IESKF Design

This document describes the Iterated Error-State Kalman Filter (IESKF) implemented
by `fast_lio`. The filter uses a 15-DOF state with fixed world gravity, a right
perturbation on SO(3), and an information-form iterative LiDAR update.

---

## State Vector

The state is defined on the manifold SO(3) × ℝ¹²:

```
x = [R_wb, p_w, v_w, b_a, b_ω]
```

| Component | Description | Index |
|-----------|-------------|-------|
| `R_wb` | Body-to-world rotation | 0-2 |
| `p_w` | Position in world frame | 3-5 |
| `v_w` | Velocity in world frame | 6-8 |
| `b_a` | Accelerometer bias | 9-11 |
| `b_ω` | Gyroscope bias | 12-14 |

**Excluded from the 15-DOF error state:**

- **Gravity:** constant `g = [0, 0, -9.81] m/s²` in the Z-up LIO world.
- **LiDAR-IMU extrinsic:** stored with the state container as `T_imu_lidar`, but
  loaded from calibration and not estimated by the filter.

**Right perturbation**: The rotation update uses `R ← R * exp(δθ)` to match the point-to-plane Jacobian convention.

---

## Coordinate Frames

**LIO world:**

- gravity-aligned and Z-up, with `g = [0, 0, -9.81] m/s²`
- local origin and arbitrary initial yaw; it is not automatically north-aligned

**IMU body:** sensor measurements and biases are expressed in this frame.

**LiDAR:** points use the fixed extrinsic
`T_world_lidar = T_world_imu * T_imu_lidar`.

---

## Prediction Step (IMU Propagation)

Bias-corrected measurements: `a_unbiased = a_imu - b_a`, `ω_unbiased = ω_imu - b_ω`

State propagation with midpoint integration:
```
R_wb ← R_wb * exp(ω_unbiased * Δt)
a_world ← R_wb * a_unbiased + g
p_w ← p_w + v_w * Δt + 0.5 * a_world * Δt²
v_w ← v_w + a_world * Δt
```

Biases follow random walk (no deterministic propagation).

### Error-State Transition

```
Φ = I + F * Δt

F[0:3, 0:3] = -[ω_unbiased]×
F[0:3, 12:15] = -I
F[3:6, 6:9] = I
F[6:9, 0:3] = -R_wb * [a_unbiased]×
F[6:9, 9:12] = -R_wb
```

### Process Noise

```
Q = diag(σ_ω²Δt, 0, σ_a²Δt, σ_ba²Δt, σ_bω²Δt)
```

The active parameter profile uses:

- `σ_ω = 0.001 rad/s/√Hz`, `σ_a = 0.01 m/s²/√Hz`
- `σ_bω = 0.00001 rad/s/√Hz`, `σ_ba = 0.0001 m/s²/√Hz`

---

## Update Step (Information-Form IESKF)

The update uses **information form** (15×15 factorization) instead of measurement-space (N×N), avoiding cubic scaling with point count.

**Prior information**: `Λ_prior = P⁻¹`

Per iteration:
1. Evaluate loss function at current iterate
2. Compute: `Λ = Λ_prior + λ * HᵀH`
3. Solve: `δx = Λ⁻¹ * (-λ * Hᵀ * residual - Λ_prior * accumulated_delta)`
4. Update state and check convergence

**Convergence:** `‖δθ‖ < 0.01°` and `‖δp‖ < 0.015 m`

**Final covariance:** `P_posterior = (Λ_prior + λ * H_finalᵀH_final)⁻¹`

State and covariance commit atomically—if final linearization fails, neither updates.

---

## Point-to-Plane Constraint

### Measurement Model

For LiDAR point `p_lidar`:
```
p_world = R_wb * R_il * p_lidar + R_wb * t_il + p_w
r = nᵀ * (p_world - q_plane)
```

Plane `(n, q_plane)` found via KNN search (default 5 neighbors).

### Jacobian

`H = [H_θ, H_p, 0, 0, 0]` where:
```
p_imu = R_il * p_lidar + t_il
H_θ = -nᵀ * R_wb * [p_imu]×
H_p = nᵀ
```

Velocity and biases have zero Jacobian.

### Plane Validation

A correspondence is accepted only when:

- at least five neighbors are returned
- the farthest returned squared distance is at most `1.0 m²`
- the smallest covariance eigenvalue is at most `0.01`

The normal is flipped when necessary so `n · q_plane >= 0`; this makes its sign
consistent but is not an additional rejection criterion. The implementation uses
an absolute eigenvalue threshold, not a normalized eigenvalue ratio.

---

## Configuration

| Parameter | Active value | Description |
|-----------|--------------|-------------|
| `na/ng` | 0.01/0.001 | Accelerometer/gyroscope noise |
| `nba/nbg` | 0.0001/0.00001 | Bias random walk |
| `lidar_cov_inv` | 200.0 | LiDAR information weight `λ` |
| `ieskf_max_iter` | 1 | Maximum re-linearizations in the active profile |
| `near_search_num` | 5 | Requested KNN neighbors; minimum valid value is 5 |
| `imu_init_num` | 50 | Stationary-window samples |
| `imu_init_accel_std_max` | 0.5 m/s² | Maximum acceleration standard deviation |
| `imu_init_gyro_rms_max` | 0.1 rad/s | Maximum gyroscope RMS |
| `imu_init_gravity_tolerance` | 3.0 m/s² | Allowed mean-acceleration norm error |
| `gravity_align` | true | Align gravity at startup |

---

## Initialization

**IMU initialization:**
1. Collect `imu_init_num` samples
2. Validate: `std(accel) < threshold`, `rms(gyro) < threshold`, `|‖mean_acc‖ - 9.81| < tolerance`
3. Apply scale: `scale = 9.81 / ‖mean_acc‖`

**Gravity alignment:**
```
R_wb = rotation_aligning(mean_acc/‖mean_acc‖ → [0,0,1])
```

Handles parallel, anti-parallel, and degenerate inputs safely.

---

## Invariants and Failure Behavior

**Safety checks:**
- Prediction rejected if `dt ≤ 0` or `dt > 0.1s`
- Gravity alignment rejected if `‖mean_acc‖ < 1e-6` or non-finite
- Covariance diagonal clamped to `[1e-12, 1e6]` after prediction
- Update aborted if LDLT factorization fails

**Failure recovery:**
- Failed update: No state/covariance change
- Failed final linearization: Atomic abort
- Invalid measurements: `shared.valid = false` skips update

---

## Tests and Limitations

**Key unit tests:**
- Large measurement update (20k points) verifies O(N) scaling
- Atomic commit on final linearization failure
- Configuration controls measurement weight and process noise
- Gravity alignment handles anti-parallel/degenerate inputs
- IMU propagation across zero-duration scans
- Sliding cube removes only departed slabs

**Known limitations:**

1. Gravity magnitude and direction are not estimated after initialization.
2. The static extrinsic is not estimated online; calibration errors propagate.
3. The LiDAR update is point-to-plane only, with no unstructured fallback.
4. The map has no loop closure or multi-session correction.
5. Distance and planarity thresholds are fixed in the implementation.
6. The active profile performs one LiDAR re-linearization per scan; any increase
   requires controlled timing and map-quality validation.

**Complexity characteristics:**

- the information solve is a fixed 15×15 LDLT factorization
- forming `HᵀH` is linear in the accepted correspondence count
- covariance storage is fixed 15×15; measurement buffers are dynamic
