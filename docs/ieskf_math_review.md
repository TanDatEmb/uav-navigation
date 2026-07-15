# IESKF Mathematical Review for FAST-LIO2 UAV Implementation

**Document**: IESKF Mathematics & Implementation Review
**Target**: UAV Platform with MID-360 LiDAR
**Update Rate**: 10-20 Hz (real-time constraint)
**Date**: 2026-07-14
**Reviewer**: State Estimation Expert (SE(3), Lie Algebra)

---

## Executive Summary

Đánh giá IESKF cho FAST-LIO2 trên UAV platform, tập trung vào state representation tối ưu, correctness của error-state propagation, và point-to-plane Jacobian computation. Đề xuất reduced 15-DOF state thay vì full 18-DOF cho UAV đã align gravity.

---

## 1. State Representation Analysis

### 1.1 Full State Vector Options

#### Option A: Full 21-DOF (Academic/Research)
```
X_full = [R (SO3), p (R³), v (R³), b_a (R³), b_ω (R³), g (R³), T_Lidar (SE3)]
       = 3 + 3 + 3 + 3 + 3 + 3 + 6 = 24 DOF (bao gồm LiDAR extrinsic)
```

**Nhận xét:**
- Quá nhiều DOF cho real-time UAV
- Gravity estimation không cần thiết nếu đã align
- LiDAR extrinsic nên calibrate offline

#### Option B: Standard 18-DOF (FAST-LIO2 Original)
```
X_18 = [R ∈ SO(3), p ∈ ℝ³, v ∈ ℝ³, b_a ∈ ℝ³, b_ω ∈ ℝ³, g ∈ ℝ³]
      = 3 + 3 + 3 + 3 + 3 + 3 = 18 DOF
```

**Nhận xét:**
- Phù hợp cho ground robot (gravity hướng theo Z-up)
- UAV: gravity đã biết (9.81 m/s², Z-down trong NED frame)

#### ✅ Option C: Reduced 15-DOF (Recommended for UAV)
```
X_15 = [R ∈ SO(3), p ∈ ℝ³, v ∈ ℝ³, b_a ∈ ℝ³, b_ω ∈ ℝ³]
      = 3 + 3 + 3 + 3 + 3 = 15 DOF
```

**Lý do:**
1. **Gravity known**: UAV sử dụng NED frame, g = [0, 0, 9.81]ᵀ (constant)
2. **3 DOF saved**: Giảm 16.7% state dimension → faster computation
3. **No gravity drift**: Không cần estimate, không có drift
4. **MID-360**: IMU-built-in, đã align với LiDAR coordinate

### 1.2 Error-State Representation

#### SO(3) Error State
Trên manifold SO(3), error state được định nghĩa:

```
True rotation:     R = R̂ · Exp(δθ) ≈ R̂ · (I + [δθ]×)
                    hoặc R = Exp(δθ) · R̂
```

**Left vs Right Perturbation:**

| Form | Definition | Jacobian Form | Use Case |
|------|------------|---------------|----------|
| **Left** | R = Exp(δθ) · R̂ | Jₗ(θ) = Jₗ(δθ) | EKF standard (recommended) |
| **Right** | R = R̂ · Exp(δθ) | Jᵣ(θ) = Jᵣ(δθ) | Alternative |

**Quy ước FAST-LIO2**: Sử dụng **Left perturbation**:
```
R ≈ (I + [δθ]×) · R̂
```

**Jacobian of rotation error (Left):**
```
∂(R·v)/∂δθ = -R̂·[v]×  (3×3 matrix)
```

#### Full Error-State Vector (15-DOF)
```
δx = [δθ ∈ ℝ³, δp ∈ ℝ³, δv ∈ ℝ³, δb_a ∈ ℝ³, δb_ω ∈ ℝ³] ∈ ℝ¹⁵
```

---

## 2. IMU Propagation

### 2.1 Error-State vs Full-State Propagation

#### ✅ Error-State EKF (Recommended)

**Advantages:**
- Linearization tốt hơn (error state nhỏ → approximation chính xác)
- SO(3) constraint được bảo toàn tự nhiên
- Numerical stability cao hơn
- Industry standard (MSCKF, OKVIS, VINS-Mono, FAST-LIO2)

**Continuous-time IMU Kinematics:**

```
Position:    ṗ = v
Velocity:    v̇ = R·(a_m - b_a) + g
Rotation:    Ṙ = R·[ω_m - b_ω]×
Bias accel:  ḃ_a = n_{b_a}  (random walk)
Bias gyro:   ḃ_ω = n_{b_ω}  (random walk)
```

**Error-State Dynamics:**

```
δθ̇ = -[ω_m - b̂_ω]× · δθ - δb_ω + n_ω
δṗ = δv
δv̇ = -R̂·[â]× · δθ - R̂·δb_a + n_a
δḃ_a = n_{b_a}
δḃ_ω = n_{b_ω}
```

Trong đó: â = a_m - b̂_a (bias-corrected acceleration)

### 2.2 State Transition Matrix F

```
F = [ F_θθ   0₃ₓ₃   0₃ₓ₃   0₃ₓ₃  -I₃ₓ₃ ]
    [  0₃ₓ₃   0₃ₓ₃   I₃ₓ₃   0₃ₓ₃   0₃ₓ₃ ]
    [ -R̂[â]×  0₃ₓ₃   0₃ₓ₃  -R̂     0₃ₓ₃ ]
    [  0₃ₓ₃   0₃ₓ₃   0₃ₓ₃   0₃ₓ₃   0₃ₓ₃ ]
    [  0₃ₓ₃   0₃ₓ₃   0₃ₓ₃   0₃ₓ₃   0₃ₓ₃ ]

(15×15 matrix)

Where:
  F_θθ = -[ω̂]×  (with ω̂ = ω_m - b̂_ω)
```

### 2.3 Noise Covariance Q

```
Q = diag(Q_θ, Q_p, Q_v, Q_{b_a}, Q_{b_ω})  (15×15)

Q_θ = σ²_{gyro} · Δt · I₃      (gyro noise)
Q_p = 0                          (position evolves deterministically)
Q_v = σ²_{accel} · Δt · I₃     (accel noise)
Q_{b_a} = σ²_{b_a} · Δt · I₃   (bias random walk)
Q_{b_ω} = σ²_{b_ω} · Δt · I₃   (bias random walk)
```

### 2.4 Discretization

#### ✅ First-Order Euler (Recommended for 200Hz IMU)
```
Φ ≈ I + F·Δt
Q_d = Φ·Q·Φᵀ·Δt  (or simplified: Q·Δt)
```

**Covariance Propagation:**
```
P_{k+1} = Φ·P_k·Φᵀ + Q_d
```

#### Alternative: Midpoint Integration (Higher Accuracy)
```
For high-dynamic UAV maneuvers, consider:
- RK4 for state propagation
- Truncated Taylor series for Φ
```

---

## 3. IESKF Iteration Mechanism

### 3.1 Why IESKF?

Standard EKF: Linearize tại prior → update once
IESKF: Iterate update để convergence → better accuracy

**Critical for LiDAR**: Point-to-plane constraint là **nonlinear**
→ IESKF cần thiết cho chính xác!

### 3.2 IESKF Algorithm

```
Input: x̂_prior, P_prior, measurements {z_i}
Output: x̂_post, P_post

1. Initialize: x̂⁽⁰⁾ = x̂_prior, P⁽⁰⁾ = P_prior
2. For iteration k = 1 to K_max:
   a. Re-linearize at x̂⁽ᵏ⁻¹⁾
   b. Stack all point-to-plane residuals: r = [r₁, r₂, ..., rₙ]ᵀ
   c. Compute Jacobians H = ∂r/∂δx tại x̂⁽ᵏ⁻¹⁾
   d. Compute Kalman gain: K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
   e. Update: δx = K·r
   f. Check convergence: ‖δx‖ < ε
   g. Update state: x̂⁽ᵏ⁾ = x̂⁽ᵏ⁻¹⁾ ⊕ δx  (manifold update)
3. Final covariance: P_post = (I - K·H)·P
```

### 3.3 Convergence Criteria

#### Recommended Settings for UAV + MID-360:

```cpp
// Convergence parameters
const int MAX_ITERATIONS = 3;        // 2-3 iterations usually sufficient
const double CONVERGENCE_EPS = 1e-3;  // Norm of state update
const double RESIDUAL_THRESH = 0.1;   // Point-to-plane residual threshold

// Early termination
if (delta_x.norm() < CONVERGENCE_EPS) break;
if (k > 0 && abs(prev_residual - curr_residual) < 1e-4) break;
```

**Timing Analysis:**
- IMU rate: 200 Hz (5ms)
- LiDAR scan: 10-20 Hz (50-100ms)
- IESKF iteration: ~1-2ms/iteration
- Total update: 3-6ms << 50ms → real-time feasible

### 3.4 Manifold Update (SO(3) Update)

**Critical**: Quaternion/Rotation phải được update đúng cách trên manifold!

```cpp
// WRONG (Euclidean update):
R_new = R_old + delta_R;  // ❌ Violates SO(3) constraint

// CORRECT (Manifold update):
// Left perturbation
R_new = SO3::exp(delta_theta) * R_old;  // ✓

// Or equivalently
R_new = R_old * SO3::exp(delta_theta);  // (depends on convention)
```

**Full State Update (15-DOF):**
```cpp
// delta_x = [delta_theta, delta_p, delta_v, delta_b_a, delta_b_omega]

R_hat = SO3::exp(delta_x(0:2)) * R_hat;   // SO(3) update
p_hat += delta_x(3:5);                     // Position (vector)
v_hat += delta_x(6:8);                     // Velocity (vector)
b_a_hat += delta_x(9:11);                  // Accel bias
b_omega_hat += delta_x(12:14);             // Gyro bias
```

---

## 4. Point-to-Plane Measurement Model

### 4.1 Point-to-Plane Error

LiDAR point `p_L` trong LiDAR frame → transform to world → project to local plane.

```
Measurement model:
  z = nᵀ · (p_world - q_plane)  = 0  (point on plane constraint)

Where:
  p_world = R_WL · p_L + t_WL   (LiDAR to World transform)
  n = plane normal (from map/kdtree)
  q_plane = point on plane
```

### 4.2 State Dependency

LiDAR-to-World transform phụ thuộc vào:
```
T_WL = T_WB · T_BL

Where:
  T_WB = [R (from IMU state), p (from IMU state)]
  T_BL = [R_BL, t_BL] (LiDAR-IMU extrinsic, calibrated)
```

### 4.3 ✅ Correct Jacobian Form

**Point transformation:**
```
p_world = R · R_BL · p_L + R · t_BL + p
        = R · (R_BL·p_L + t_BL) + p
        = R · p_L_body + p

where p_L_body = R_BL·p_L + t_BL (point in body frame)
```

**Residual:**
```
r = nᵀ · (p_world - q_plane)
  = nᵀ · (R · p_L_body + p - q_plane)
```

**Jacobian w.r.t. error state (Left perturbation):**

```
∂r/∂δθ = nᵀ · ∂(R·p_L_body)/∂δθ
       = nᵀ · (-R·[p_L_body]×)            (SO(3) left perturbation)
       = -nᵀ · R · [p_L_body]×

∂r/∂δp = nᵀ · I₃ = nᵀ                    (position)

∂r/∂δv = 0                                (velocity not in measurement)

∂r/∂δb_a = 0                              (accel bias not in measurement)

∂r/∂δb_ω = 0                              (gyro bias not in measurement)
```

**Full measurement Jacobian (1×15):**
```
H = [ -nᵀ·R·[p_L_body]× , nᵀ , 0 , 0 , 0 ]  (1×15)
     ← 3 →           ←3→ ←3→←3→←3→
      θ               p   v  b_a b_ω
```

**Compact form:**
```
H_θ = -nᵀ · R · [p_L_body]×   (1×3)
H_p = nᵀ                      (1×3)
H_v = 0                       (1×3)
H_{b_a} = 0                   (1×3)
H_{b_ω} = 0                   (1×3)
```

### 4.4 Stacked Measurements

Với N points, stack vertically:
```
H_full = [ H_θ₁  H_p₁  0  0  0 ]   (N×15)
         [ H_θ₂  H_p₂  0  0  0 ]
         [ ...              ... ]
         [ H_θₙ  H_pₙ  0  0  0 ]

r_full = [ r₁, r₂, ..., rₙ ]ᵀ      (N×1)
```

### 4.5 Information Selection (Critical!)

Không phải tất cả points đều tốt! FAST-LIO2 sử dụng:

```cpp
// 1. Distance threshold
if (point_to_plane_dist > threshold) reject;

// 2. Normal consistency
if (abs(n.dot(estimated_normal)) < 0.9) reject;

// 3. Residual check (after 1st iteration)
if (abs(residual) > 3*sigma) reject;  // outlier rejection

// 4. Feature selection (optional)
// Select points with maximum information gain
// → Reduce N while maintaining observability
```

---

## 5. Covariance & Numerical Stability

### 5.1 Covariance Initialization

```cpp
// Initial covariance for 15-DOF state
P = diag(P_theta, P_p, P_v, P_ba, P_bw);

P_theta = (5°)² * I₃      // orientation uncertainty
P_p = (0.1m)² * I₃        // position uncertainty
P_v = (0.1m/s)² * I₃      // velocity uncertainty
P_ba = (0.1m/s²)² * I₃    // accel bias uncertainty
P_bw = (0.01rad/s)² * I₃  // gyro bias uncertainty
```

### 5.2 Joseph Form Update (Numerical Stability)

```cpp
// Standard form (less stable):
P_post = (I - K*H) * P_prior;

// Joseph form (more stable):
P_post = (I - K*H) * P_prior * (I - K*H).transpose() + K*R*K.transpose();
```

### 5.3 Covariance Clamping

```cpp
// Prevent negative variances (numerical error)
P = P.cwiseMax(1e-12).cwiseMin(1e6);

// Symmetrize
P = 0.5 * (P + P.transpose());
```

---

## 6. Code Skeleton: IESKF Core

```cpp
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace fast_lio {

// Forward declarations
struct IMUData {
    double timestamp;
    Eigen::Vector3d accel;  // m/s²
    Eigen::Vector3d gyro;   // rad/s
};

struct PointData {
    Eigen::Vector3d point_lidar;  // in LiDAR frame
    Eigen::Vector3d plane_normal; // map normal
    Eigen::Vector3d plane_point;  // map point
    double residual;
    bool valid;
};

/**
 * @brief Iterated Error-State Kalman Filter (IESKF)
 * Optimized for UAV with 15-DOF state
 */
class IESKF {
public:
    // State: [R, p, v, b_a, b_ω] = 15 DOF
    struct State {
        Eigen::Matrix3d R;      // SO(3): body to world
        Eigen::Vector3d p;      // position in world
        Eigen::Vector3d v;      // velocity in world
        Eigen::Vector3d b_a;    // accel bias
        Eigen::Vector3d b_w;    // gyro bias

        double timestamp;

        // Manifold update
        void update(const Eigen::Matrix<double, 15, 1>& dx) {
            R = SO3Exp(dx.head<3>()) * R;  // Left perturbation
            p += dx.segment<3>(3);
            v += dx.segment<3>(6);
            b_a += dx.segment<3>(9);
            b_w += dx.segment<3>(12);
        }
    };

    // Extrinsic: LiDAR to IMU (calibrated offline)
    struct Extrinsic {
        Eigen::Matrix3d R_BL;   // LiDAR to Body rotation
        Eigen::Vector3d t_BL;   // LiDAR to Body translation
    };

    IESKF();

    // IMU prediction (high rate: 200Hz)
    void predict(const IMUData& imu, double dt);

    // IESKF update (LiDAR rate: 10-20Hz)
    void update(const std::vector<PointData>& points);

    // Get current state
    const State& getState() const { return state_; }

    // Setters
    void setExtrinsic(const Extrinsic& ext) { extrinsic_ = ext; }
    void setGravity(const Eigen::Vector3d& g) { gravity_ = g; }

private:
    // State
    State state_;
    Eigen::Matrix<double, 15, 15> P_;  // Error-state covariance

    // Parameters
    Extrinsic extrinsic_;
    Eigen::Vector3d gravity_{0, 0, -9.81};  // NED frame

    // Noise parameters
    struct NoiseParams {
        double gyro_arw;      // Angle random walk (rad/√Hz)
        double accel_vrw;     // Velocity random walk (m/s²/√Hz)
        double gyro_bias_rw;  // Gyro bias random walk
        double accel_bias_rw; // Accel bias random walk
    } noise_;

    // IESKF parameters
    struct IESKFParams {
        int max_iterations = 3;
        double convergence_eps = 1e-3;
        double residual_threshold = 0.1;
    } params_;

    // Helper functions
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) const;
    Eigen::Matrix3d SO3Exp(const Eigen::Vector3d& omega) const;

    // State transition
    Eigen::Matrix<double, 15, 15> computeStateTransition(double dt);

    // Measurement Jacobian
    void computeMeasurementJacobian(
        const PointData& point,
        const State& state,
        Eigen::Matrix<double, 1, 15>& H,
        double& residual);

    // Point-to-plane error
    double computePointPlaneError(
        const Eigen::Vector3d& p_lidar,
        const State& state,
        const Eigen::Vector3d& plane_n,
        const Eigen::Vector3d& plane_q);
};

// ============================================
// Implementation
// ============================================

IESKF::IESKF() {
    // Initialize state
    state_.R = Eigen::Matrix3d::Identity();
    state_.p = Eigen::Vector3d::Zero();
    state_.v = Eigen::Vector3d::Zero();
    state_.b_a = Eigen::Vector3d::Zero();
    state_.b_w = Eigen::Vector3d::Zero();

    // Initialize covariance
    P_ = Eigen::Matrix<double, 15, 15>::Identity();
    P_.block<3, 3>(0, 0) *= std::pow(5.0 * M_PI / 180.0, 2);   // theta
    P_.block<3, 3>(3, 3) *= std::pow(0.1, 2);                  // p
    P_.block<3, 3>(6, 6) *= std::pow(0.1, 2);                  // v
    P_.block<3, 3>(9, 9) *= std::pow(0.1, 2);                  // b_a
    P_.block<3, 3>(12, 12) *= std::pow(0.01, 2);               // b_w
}

void IESKF::predict(const IMUData& imu, double dt) {
    // Bias-corrected measurements
    Eigen::Vector3d a_hat = imu.accel - state_.b_a;
    Eigen::Vector3d w_hat = imu.gyro - state_.b_w;

    // State propagation (Midpoint or Euler)
    Eigen::Vector3d v_dot = state_.R * a_hat + gravity_;

    state_.p += state_.v * dt + 0.5 * v_dot * dt * dt;
    state_.v += v_dot * dt;
    state_.R = state_.R * SO3Exp(w_hat * dt);

    // Covariance propagation
    auto Phi = computeStateTransition(dt);

    // Discrete noise
    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    Q.block<3, 3>(0, 0) = std::pow(noise_.gyro_arw, 2) * dt * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(6, 6) = std::pow(noise_.accel_vrw, 2) * dt * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(9, 9) = std::pow(noise_.accel_bias_rw, 2) * dt * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(12, 12) = std::pow(noise_.gyro_bias_rw, 2) * dt * Eigen::Matrix3d::Identity();

    P_ = Phi * P_ * Phi.transpose() + Q;

    // Symmetrize
    P_ = 0.5 * (P_ + P_.transpose());
}

Eigen::Matrix<double, 15, 15> IESKF::computeStateTransition(double dt) {
    Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity();

    // Bias-corrected accel (from previous state)
    // Note: In practice, store from predict step

    // Rotation error dynamics: delta_theta_dot = -[omega]x * delta_theta - delta_b_w
    // For first-order: Phi_00 = I - [omega]x * dt
    // Simplified: assume Phi_00 ≈ I for small dt

    Phi.block<3, 3>(3, 6) = dt * Eigen::Matrix3d::Identity();  // p-v coupling
    // Additional cross terms based on current state...

    return Phi;
}

void IESKF::update(const std::vector<PointData>& points) {
    // Filter valid points
    std::vector<PointData> valid_points;
    for (const auto& pt : points) {
        if (pt.valid && std::abs(pt.residual) < params_.residual_threshold) {
            valid_points.push_back(pt);
        }
    }

    if (valid_points.empty()) return;

    const int N = valid_points.size();

    // IESKF Iteration
    State state_iter = state_;

    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        // Stack Jacobians and residuals
        Eigen::MatrixXd H(N, 15);
        Eigen::VectorXd r(N);

        for (int i = 0; i < N; ++i) {
            computeMeasurementJacobian(valid_points[i], state_iter,
                                       H.row(i), r(i));
        }

        // Measurement noise (point-to-plane)
        double sigma_r = 0.05;  // 5cm point-to-plane noise
        Eigen::MatrixXd R = sigma_r * sigma_r * Eigen::MatrixXd::Identity(N, N);

        // Kalman gain
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(N, N));

        // State update
        Eigen::Matrix<double, 15, 1> dx = K * r;

        // Convergence check
        if (dx.norm() < params_.convergence_eps) {
            break;
        }

        // Update iterate (manifold)
        state_iter.update(dx);
    }

    // Final state update
    state_ = state_iter;

    // Covariance update (Joseph form)
    Eigen::MatrixXd H_final(N, 15);
    for (int i = 0; i < N; ++i) {
        double dummy;
        computeMeasurementJacobian(valid_points[i], state_, H_final.row(i), dummy);
    }

    Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() -
                           P_ * H_final.transpose() *
                           (H_final * P_ * H_final.transpose() +
                            0.0025 * Eigen::MatrixXd::Identity(N, N)).ldlt().solve(H_final);

    P_ = I_KH * P_ * I_KH.transpose() +
         P_ * H_final.transpose() * 0.0025 *
         (H_final * P_ * H_final.transpose() + 0.0025 * Eigen::MatrixXd::Identity(N, N)).ldlt().solve(H_final) * P_;

    // Symmetrize and clamp
    P_ = 0.5 * (P_ + P_.transpose());
    P_ = P_.cwiseMax(1e-12).cwiseMin(1e6);
}

void IESKF::computeMeasurementJacobian(
    const PointData& point,
    const State& state,
    Eigen::Matrix<double, 1, 15>& H,
    double& residual) {

    // Transform point to body frame
    Eigen::Vector3d p_body = extrinsic_.R_BL * point.point_lidar + extrinsic_.t_BL;

    // Point in world
    Eigen::Vector3d p_world = state.R * p_body + state.p;

    // Point-to-plane residual
    residual = point.plane_normal.dot(p_world - point.plane_point);

    // Jacobian w.r.t. rotation (left perturbation)
    // d(R*p_body)/d(delta_theta) = -R * [p_body]x
    Eigen::Matrix3d p_body_skew = skewSymmetric(p_body);
    Eigen::Vector3d dres_dtheta = -point.plane_normal.transpose() * state.R * p_body_skew;

    // Jacobian w.r.t. position
    Eigen::Vector3d dres_dp = point.plane_normal;

    // Full Jacobian (only R and p are observable from point-to-plane)
    H.setZero();
    H.segment<3>(0) = dres_dtheta;   // theta
    H.segment<3>(3) = dres_dp;        // p
    // H.segment<3>(6) = 0;            // v (not observable)
    // H.segment<3>(9) = 0;            // b_a (not observable)
    // H.segment<3>(12) = 0;           // b_w (not observable)
}

Eigen::Matrix3d IESKF::skewSymmetric(const Eigen::Vector3d& v) const {
    Eigen::Matrix3d M;
    M << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return M;
}

Eigen::Matrix3d IESKF::SO3Exp(const Eigen::Vector3d& omega) const {
    double theta = omega.norm();
    Eigen::Matrix3d R;

    if (theta < 1e-6) {
        R = Eigen::Matrix3d::Identity() + skewSymmetric(omega);
    } else {
        Eigen::Vector3d k = omega / theta;
        Eigen::Matrix3d K = skewSymmetric(k);
        R = Eigen::Matrix3d::Identity() + std::sin(theta) * K +
            (1 - std::cos(theta)) * K * K;
    }
    return R;
}

} // namespace fast_lio
```

---

## 7. Jacobian Computation Checklist

### 7.1 Pre-computation Checklist

- [ ] **Extrinsic calibration**: T_BL đã calibrated chính xác?
- [ ] **Frame convention**: X-Forward, Y-Left, Z-Up (hoặc NED)?
- [ ] **Perturbation convention**: Left hay Right? (document rõ)
- [ ] **Gravity vector**: Đã set đúng theo convention?

### 7.2 Jacobian Verification

| Jacobian | Formula | Verification Method |
|----------|---------|---------------------|
| `∂(R·v)/∂δθ` | -R·[v]× | Numerical: `(Exp(δθ)·R·v - R·v)/‖δθ‖` |
| `∂p_world/∂δp` | I₃ | Direct check |
| `∂p_world/∂δθ` | -R·[p_body]× | Check skew-symmetry |
| `∂residual/∂δv` | 0 | Point-to-plane không phụ thuộc velocity |
| `∂residual/∂δb` | 0 | Point-to-plane không phụ thuộc bias |

### 7.3 Numerical Verification

```cpp
// Numerical Jacobian verification
template<int Rows, int Cols>
void verifyJacobian(
    const std::function<Eigen::Matrix<double, Rows, 1>(const Eigen::Matrix<double, Cols, 1>&)>& func,
    const Eigen::Matrix<double, Cols, 1>& x,
    const Eigen::Matrix<double, Rows, Cols>& analytical_jacobian,
    double eps = 1e-6) {

    Eigen::Matrix<double, Rows, Cols> numerical;
    for (int i = 0; i < Cols; ++i) {
        Eigen::Matrix<double, Cols, 1> x_plus = x;
        x_plus(i) += eps;

        Eigen::Matrix<double, Cols, 1> x_minus = x;
        x_minus(i) -= eps;

        numerical.col(i) = (func(x_plus) - func(x_minus)) / (2 * eps);
    }

    double error = (analytical_jacobian - numerical).norm();
    if (error > 1e-4) {
        std::cerr << "Jacobian verification FAILED! Error: " << error << std::endl;
    }
}
```

---

## 8. Implementation Recommendations

### 8.1 UAV-Specific Optimizations

1. **Gravity Alignment**: Sử dụng 15-DOF, bỏ gravity estimation
2. **Extrinsic**: Calibrate T_BL offline, fix trong runtime
3. **Feature Selection**: Selective point cloud downsampling
4. **Temporal Alignment**: Synchronize IMU và LiDAR timestamps

### 8.2 MID-360 Specific

1. **Scan format**: Ring-based processing
2. **IMU-LiDAR sync**: MID-360 có built-in sync
3. **Field of view**: 360° x 59° → optimize point selection
4. **Range**: 0.1-70m → different noise model theo range

### 8.3 Performance Targets

| Metric | Target | Note |
|--------|--------|------|
| IMU integration | 200 Hz | ~5ms interval |
| LiDAR update | 10-20 Hz | ~50-100ms |
| IESKF iteration | < 5ms | Real-time constraint |
| Max iterations | 3 | Early termination |
| Position accuracy | < 5cm RMSE | Outdoor test |
| Orientation accuracy | < 0.5° RMSE | Heading |

---

## 9. Common Pitfalls & Solutions

### 9.1 State Representation

| Pitfall | Solution |
|---------|----------|
| Quaternion vs Rotation matrix | Use Rotation matrix cho IESKF, convert to quaternion nếu cần output |
| Left vs Right perturbation | Document rõ, dùng Left cho consistency với FAST-LIO2 |
| Gravity sign | NED: g = [0,0,-9.81], ENU: g = [0,0,9.81] |

### 9.2 Numerical Issues

| Pitfall | Solution |
|---------|----------|
| Covariance drift | Joseph form update + clamping |
| SO(3) constraint violation | Manifold update, re-normalize rotation |
| Singular covariance | Add small epsilon to diagonal |
| Measurement outlier | Robust kernel (Huber/Cauchy) |

### 9.3 Convergence

| Pitfall | Solution |
|---------|----------|
| Không convergence | Giảm step size, check residual |
| Slow convergence | Tune noise parameters |
| Divergence | Check Jacobians, measurement validity |

---

## 10. References

1. **FAST-LIO2**: Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry"
2. **ESKF**: Bloesch et al., "Iterated Extended Kalman Filter on Manifolds"
3. **SO(3)**: Sola et al., "A micro Lie theory for state estimation in robotics"
4. **Point-to-Plane**: Low, "Linear Least-Squares Optimization for Point-to-Plane ICP"
5. **MID-360**: Livox datasheet & calibration guide

---

## Summary

### Đề xuất cho UAV + MID-360:

1. **State**: 15-DOF `[R, p, v, b_a, b_ω]` (bỏ gravity)
2. **Perturbation**: Left SO(3) error-state
3. **Propagation**: Error-state EKF với continuous-time discretization
4. **Update**: IESKF với max 3 iterations
5. **Jacobian**: Point-to-plane chỉ phụ thuộc R và p
6. **Convergence**: `‖δx‖ < 1e-3`
7. **Real-time**: < 5ms per update

**Critical Checklist trước khi flight test:**
- [ ] Jacobian numerically verified
- [ ] Gravity direction confirmed
- [ ] Extrinsic calibration validated
- [ ] Covariance tuning ( Monte Carlo simulation)
- [ ] Indoor test: hover, waypoint, return-to-home
- [ ] Outdoor test: wind, GPS-denied environment

---

*Document version: 1.0*
*Reviewed by: State Estimation Expert*
*Next review: After first flight test*
