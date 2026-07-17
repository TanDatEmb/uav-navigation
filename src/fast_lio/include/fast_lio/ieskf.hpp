#ifndef FAST_LIO_IESKF_HPP_
#define FAST_LIO_IESKF_HPP_

#include "fast_lio/commons.hpp"

#include <Eigen/Dense>
#include <functional>

namespace fast_lio {

// State and SharedState15 already defined in commons.hpp

/**
 * @brief Status of IESKF update operation.
 */
enum class IeskfUpdateStatus {
    kSuccess = 0,                    ///< Update succeeded, state corrected
    kNoMeasurements = 1,             ///< No valid measurements provided
    kInsufficientMeasurements = 2,   ///< Below minimum correspondence threshold
    kPriorFactorizationFailure = 3,  ///< Prior covariance factorization failed
    kMeasurementFactorizationFailure = 4,  ///< Measurement info factorization failed
    kNonFiniteCorrection = 5,        ///< Correction delta contains NaN/Inf
    kFinalLinearizationFailure = 6,  ///< Final covariance linearization failed
};

/**
 * @brief Result of IESKF update operation.
 */
struct IeskfUpdateResult {
    IeskfUpdateStatus status = IeskfUpdateStatus::kNoMeasurements;
    bool converged = false;               ///< Whether iteration converged
    std::size_t iterations = 0;           ///< Number of iterations executed
    std::size_t measurements = 0;          ///< Number of accepted measurements
    double final_delta_rotation_rad = 0.0;  ///< Final rotation correction norm [rad]
    double final_delta_position_m = 0.0;      ///< Final position correction norm [m]
    
    [[nodiscard]] bool success() const {
        return status == IeskfUpdateStatus::kSuccess;
    }
};

/**
 * @brief Iterated Error-State Kalman Filter (15-DOF)
 *
 * Mathematical baseline (see docs/ieskf_design.md):
 *   - State: x = [R_wb, p_w, v_w, b_a, b_ω] ∈ SO(3) × ℝ¹²
 *   - Right perturbation on SO(3): R' = R * Exp(δθ)
 *   - Prediction: continuous-time IMU error-state EKF with midpoint integration
 *   - Update: information-form IESKF; 15×15 factorization avoids N×N scaling
 *
 * Real-time target: < 5ms per update (10-20Hz LiDAR)
 */
class IESKF {
   public:
    /**
     * @brief Constructor with UAV-optimized defaults
     *
     * Default noise values for MID-360 ICM-40609 IMU
     */
    IESKF();

    /**
     * @brief Apply filter noise and LiDAR measurement information from config.
     */
    void configure(const Config& config);

    // Set/get state
    void setState(const State15& state) {
        x_ = state;
    }
    const State15& getState() const {
        return x_;
    }
    State15& getState() {
        return x_;
    }

    /**
     * @brief Prediction step (IMU propagation)
     *
     * Propagates state using IMU measurements.
     * Continuous-time error-state EKF with midpoint integration:
     *   a_unbiased = a - b_a,   ω_unbiased = ω - b_ω
     *   R ← R * Exp(ω_unbiased * Δt)
     *   a_world = R * a_unbiased + g
     *   p ← p + v * Δt + 0.5 * a_world * Δt²
     *   v ← v + a_world * Δt
     *
     * Error-state transition: Φ = I + F * Δt
     * Process noise: Q = diag(σ_ω²Δt, 0, σ_a²Δt, σ_ba²Δt, σ_bω²Δt)
     *
     * @param imu IMU measurement [accel, gyro] in body frame
     * @param dt Time step in seconds
     */
    void predict(const IMUData& imu, double dt);

    /**
     * @brief IESKF update step (iterated)
     *
     * Information-form iterated update:
     *   Λ_prior = P⁻¹
     *   Λ = Λ_prior + λ * HᵀH
     *   δx = Λ⁻¹ * (-λ * Hᵀ * r - Λ_prior * accumulated_delta)
     *   x ← x ⊕ δx
     * Iterates until convergence (‖δθ‖ < threshold, ‖δp‖ < threshold).
     *
     * @param max_iterations Maximum IESKF iterations (default: 3)
     * @return Update result with status and diagnostics
     */
    IeskfUpdateResult update(int max_iterations = 3);

    // Set loss function for point-to-plane constraints
    // Signature: void(State15&, SharedState15&)
    void setLossFunction(std::function<void(const State15&, SharedState15&)> func);

    // Set convergence check
    // Returns true if converged (‖δx‖ < threshold)
    void setConvergenceCheck(std::function<bool(const V15D&)> func);

    // Reset filter to initial state
    void reset();

    /**
     * @brief Initialize with gravity alignment
     *
     * Uses mean accelerometer reading to estimate initial attitude.
     * Only valid when sensor is stationary.
     *
     * @param mean_acc Mean accelerometer reading [m/s²] in body frame
     */
    void initWithGravity(const Eigen::Vector3d& mean_acc);

    // Get covariance for debugging/health monitoring
    const Eigen::Matrix<double, 15, 15>& getCovariance() const {
        return P_;
    }

   private:
    // Current state estimate
    State15 x_;

    // Error-state covariance (15×15)
    Eigen::Matrix<double, 15, 15> P_;

    // Process noise (continuous-time)
    struct NoiseParams {
        double gyro_noise;     // σ_ω [rad/s/√Hz]
        double accel_noise;    // σ_a [m/s²/√Hz]
        double gyro_bias_rw;   // σ_bω [rad/s/√Hz]
        double accel_bias_rw;  // σ_ba [m/s²/√Hz]
    } noise_params_;

    double lidar_information_{200.0};

    // Gravity in the z-up LIO world frame.
    // Use inline static instead of constexpr for Eigen
    static inline const Eigen::Vector3d GRAVITY_WORLD_{0.0, 0.0, -9.80665};

    // Callbacks
    std::function<void(const State15&, SharedState15&)> loss_func_;
    std::function<bool(const V15D&)> convergence_check_;

    // Default convergence check
    static bool defaultConvergenceCheck(const V15D& delta_x);

    // IMU propagation helpers
    void propagateState(const IMUData& imu, double dt);
    Eigen::Matrix<double, 15, 15> computeStateTransition(const IMUData& imu, double dt) const;
    Eigen::Matrix<double, 15, 15> computeProcessNoise(const IMUData& imu, double dt) const;

    // Skew-symmetric matrix
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
};

}  // namespace fast_lio

#endif  // FAST_LIO_IESKF_HPP_
