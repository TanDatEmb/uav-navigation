#ifndef FAST_LIO_IESKF_HPP_
#define FAST_LIO_IESKF_HPP_

#include "fast_lio/commons.hpp"

#include <Eigen/Dense>
#include <functional>

namespace fast_lio {

// State and SharedState15 already defined in commons.hpp

/**
 * @brief Iterated Error-State Kalman Filter (15-DOF)
 *
 * Optimized for UAV navigation with:
 * - 15-DOF state with fixed world gravity
 * - Right perturbation on SO(3), consistent with the LiDAR Jacobian
 * - Point-to-plane LiDAR constraints
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
     * Continuous-time error-state EKF with discretization.
     *
     * @param imu IMU measurement [accel, gyro] in body frame
     * @param dt Time step in seconds
     */
    void predict(const IMUData& imu, double dt);

    /**
     * @brief IESKF update step (iterated)
     *
     * Iteratively re-linearizes at current estimate until convergence.
     * Uses loss function provided by LidarProcessor.
     *
     * @param max_iterations Maximum IESKF iterations (default: 3)
     */
    void update(int max_iterations = 3);

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
    static inline const Eigen::Vector3d GRAVITY_WORLD_{0.0, 0.0, -9.81};

    // Callbacks
    std::function<void(const State15&, SharedState15&)> loss_func_;
    std::function<bool(const V15D&)> convergence_check_;

    // Default convergence check
    static bool defaultConvergenceCheck(const V15D& delta_x);

    // IMU propagation helpers
    void propagateState(const IMUData& imu, double dt);
    Eigen::Matrix<double, 15, 15> computeStateTransition(const IMUData& imu, double dt) const;
    Eigen::Matrix<double, 15, 15> computeProcessNoise(double dt) const;

    // Skew-symmetric matrix
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
};

}  // namespace fast_lio

#endif  // FAST_LIO_IESKF_HPP_
