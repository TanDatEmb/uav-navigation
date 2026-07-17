#ifndef FAST_LIO_IESKF_HPP_
#define FAST_LIO_IESKF_HPP_

#include "fast_lio/commons.hpp"
#include "fast_lio/estimator/estimator.hpp"

#include <Eigen/Dense>
#include <functional>

namespace fast_lio {

/**
 * @brief Iterated Error-State Kalman Filter (15-DOF)
 *
 * Mathematical baseline (see docs/ieskf_design.md):
 *   - State: x = [R_wb, p_w, v_w, b_a, b_ω] ∈ SO(3) × ℝ¹²
 *   - Right perturbation on SO(3): R' = R * Exp(δθ)
 *   - Prediction: continuous-time IMU error-state EKF with midpoint integration
 *   - Update: information-form IESKF; 15×15 factorization avoids N×N scaling
 *
 * Implements the project-wide Estimator interface.
 */
class IESKF : public Estimator {
   public:
    /**
     * @brief Constructor with UAV-optimized defaults
     *
     * Default noise values for MID-360 ICM-40609 IMU
     */
    IESKF();

    void configure(const Config& config) override;

    void setState(const State15& state) override {
        x_ = state;
    }
    State15 getState() const override {
        return x_;
    }

    /**
     * @brief Prediction step (IMU propagation)
     *
     * Propagates state using IMU measurements.
     * Continuous-time error-state EKF with midpoint integration.
     */
    void predict(const IMUData& imu, double dt) override;

    /**
     * @brief IESKF update step (iterated)
     *
     * Information-form iterated update using the SharedState15 supplied by the
     * installed measurement callback.
     */
    EstimatorUpdateResult update(int max_iterations) override;

    void setMeasurementCallback(MeasurementCallback callback) override {
        measurement_callback_ = std::move(callback);
    }

    void reset() override;

    void initWithGravity(const Eigen::Vector3d& mean_acc) override;

    Eigen::Matrix<double, 15, 15> getCovariance() const override {
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
    static inline const Eigen::Vector3d GRAVITY_WORLD_{0.0, 0.0, -9.80665};

    // Measurement callback installed by LidarProcessor.
    MeasurementCallback measurement_callback_;

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
