// Copyright 2026 TanDatEmb.
//
// IKFoM-based 15-DOF iterated error-state Kalman filter baseline.
//
// This class wraps the HKUST IKFoM manifold toolkit
// (https://github.com/hku-mars/IKFoM, commit 59cfc09) so the FAST-LIO2
// propagation and update pipeline can be expressed on the same 15-DOF state
// used by the existing custom IESKF. The public API intentionally mirrors
// IESKF to allow drop-in replacement once the point-to-plane measurement
// model is wired in Commit 6.

#ifndef FAST_LIO_ESTIMATOR_IKFOM_ESTIMATOR_HPP_
#define FAST_LIO_ESTIMATOR_IKFOM_ESTIMATOR_HPP_

#include "fast_lio/commons.hpp"
#include "fast_lio/estimator/ikfom_state.hpp"

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

#include <functional>
#include <memory>

namespace fast_lio {

/**
 * @brief Callback that provides point-to-plane measurements for the IKFoM update.
 *
 * The estimator calls this provider once per iteration with the current state
 * estimate. The provider must fill:
 *   - H:         N × 15 Jacobian of the residual w.r.t. the IKFoM error state
 *                order [pos, rot, vel, bg, ba].
 *   - residuals: N × 1 signed point-to-plane residuals.
 *   - R:         N × N measurement noise covariance.
 *
 * Return false when no valid measurements are available.
 */
using IkfomMeasurementProvider =
    std::function<bool(const State15&, Eigen::MatrixXd&, Eigen::VectorXd&,
                       Eigen::MatrixXd&)>;

// Forward-declared PIMPL storage so the esekf template stays out of the header.
struct IkfomEstimatorImpl;

/**
 * @brief Status of an IKFoM estimator update operation.
 */
enum class IkfomUpdateStatus {
    kSuccess = 0,
    kNoMeasurements = 1,
    kInsufficientMeasurements = 2,
    kNotImplemented = 3,
};

/**
 * @brief Result of an IKFoM estimator update operation.
 */
struct IkfomUpdateResult {
    IkfomUpdateStatus status = IkfomUpdateStatus::kNoMeasurements;
    bool converged = false;
    std::size_t iterations = 0;
    std::size_t measurements = 0;
    double final_delta_rotation_rad = 0.0;
    double final_delta_position_m = 0.0;

    [[nodiscard]] bool success() const {
        return status == IkfomUpdateStatus::kSuccess;
    }
};

/**
 * @brief IKFoM-based iterated error-state Kalman filter (15-DOF).
 *
 * State: [R, p, v, b_a, b_ω] ∈ SO(3) × ℝ¹²
 * Right perturbation on SO(3), constant gravity in the z-up LIO world frame.
 */
class IkfomEstimator {
   public:
    IkfomEstimator();
    ~IkfomEstimator();

    // Apply filter noise parameters from the project-wide Config.
    void configure(const Config& config);

    // Set/get state in the project-wide State15 representation.
    void setState(const State15& state);
    State15 getState() const;

    /**
     * @brief IMU propagation step.
     *
     * Uses IKFoM's manifold propagation with the 15-DOF process model:
     *   ṗ = v
     *   Ṙ = R * [(ω - b_ω)^∧]
     *   v̇ = R * (a - b_a) + g
     *   ḃ_a = 0
     *   ḃ_ω = 0
     */
    void predict(const IMUData& imu, double dt);

    /**
     * @brief Install the measurement provider used by update().
     *
     * The provider is normally supplied by LidarProcessor and captures the
     * current downsampled cloud and the local map tree.
     */
    void setMeasurementProvider(IkfomMeasurementProvider provider);

    /**
     * @brief Iterated LiDAR update using point-to-plane measurements.
     *
     * Runs IKFoM's iterated error-state EKF update. The measurement provider
     * must already be installed; otherwise the update returns kNoMeasurements.
     */
    IkfomUpdateResult update(int max_iterations = 3);

    void reset();

    /**
     * @brief Initialize attitude from mean accelerometer reading (stationary).
     */
    void initWithGravity(const Eigen::Vector3d& mean_acc);

    // Get covariance for debugging/health monitoring (15×15 error state).
    Eigen::Matrix<double, 15, 15> getCovariance() const;

   private:
    std::unique_ptr<IkfomEstimatorImpl> impl_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_ESTIMATOR_IKFOM_ESTIMATOR_HPP_
