// Copyright 2026 TanDatEmb.
//
// Abstract estimator interface used by the FAST-LIO2 map-building pipeline.
//
// Both the legacy custom IESKF and the IKFoM-based estimator implement this
// interface so that MapBuilder, LidarProcessor, and IMUProcessor can work with
// either backend without knowing the concrete filter type.

#ifndef FAST_LIO_ESTIMATOR_ESTIMATOR_HPP_
#define FAST_LIO_ESTIMATOR_ESTIMATOR_HPP_

#include "fast_lio/commons.hpp"

#include <Eigen/Dense>
#include <functional>
#include <memory>

namespace fast_lio {

/**
 * @brief Unified status for any estimator update operation.
 */
enum class EstimatorUpdateStatus {
    kSuccess = 0,
    kNoMeasurements = 1,
    kInsufficientMeasurements = 2,
    kFailure = 3,
};

/**
 * @brief Unified result for any estimator update operation.
 */
struct EstimatorUpdateResult {
    EstimatorUpdateStatus status = EstimatorUpdateStatus::kNoMeasurements;
    bool converged = false;
    std::size_t iterations = 0;
    std::size_t measurements = 0;
    double final_delta_rotation_rad = 0.0;
    double final_delta_position_m = 0.0;

    [[nodiscard]] bool success() const {
        return status == EstimatorUpdateStatus::kSuccess;
    }
};

/**
 * @brief Abstract backend for IMU propagation and iterated LiDAR updates.
 *
 * The pipeline uses:
 *   - predict() for IMU integration
 *   - setMeasurementCallback() to install a callback that fills a SharedState15
 *     with point-to-plane measurements in the project error-state order:
 *       [δθ, δp, δv, δb_a, δb_ω]
 *   - update() to run the iterated correction
 *
 * Implementations are responsible for converting this shared representation to
 * their internal error-state ordering and math.
 */
class Estimator {
   public:
    using MeasurementCallback = std::function<bool(const State15&, SharedState15&)>;

    virtual ~Estimator() = default;

    // Apply filter noise / measurement parameters.
    virtual void configure(const Config& config) = 0;

    // Set/get the 15-DOF navigation state.
    virtual void setState(const State15& state) = 0;
    virtual State15 getState() const = 0;

    // IMU propagation step.
    virtual void predict(const IMUData& imu, double dt) = 0;

    // Initialise attitude from a stationary accelerometer reading.
    virtual void initWithGravity(const Eigen::Vector3d& mean_acc) = 0;

    // Error-state covariance (15x15) for diagnostics / health monitoring.
    virtual Eigen::Matrix<double, 15, 15> getCovariance() const = 0;

    // Install the measurement callback used by update().
    virtual void setMeasurementCallback(MeasurementCallback callback) = 0;

    // Run the iterated LiDAR update.
    virtual EstimatorUpdateResult update(int max_iterations) = 0;

    // Reset to initial state and covariance.
    virtual void reset() = 0;
};

}  // namespace fast_lio

#endif  // FAST_LIO_ESTIMATOR_ESTIMATOR_HPP_
