// Copyright 2026 TanDatEmb.
//
// IKFoM-based 15-DOF iterated error-state Kalman filter baseline.
//
// This class wraps the HKUST IKFoM manifold toolkit
// (https://github.com/hku-mars/IKFoM, commit 59cfc09) so the FAST-LIO2
// propagation and update pipeline can be expressed on the same 15-DOF state
// used by the existing custom IESKF.

#ifndef FAST_LIO_ESTIMATOR_IKFOM_ESTIMATOR_HPP_
#define FAST_LIO_ESTIMATOR_IKFOM_ESTIMATOR_HPP_

#include "fast_lio/commons.hpp"
#include "fast_lio/estimator/estimator.hpp"
#include "fast_lio/estimator/ikfom_state.hpp"

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

#include <functional>
#include <memory>

namespace fast_lio {

// Forward-declared PIMPL storage so the esekf template stays out of the header.
struct IkfomEstimatorImpl;

/**
 * @brief IKFoM-based iterated error-state Kalman filter (15-DOF).
 *
 * Implements the project-wide Estimator interface. The internal error-state
 * order is [pos, rot, vel, bg, ba]; the interface consumes SharedState15 in
 * the project order [rot, pos, vel, ba, bw] and reorders columns internally.
 */
class IkfomEstimator : public Estimator {
   public:
    IkfomEstimator();
    ~IkfomEstimator() override;

    void configure(const Config& config) override;

    void setState(const State15& state) override;
    State15 getState() const override;

    void predict(const IMUData& imu, double dt) override;

    EstimatorUpdateResult update(int max_iterations) override;

    void setMeasurementCallback(MeasurementCallback callback) override {
        measurement_callback_ = std::move(callback);
    }

    void reset() override;

    void initWithGravity(const Eigen::Vector3d& mean_acc) override;

    Eigen::Matrix<double, 15, 15> getCovariance() const override;

   private:
    std::unique_ptr<IkfomEstimatorImpl> impl_;
    MeasurementCallback measurement_callback_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_ESTIMATOR_IKFOM_ESTIMATOR_HPP_
