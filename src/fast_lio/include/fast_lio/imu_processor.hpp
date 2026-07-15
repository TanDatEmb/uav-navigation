#ifndef FAST_LIO_IMU_PROCESSOR_HPP_
#define FAST_LIO_IMU_PROCESSOR_HPP_

#include "fast_lio/commons.hpp"
#include "fast_lio/ieskf.hpp"

#include <deque>
#include <memory>

namespace fast_lio {

/**
 * @brief IMU processing: initialization, propagation, and bias estimation.
 */
class IMUProcessor {
   public:
    explicit IMUProcessor(const Config& config);

    // Initialize IMU with static measurements
    bool initialize(const std::deque<IMUData>& imu_buffer);

    // Check if initialized
    bool isInitialized() const {
        return initialized_;
    }

    // Propagate from the previous processed scan time to this scan end.
    void processIMU(std::shared_ptr<IESKF> kf, const std::deque<IMUData>& imus, double end_time);

    // Get mean IMU measurements
    V3D getMeanAcc() const {
        return mean_acc_;
    }
    V3D getMeanGyro() const {
        return mean_gyro_;
    }
    double getAccelScale() const {
        return accel_scale_;
    }

   private:
    Config config_;
    bool initialized_;

    // Sliding initialization window collected across LiDAR packages.
    V3D mean_acc_;
    V3D mean_gyro_;
    std::deque<IMUData> init_window_;
    double accel_scale_{1.0};

    // Zero-order hold state spanning consecutive LiDAR packages.
    bool integration_initialized_{false};
    double integration_time_{0.0};
    IMUData last_imu_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_IMU_PROCESSOR_HPP_
