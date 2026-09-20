#include "fast_lio_core/sensor/imu_sample.hpp"

namespace uav::nav::lio {

Status ImuSample::validate() const {
  if (time.nanoseconds() <= 0) {
    return Status(StatusCode::kInvalidArgument, "IMU sample timestamp must be positive");
  }
  if (!angular_velocity_imu_rad_s.allFinite() || !linear_acceleration_imu_m_s2.allFinite()) {
    return Status(StatusCode::kInvalidArgument, "IMU sample contains non-finite values");
  }
  return Status::Ok();
}

}  // namespace uav::nav::lio
