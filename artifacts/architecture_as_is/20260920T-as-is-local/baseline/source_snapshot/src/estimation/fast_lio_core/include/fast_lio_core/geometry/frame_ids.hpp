#pragma once

#include "fast_lio_core/geometry/frame.hpp"

namespace uav::nav::lio {

[[nodiscard]] inline FrameId lioOdomFrame() { return FrameId("lio_odom"); }
[[nodiscard]] inline FrameId baseFrame() { return FrameId("base_link"); }
[[nodiscard]] inline FrameId imuFrame() { return FrameId("livox_imu_frame"); }
[[nodiscard]] inline FrameId lidarFrame() { return FrameId("livox_frame"); }

}  // namespace uav::nav::lio
