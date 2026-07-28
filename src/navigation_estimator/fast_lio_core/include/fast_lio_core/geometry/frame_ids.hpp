#pragma once

#include "fast_lio_core/geometry/frame.hpp"

namespace uav::nav::lio {

[[nodiscard]] inline FrameId odomFrame() { return FrameId("odom"); }
[[nodiscard]] inline FrameId baseFrame() { return FrameId("base_link"); }
[[nodiscard]] inline FrameId imuFrame() { return FrameId("imu_link"); }
[[nodiscard]] inline FrameId lidarFrame() { return FrameId("lidar_link"); }

}  // namespace uav::nav::lio
