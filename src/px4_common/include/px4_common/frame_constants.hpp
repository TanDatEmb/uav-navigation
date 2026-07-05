#ifndef PX4_COMMON_FRAME_CONSTANTS_HPP_
#define PX4_COMMON_FRAME_CONSTANTS_HPP_

#include <string_view>

namespace px4_common::frame {

/// PX4 local world frame: North-East-Down.
inline constexpr std::string_view kMapNed = "map_ned";

/// FAST-LIO2 initialization frame.
inline constexpr std::string_view kCameraInit = "camera_init";

/// ROS REP-103 body frame: Forward-Left-Up.
inline constexpr std::string_view kBaseLink = "base_link";

/// PX4 body frame: Forward-Right-Down.
inline constexpr std::string_view kAircraft = "aircraft";

/// Sensor frame for LiDAR/depth in ROS conventions.
inline constexpr std::string_view kSensor = "sensor";

}  // namespace px4_common::frame

#endif  // PX4_COMMON_FRAME_CONSTANTS_HPP_
