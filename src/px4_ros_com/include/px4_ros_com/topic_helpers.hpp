/**
 * @brief Helpers for building PX4 DDS topic names.
 *
 * PX4 message topic names may gain a `_vN` suffix when the message
 * definition version changes. This helper appends the suffix only if the
 * message type exposes a non-zero MESSAGE_VERSION constant.
 */

#ifndef PX4_ROS_COM_TOPIC_HELPERS_HPP_
#define PX4_ROS_COM_TOPIC_HELPERS_HPP_

#include <string>
#include <type_traits>

namespace px4_ros_com::topic {

namespace detail {

template <typename T, typename = void>
struct HasMessageVersion : std::false_type {};

template <typename T>
struct HasMessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>> : std::true_type {};

}  // namespace detail

/**
 * @brief Return the version suffix for a PX4 message type.
 *
 * @tparam T px4_msgs message type.
 * @return "" if MESSAGE_VERSION is absent or zero, otherwise "_vN".
 */
template <typename T>
std::string MessageVersionSuffix() {
    if constexpr (detail::HasMessageVersion<T>::value) {
        constexpr int version = T::MESSAGE_VERSION;
        if (version == 0) {
            return "";
        }
        return "_v" + std::to_string(version);
    }
    return "";
}

/**
 * @brief Build a full PX4 DDS topic name from a base name and message type.
 *
 * @tparam T px4_msgs message type.
 * @param base_topic Topic base, e.g. "/fmu/out/vehicle_local_position".
 * @return Full topic name with version suffix appended when applicable.
 */
template <typename T>
std::string Px4TopicName(const std::string& base_topic) {
    return base_topic + MessageVersionSuffix<T>();
}

}  // namespace px4_ros_com::topic

#endif  // PX4_ROS_COM_TOPIC_HELPERS_HPP_
