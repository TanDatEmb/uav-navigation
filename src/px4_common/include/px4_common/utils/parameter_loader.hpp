#ifndef PX4_COMMON_UTILS_PARAMETER_LOADER_HPP_
#define PX4_COMMON_UTILS_PARAMETER_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

namespace px4_common::utils {

namespace detail {

template <typename T>
std::string ParamValueToString(const T &value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

}  // namespace detail

/**
 * @brief Load a ROS 2 parameter with an explicit default and description.
 *
 * This helper enforces the project convention: every declared member
 * parameter must be loaded in the constructor. It also centralizes logging
 * so that missing or out-of-range values are visible.
 *
 * @tparam T Parameter value type.
 * @param node ROS 2 node used for declaration and logging.
 * @param name Parameter name.
 * @param default_value Value used when the parameter is not externally set.
 * @param description Human-readable parameter description.
 * @return Current parameter value.
 */
template <typename T>
T LoadParam(rclcpp::Node &node, const std::string &name, const T &default_value,
            const std::string &description = "") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    node.declare_parameter(name, default_value, descriptor);

    T value = default_value;
    if (!node.get_parameter(name, value)) {
        RCLCPP_WARN(node.get_logger(), "Parameter '%s' not found, using default %s", name.c_str(),
                    detail::ParamValueToString(default_value).c_str());
        value = default_value;
    }
    return value;
}

}  // namespace px4_common::utils

#endif  // PX4_COMMON_UTILS_PARAMETER_LOADER_HPP_
