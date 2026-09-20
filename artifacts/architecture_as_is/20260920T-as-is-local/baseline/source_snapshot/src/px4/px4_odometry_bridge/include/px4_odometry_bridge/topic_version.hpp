#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>

namespace px4_odometry_bridge {

template <typename Message, typename = void>
struct message_version : std::integral_constant<std::uint32_t, 0> {};

template <typename Message>
struct message_version<Message, std::void_t<decltype(Message::MESSAGE_VERSION)>>
    : std::integral_constant<std::uint32_t, Message::MESSAGE_VERSION> {};

template <typename Message>
std::string versioned_topic(std::string_view base_topic) {
  constexpr auto version = message_version<Message>::value;
  if constexpr (version == 0) {
    return std::string(base_topic);
  } else {
    return std::string(base_topic) + "_v" + std::to_string(version);
  }
}

}  // namespace px4_odometry_bridge
