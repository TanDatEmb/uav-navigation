#pragma once

#include <concepts>
#include <expected>
#include <string_view>
#include <type_traits>

namespace uavnav {

/// Success value or a typed failure reason. Failures are values, never exceptions.
template <class T, class R>
using Result = std::expected<T, R>;

/// A reason is a scoped enum with a free `to_string(E) -> std::string_view`
/// found by ADL, so every failure can be logged and reported by name.
template <class E>
concept ReasonEnum = std::is_enum_v<E> && requires(E e) {
  { to_string(e) } -> std::same_as<std::string_view>;
};

}  // namespace uavnav
