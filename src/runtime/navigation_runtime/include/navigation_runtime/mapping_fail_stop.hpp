#pragma once

#include <exception>

namespace navigation_runtime {

[[noreturn]] void mappingFailStop(std::exception_ptr failure) noexcept;

}  // namespace navigation_runtime
