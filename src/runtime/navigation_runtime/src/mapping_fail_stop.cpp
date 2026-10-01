#include "navigation_runtime/mapping_fail_stop.hpp"

#include <cstdio>
#include <cstdlib>

namespace navigation_runtime {

[[noreturn]] void mappingFailStop(std::exception_ptr failure) noexcept {
  try {
    if (failure) std::rethrow_exception(failure);
  } catch (const std::exception& error) {
    std::fprintf(stderr,
        "FATAL: MappingWorker failed after mutable ROG processing began: %s\n",
        error.what());
  } catch (...) {
    std::fprintf(stderr,
        "FATAL: MappingWorker failed with a non-standard exception\n");
  }
  std::fflush(stderr);
  std::abort();
}

}  // namespace navigation_runtime
