#include <gtest/gtest.h>

#include <csignal>
#include <exception>
#include <stdexcept>

#include "navigation_runtime/mapping_fail_stop.hpp"

namespace navigation_runtime {
namespace {

TEST(MappingFailStop, AbortsTheProcessAndEmitsTheMutationFailureReason) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      mappingFailStop(std::make_exception_ptr(
          std::runtime_error("injected failure after mutable map update"))),
      ::testing::KilledBySignal(SIGABRT),
      "FATAL: MappingWorker failed after mutable ROG processing began: "
      "injected failure after mutable map update");
}

TEST(MappingFailStop, AbortsForNonStandardExceptions) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      mappingFailStop(std::make_exception_ptr(42)),
      ::testing::KilledBySignal(SIGABRT),
      "FATAL: MappingWorker failed with a non-standard exception");
}

}  // namespace
}  // namespace navigation_runtime
