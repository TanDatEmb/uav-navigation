#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>

#include "uavnav/core/result.hpp"

namespace sample {
enum class Reason : std::uint8_t { kOk, kStale };
constexpr std::string_view to_string(Reason r) { return r == Reason::kOk ? "OK" : "STALE"; }
}  // namespace sample

static_assert(uavnav::ReasonEnum<sample::Reason>);
static_assert(!uavnav::ReasonEnum<int>);

TEST(Result, CarriesValueOrReason) {
  uavnav::Result<int, sample::Reason> ok = 3;
  uavnav::Result<int, sample::Reason> bad = std::unexpected(sample::Reason::kStale);
  EXPECT_EQ(*ok, 3);
  ASSERT_FALSE(bad.has_value());
  EXPECT_EQ(to_string(bad.error()), "STALE");
}
